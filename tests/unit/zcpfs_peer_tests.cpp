// The per-contact state machine of zcpfs (zephcore/lib/zcpfs/zcpfs_peer.c), run
// as whole nodes: a stand-in for the firmware around the engine (contacts, ACK
// packets, path returns, a legacy text envelope, a phone app that retries) and
// a one-hop link with loss, delay, duplication and MeshCore's duplicate table.
// The scenarios are those of lib/zcpfs/ref/tests/test_sim.py, plus the rules
// that only exist in the C: the bounded record table and the timer.
#include "test.h"
#include <algorithm>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <vector>

extern "C" {
#include "ascon.h"
#include "monocypher.h"
#include "zcpfs.h"
#include "zcpfs_internal.h"
}

namespace {
using Bytes = std::vector<uint8_t>;

enum : uint8_t { REQ = ZCPFS_PT_REQ, RESPONSE = ZCPFS_PT_RESPONSE, TXT = ZCPFS_PT_TXT, ACK = 0x03, ANON = ZCPFS_PT_ANON, PATH = ZCPFS_PT_PATH };
enum : uint8_t { ROLE_ADMIN = 1, ROLE_GUEST = 2, ROLE_KNOWN = 3 };
const uint8_t HINT_BYTE = 0x5A;

// Every contact here reports a send timeout of 5 s, so the step of SPEC 10.4 is 8 s.
const uint32_t TIMEOUT_MS = 5000, STEP = 8;
const uint32_t HS_RETRY = ZCPFS_HS_RETRY_STEPS * STEP, HS_WAIT = ZCPFS_HS_WAIT_STEPS * STEP,
               HELD = ZCPFS_HELD_STEPS * STEP, DATA_RETRY = ZCPFS_DATA_RETRY_STEPS * STEP,
               LOGIN_WAIT = ZCPFS_LOGIN_STEPS * STEP;

struct Packet { uint8_t type; Bytes payload; bool flood; };

Bytes bytes(const std::string& s) { return Bytes(s.begin(), s.end()); }
Bytes cat(Bytes a, const Bytes& b) { a.insert(a.end(), b.begin(), b.end()); return a; }
Bytes digest(const Bytes& in, size_t n = 32) {
    uint8_t out[ASCON_HASH256_LEN];
    ascon_hash256(out, in.data(), in.size());
    return Bytes(out, out + n);
}
Bytes stream(const Bytes& key, size_t n) {
    ascon_xof x;
    Bytes out(n);
    ascon_xof128_init(&x);
    ascon_xof_absorb(&x, key.data(), key.size());
    ascon_xof_squeeze(&x, out.data(), n);
    return out;
}

// ---- the legacy text envelope, shape only: 2-byte MAC, whole 16-byte blocks ----

Bytes legacy_seal(const Bytes& ss, Bytes plain) {
    plain.resize(std::max<size_t>(16, (plain.size() + 15) / 16 * 16), 0);
    const Bytes ks = stream(cat(bytes("legacy-standin"), ss), plain.size());
    for (size_t i = 0; i < plain.size(); ++i) plain[i] ^= ks[i];
    return cat(digest(cat(ss, plain), 2), plain);
}
std::optional<Bytes> legacy_open(const Bytes& ss, const Bytes& body) {
    if (body.size() < 18 || (body.size() - 2) % 16 != 0) return std::nullopt;
    Bytes ct(body.begin() + 2, body.end());
    if (digest(cat(ss, ct), 2) != Bytes(body.begin(), body.begin() + 2)) return std::nullopt;
    const Bytes ks = stream(cat(bytes("legacy-standin"), ss), ct.size());
    for (size_t i = 0; i < ct.size(); ++i) ct[i] ^= ks[i];
    return ct;
}

// ---- the text plaintext of SPEC 9.3 and the padding hint of 6.3 ----

Bytes text_plain(uint32_t timestamp, unsigned attempt, const std::string& text) {
    Bytes p{uint8_t(timestamp), uint8_t(timestamp >> 8), uint8_t(timestamp >> 16), uint8_t(timestamp >> 24), uint8_t(attempt & 3)};
    p.insert(p.end(), text.begin(), text.end());
    if (attempt > 3) { p.push_back(0); p.push_back(uint8_t(attempt)); }
    return p;
}
Bytes with_hint(uint32_t timestamp, unsigned attempt, const std::string& text) {
    const Bytes plain = text_plain(timestamp, attempt, text);
    Bytes hinted = text_plain(timestamp, attempt & 3, text);
    hinted.push_back(0);
    hinted.push_back(uint8_t(attempt > 3 ? attempt : 0));
    hinted.push_back(HINT_BYTE);
    auto blocks = [](size_t n) { return std::max<size_t>(1, (n + 15) / 16); };
    return blocks(hinted.size()) <= blocks(plain.size()) ? hinted : plain;
}
struct Text { std::string text; unsigned attempt; bool hint; size_t end; };
Text parse_text(Bytes data) {
    data.resize(data.size() + 3, 0);
    size_t end = 5;
    while (data[end] != 0) ++end;
    const unsigned ext = data[end + 1];
    return {std::string(data.begin() + 5, data.begin() + long(end)), ext > 3 ? ext : unsigned(data[4] & 3), data[end + 2] == HINT_BYTE, end};
}
Bytes legacy_ack(const Bytes& plain, const Bytes& sender) {
    return digest(cat(Bytes(plain.begin(), plain.begin() + long(parse_text(plain).end)), sender), 4);
}

// ---- one device: the engine and what the firmware does around it ----

struct Contact {
    Bytes S, ss;
    bool pinned = false, has_path = false;
    uint32_t timeout_ms = TIMEOUT_MS;
    std::vector<std::pair<Bytes, uint32_t>> legacy_expect;
};

struct Host {
    Bytes a = Bytes(32), S = Bytes(32);
    bool zc;
    const uint64_t *clock_ms;
    std::deque<Contact> contacts;
    zcpfs_drbg drbg{};
    std::vector<zcpfs_peer> records;
    zcpfs_node node{};
    zcpfs_ops ops{};
    std::vector<Packet> out;                                // what the last calls produced
    std::vector<std::pair<Bytes, std::string>> inbox;       // sender, text
    std::vector<uint32_t> confirmed;                        // tokens of acknowledged sends
    std::vector<std::string> violations;                    // rules broken inside a callback
    std::vector<zcpfs_offer> offers;                        // a server's unconfirmed logins
    std::string password, guest_password;                   // a server's
    std::set<Bytes> acl;                                    // clients that may log in with an empty password
    std::vector<std::pair<Bytes, uint8_t>> logged_in;       // server: client, role granted
    std::vector<std::pair<Bytes, Bytes>> logins;            // client: server, login response
    std::map<Bytes, uint32_t> last_login;                   // server: timestamp of the last login answered, per client
    uint32_t login_wait = 0;
    int logins_sent = 0, logins_dropped = 0;
    uint32_t token = 0;
    uint32_t held_for = 0;                                  // seconds the last send is held, 0 if it was not
    bool in_send = false;
    int floods_in_send = 0;
    int app_attempts = 0, engine_floods = 0, legacy_sent = 0, sealed_sent = 0, hs1_sent = 0, hs2_sent = 0,
        probes_sent = 0, confirmations_sent = 0, reciprocal_paths = 0;

    Host(const std::string& label, bool zcpfs, size_t table, const uint64_t *clock) : zc(zcpfs), clock_ms(clock), records(table) {
        const Bytes seed = digest(bytes("zcpfs-sim-identity:" + label));
        crypto_eddsa_trim_scalar(a.data(), seed.data());
        crypto_eddsa_scalarbase(S.data(), a.data());
        ops.send = op_send;
        ops.reciprocal_path = op_reciprocal_path;
        ops.pin = op_pin;
        ops.contact = op_contact;
        ops.granted = op_granted;
        boot(label);
    }
    Host(const Host&) = delete;

    uint32_t now() const { return uint32_t(*clock_ms / 1000); }

    // Sessions and timers are RAM; contacts, pins and paths survive.
    void boot(const std::string& entropy) {
        const Bytes hw = digest(bytes("hw" + entropy)), saved(32, 0);
        zcpfs_drbg_seed(&drbg, hw.data(), saved.data());
        zcpfs_node_init(&node, S.data(), a.data(), &drbg, records.data(), records.size(), &ops, this);
        if (!offers.empty()) zcpfs_node_serve(&node, offers.data(), offers.size());
        for (auto& c : contacts) c.legacy_expect.clear();
    }

    Contact& add_contact(const Bytes& peer, bool has_path) {
        Contact c;
        uint8_t x[32];
        c.S = peer;
        c.ss.resize(32);
        zcpfs_ed_to_x(x, peer.data());
        zcpfs_dh(c.ss.data(), a.data(), x);
        c.has_path = has_path;
        contacts.push_back(c);
        return contacts.back();
    }
    Contact *find(const uint8_t *peer) {
        for (auto& c : contacts) if (std::memcmp(c.S.data(), peer, 32) == 0) return &c;
        return nullptr;
    }
    Contact& contact(const Bytes& peer) { Contact *c = find(peer.data()); CHECK(c != nullptr); return *c; }
    static zcpfs_contact view(const Contact& c) { return {c.S.data(), c.ss.data(), c.pinned, c.has_path, c.timeout_ms}; }
    zcpfs_peer *record(const Bytes& peer) {
        for (auto& r : records) if (r.used && std::memcmp(r.S, peer.data(), 32) == 0) return &r;
        return nullptr;
    }

    // An advert of the peer was accepted; `tagged` is its capability tag (SPEC 6.2).
    void advert(const Bytes& peer, bool tagged) {
        Contact& c = contact(peer);
        if (!zc || c.pinned == tagged) return;
        c.pinned = tagged;
        if (!tagged) zcpfs_forget(&node, peer.data());
    }

    std::vector<Packet> take() { std::vector<Packet> p; p.swap(out); return p; }

    // -- the engine's callbacks; an exception may not cross the C frames, so rules are only noted here --

    static void op_send(void *user, const uint8_t peer[32], uint8_t type, const uint8_t *payload, size_t len, int flood) {
        Host& h = *static_cast<Host *>(user);
        if (len > ZCPFS_MAX_PAYLOAD) h.violations.push_back("a zcpfs payload is too long");
        if (type == ANON) {                 // a login: dst, the client's key, and the block rule of SPEC 7 for it
            if ((len - 35) % 16 == 0) h.violations.push_back("a login is block-aligned");
            if (payload[0] != peer[0] || std::memcmp(payload + 1, h.S.data(), 32) != 0) h.violations.push_back("wrong login header");
            if (flood && (!h.in_send || ++h.floods_in_send > 1)) h.violations.push_back("a flood outside the send point");
            h.engine_floods += flood != 0;
            h.logins_sent++;
            h.out.push_back({type, Bytes(payload, payload + len), flood != 0});
            return;
        }
        if ((len - 4) % 16 == 0) h.violations.push_back("a zcpfs payload is block-aligned");
        if (payload[0] != peer[0] || payload[1] != h.S[0]) h.violations.push_back("wrong address bytes");
        if (flood) {
            h.engine_floods++;
            if (!h.in_send || ++h.floods_in_send > 1) h.violations.push_back("a flood outside the send point");
            if (type == RESPONSE || len == 42 || len == 10) h.violations.push_back("a handshake or confirmation flooded");
        }
        if (type == REQ && len == 42) h.hs1_sent++;
        else if (type == REQ) h.probes_sent++;
        else if (type == RESPONSE) h.hs2_sent++;
        else if (len == 10) h.confirmations_sent++;
        else h.sealed_sent++;
        h.out.push_back({type, Bytes(payload, payload + len), flood != 0});
    }
    static void op_reciprocal_path(void *user, const uint8_t peer[32]) {
        Host& h = *static_cast<Host *>(user);
        h.reciprocal_paths++;
        h.send_typed(*h.find(peer), PATH, {1}, false);
    }
    static void op_pin(void *user, const uint8_t peer[32]) { static_cast<Host *>(user)->find(peer)->pinned = true; }
    static int op_contact(void *user, const uint8_t peer[32], zcpfs_contact *c) {
        const Contact *found = static_cast<Host *>(user)->find(peer);
        if (found == nullptr) return 0;
        *c = view(*found);
        return 1;
    }

    // -- sending --

    Bytes hdr(const Contact& c) const { return {c.S[0], S[0]}; }

    // A path return (or any REQ, RESPONSE, PATH): sealed towards a pinned contact
    // or when answering something that arrived sealed (SPEC 18.2), legacy otherwise.
    void send_typed(Contact& c, uint8_t type, const Bytes& plain, bool flood, bool in_kind = false) {
        if (zc && (c.pinned || in_kind)) {
            uint8_t n6[6];
            Bytes payload(plain.size() + ZCPFS_PAIR_OVERHEAD + 1);
            size_t len = 0;
            zcpfs_drbg_generate(&drbg, n6, sizeof(n6));
            if (zcpfs_pair_seal(c.ss.data(), S.data(), c.S.data(), type, n6, plain.data(), plain.size(), payload.data(), &len) != ZCPFS_OK)
                violations.push_back("pair_seal failed");
            payload.resize(len);
            out.push_back({type, payload, flood});
        } else {
            out.push_back({type, cat(hdr(c), legacy_seal(c.ss, plain)), flood});
        }
    }

    // One send attempt by the application. Returns the token it is told to wait for.
    uint32_t app_send(const Bytes& peer, const std::string& text, uint32_t timestamp, unsigned attempt, int *result = nullptr) {
        Contact& c = contact(peer);
        Bytes plain = text_plain(timestamp, attempt, text);
        int r = ZCPFS_LEGACY;
        ++token;
        ++app_attempts;
        held_for = 0;
        if (zc) {
            const zcpfs_contact v = view(c);
            in_send = true;
            floods_in_send = 0;
            r = zcpfs_send_text(&node, &v, plain.data(), plain.size(), token, now(), &held_for);
            in_send = false;
        }
        if (result != nullptr) *result = r;
        if ((r == ZCPFS_HELD) != (held_for != 0)) violations.push_back("held_for does not match the result");
        if (r == ZCPFS_LEGACY) {
            if (zc) plain = with_hint(timestamp, attempt, text);
            c.legacy_expect.push_back({legacy_ack(plain, S), token});
            if (c.legacy_expect.size() > 16) c.legacy_expect.erase(c.legacy_expect.begin());
            ++legacy_sent;
            out.push_back({TXT, cat(hdr(c), legacy_seal(c.ss, plain)), !c.has_path});
        }
        return token;
    }

    void send_ack(Contact& c, const Bytes& ack4, unsigned attempt, bool was_flood, bool sealed) {
        Bytes ack = ack4;
        if (!sealed) {                       // legacy: attempt and a random byte keep ACK packets apart
            uint8_t r;
            zcpfs_drbg_generate(&drbg, &r, 1);
            ack.push_back(uint8_t(attempt));
            ack.push_back(r);
        }
        if (was_flood) send_typed(c, PATH, cat({1}, ack), true, sealed);   // a path return carrying the ACK
        else out.push_back({ACK, ack, !c.has_path});
    }

    // -- login (SPEC 19) --

    // Make this node a server: it accepts logins with these passwords and keeps `room` unconfirmed ones.
    void serve(const std::string& admin, size_t room = 4) {
        password = admin;
        offers.assign(room, zcpfs_offer{});
        zcpfs_node_serve(&node, offers.data(), offers.size());
    }
    size_t offers_kept() const {
        return size_t(std::count_if(offers.begin(), offers.end(), [](const zcpfs_offer& o) { return o.used != 0; }));
    }

    // The application logs in. Returns what zcpfs_login_send said.
    int login(const Bytes& server, const std::string& pw, uint32_t timestamp) {
        const Contact& c = contact(server);
        const zcpfs_contact v = view(c);
        const Bytes stamp{uint8_t(timestamp), uint8_t(timestamp >> 8), uint8_t(timestamp >> 16), uint8_t(timestamp >> 24)};
        ++app_attempts;
        in_send = true;
        floods_in_send = 0;
        const int r = zcpfs_login_send(&node, &v, reinterpret_cast<const uint8_t *>(pw.data()), pw.size(), stamp.data(),
                                       stamp.size(), now(), &login_wait);
        in_send = false;
        if ((r == ZCPFS_SENT) != (login_wait != 0)) violations.push_back("wait_for does not match the result");
        return r;
    }

    // Server: an ANON_REQ for this node. Every password this client could be using gets a branch; the
    // login says whether one was typed at all, which keeps that to two.
    void on_login(const Packet& p) {
        const Bytes claimed(p.payload.begin() + 1, p.payload.begin() + 33);
        const Contact *listed = find(claimed.data());
        const bool known = acl.count(claimed) != 0 || (listed != nullptr && listed->pinned);
        zcpfs_login_req req;
        if (zcpfs_login_rx(&node, p.payload.data(), p.payload.size(), known, now(), &req) != ZCPFS_OK) {
            ++logins_dropped;
            return;
        }
        // as legacy: a client the server knows cannot have an old login of its replayed
        const uint32_t sent = uint32_t(req.plain[0]) | uint32_t(req.plain[1]) << 8 | uint32_t(req.plain[2]) << 16 | uint32_t(req.plain[3]) << 24;
        if (known && last_login.count(claimed) && sent <= last_login[claimed]) {
            ++logins_dropped;
            return;
        }
        const uint32_t t = now();
        const Bytes stamp{uint8_t(t), uint8_t(t >> 8), uint8_t(t >> 16), uint8_t(t >> 24)};
        const Bytes r_admin = cat(stamp, bytes("ad")), r_guest = cat(stamp, bytes("gu")), r_known = cat(stamp, bytes("kn"));
        const uint8_t *pw = reinterpret_cast<const uint8_t *>(password.data());
        const uint8_t *guest = reinterpret_cast<const uint8_t *>(guest_password.data());
        std::vector<zcpfs_branch> branches;
        if (req.empty_password) {
            if (acl.count(claimed)) branches.push_back({pw, 0, r_known.data(), ROLE_KNOWN});
        } else {
            branches.push_back({pw, password.size(), r_admin.data(), ROLE_ADMIN});
            if (!guest_password.empty()) branches.push_back({guest, guest_password.size(), r_guest.data(), ROLE_GUEST});
        }
        Bytes reply(ZCPFS_MAX_PAYLOAD);
        size_t len = 0;
        if (branches.empty() ||
            zcpfs_login_answer(&node, &req, branches.data(), branches.size(), TIMEOUT_MS, now(), reply.data(), &len) != ZCPFS_OK) {
            ++logins_dropped;
            return;
        }
        last_login[claimed] = sent;
        reply.resize(len);
        if (p.flood) {
            // as legacy answers a flooded login: a path return with the reply as its extra, here kind 4 and a length byte
            uint8_t n6[6];
            const Bytes plain = cat({4, uint8_t(len)}, reply);
            Bytes payload(plain.size() + ZCPFS_PAIR_OVERHEAD + 1);
            size_t sealed = 0;
            zcpfs_drbg_generate(&drbg, n6, sizeof(n6));
            if (zcpfs_pair_seal(req.ss, S.data(), req.S_c, PATH, n6, plain.data(), plain.size(), payload.data(), &sealed) != ZCPFS_OK)
                violations.push_back("the login reply does not fit a path return");
            payload.resize(sealed);
            out.push_back({PATH, payload, true});
        } else {
            const Bytes payload = cat({req.S_c[0], S[0]}, reply);
            if ((payload.size() - 4) % 16 == 0 || payload.size() == 42 || payload.size() > ZCPFS_MAX_PAYLOAD)
                violations.push_back("the login reply has a length it must not have");
            out.push_back({RESPONSE, payload, listed == nullptr || !listed->has_path});
        }
    }
    static void op_granted(void *user, const uint8_t peer[32], uint8_t id) {
        Host& h = *static_cast<Host *>(user);
        Contact *c = h.find(peer);
        if (c == nullptr) c = &h.add_contact(Bytes(peer, peer + 32), false);    // the server learns its client here
        c->pinned = true;
        h.logged_in.push_back({c->S, id});
    }

    // -- receiving --

    void receive(const Packet& p) {
        const Bytes& pl = p.payload;
        if (p.type == ACK) {
            if (pl.size() >= 4) on_ack(Bytes(pl.begin(), pl.begin() + 4), p.flood);
            return;
        }
        if (p.type == ANON) {
            if (zc && !offers.empty() && pl.size() > 35 && pl[0] == S[0]) on_login(p);
            return;
        }
        if ((p.type != REQ && p.type != RESPONSE && p.type != TXT && p.type != PATH) || pl.size() <= 4 || pl[0] != S[0]) return;
        for (auto& c : contacts)
            if (c.S[0] == pl[1] && dispatch(c, p)) return;
        if (!zc || offers.empty() || p.type != TXT) return;
        // no contact claimed it: the first packet of a client that has just logged in (SPEC 19.5)?
        zcpfs_rx_out o;
        uint8_t who[32];
        const int r = zcpfs_rx_offer(&node, pl.data(), pl.size(), now(), who, &o);
        if (r == ZCPFS_RX_NEXT) return;
        Contact *from = find(who);
        if (from == nullptr) { violations.push_back("granted did not make the client known"); return; }
        const Bytes plain(o.plain, o.plain + o.len);
        if (r == ZCPFS_RX_TEXT) inbox.push_back({from->S, parse_text(plain).text});
        if (r == ZCPFS_RX_TEXT || r == ZCPFS_RX_DUPLICATE) send_ack(*from, Bytes(o.ack4, o.ack4 + 4), 0, p.flood, true);
    }

    // SPEC 7.1: the engine first, legacy only if it says so. True if the packet was consumed.
    bool dispatch(Contact& c, const Packet& p) {
        if (zc) {
            const zcpfs_contact v = view(c);
            zcpfs_rx_out o;
            const int r = zcpfs_rx(&node, &v, p.type, p.payload.data(), p.payload.size(), p.flood, now(), &o);
            if (!std::all_of(o.plain + o.len, o.plain + sizeof(o.plain), [](uint8_t b) { return b == 0; }))
                violations.push_back("the plaintext is not zero-extended");
            const Bytes plain(o.plain, o.plain + o.len), ack4(o.ack4, o.ack4 + 4);
            switch (r) {
            case ZCPFS_RX_NEXT: return false;
            case ZCPFS_RX_CONSUMED: return true;
            case ZCPFS_RX_TEXT:
                inbox.push_back({c.S, parse_text(plain).text});
                [[fallthrough]];
            case ZCPFS_RX_DUPLICATE:
                send_ack(c, ack4, 0, p.flood, true);
                return true;
            case ZCPFS_RX_PROBE:
                send_typed(c, PATH, {1}, true, true);   // as a flooded legacy message without ACK is answered
                return true;
            case ZCPFS_RX_PAIR: return handle(c, p.type, plain, p.flood, true);
            case ZCPFS_RX_LOGIN:
                logins.push_back({c.S, plain});
                return true;
            default: break;
            }
        }
        const auto plain = legacy_open(c.ss, Bytes(p.payload.begin() + 2, p.payload.end()));
        return plain && handle(c, p.type, *plain, p.flood, false);
    }

    bool handle(Contact& c, uint8_t type, const Bytes& plain, bool flood, bool sealed) {
        if (type == PATH) {
            // The stand-in for a path return's content: a kind byte, then its extra. 1: an ACK or nothing.
            // 4: a login reply behind a length byte.
            const uint8_t kind = plain.empty() ? 1 : plain[0];
            c.has_path = true;
            if (kind == 1 && plain.size() >= 5 && std::any_of(plain.begin() + 1, plain.begin() + 5, [](uint8_t b) { return b != 0; }))
                on_ack(Bytes(plain.begin() + 1, plain.begin() + 5), false);
            if (flood) send_typed(c, PATH, {1}, false, sealed);     // the reciprocal path
            if (zc) { const zcpfs_contact v = view(c); zcpfs_on_path(&node, &v, flood, now()); }
            if (zc && kind == 4 && plain.size() >= 2 && plain.size() >= size_t(2 + plain[1])) {
                const zcpfs_contact v = view(c);
                zcpfs_rx_out o;
                if (zcpfs_rx_login_reply(&node, &v, plain.data() + 2, plain[1], flood, now(), &o) == ZCPFS_RX_LOGIN)
                    logins.push_back({c.S, Bytes(o.plain, o.plain + o.len)});
            }
            return true;
        }
        if (type != TXT) return true;
        if (zc && c.pinned) return true;      // never legacy text from a pinned contact
        const Text t = parse_text(plain);
        inbox.push_back({c.S, t.text});
        send_ack(c, legacy_ack(plain, c.S), t.attempt, flood, false);
        if (zc && t.hint) { const zcpfs_contact v = view(c); zcpfs_on_hint(&node, &v, now()); }
        return true;
    }

    void on_ack(const Bytes& ack4, bool flood) {
        uint32_t tok = 0;
        uint8_t who[32];
        Contact *from = nullptr;
        if (zc && zcpfs_on_ack(&node, ack4.data(), &tok, who)) {
            from = find(who);
        } else {
            for (auto& c : contacts)
                for (size_t i = 0; from == nullptr && i < c.legacy_expect.size(); ++i)
                    if (c.legacy_expect[i].first == ack4) {
                        tok = c.legacy_expect[i].second;
                        c.legacy_expect.erase(c.legacy_expect.begin() + long(i));
                        from = &c;
                    }
        }
        if (from == nullptr) return;
        confirmed.push_back(tok);
        if (flood && from->has_path) send_typed(*from, PATH, {1}, false);   // legacy handleReturnPathRetry
    }

    uint32_t tick() { return zc ? zcpfs_tick(&node, now()) : UINT32_MAX; }
    std::vector<Packet> tick_packets() { tick(); return take(); }

    size_t received(const Bytes& from, const std::string& text) const {
        return size_t(std::count(inbox.begin(), inbox.end(), std::make_pair(from, text)));
    }
    std::set<std::string> delivered(const Bytes& from) const {
        std::set<std::string> texts;
        for (const auto& m : inbox) if (m.first == from) texts.insert(m.second);
        return texts;
    }
    bool is_confirmed(uint32_t tok) const { return std::count(confirmed.begin(), confirmed.end(), tok) > 0; }
};

// ---- the link ----

struct Air { uint64_t t; size_t src; uint8_t type; size_t len; bool flood; };

struct Sim {
    uint64_t t = 0;                    // milliseconds
    uint32_t rng;
    double loss = 0, duplicate = 0;
    uint32_t delay_min = 300, delay_max = 1200;
    std::vector<std::unique_ptr<Host>> nodes;
    std::vector<std::deque<Bytes>> seen;           // MeshCore's duplicate table per node
    std::vector<uint64_t> next_tick;
    std::vector<Air> air;
    std::vector<Packet> tap;
    struct Event { uint64_t when; uint64_t seq; std::function<void()> fn; };
    struct Later { bool operator()(const Event& x, const Event& y) const { return x.when != y.when ? x.when > y.when : x.seq > y.seq; } };
    std::priority_queue<Event, std::vector<Event>, Later> queue;
    uint64_t seq = 0;

    explicit Sim(uint32_t seed) : rng(seed * 2654435761u + 1) {}

    double random() { return (nextRandom(rng) >> 8) * (1.0 / 16777216.0); }

    Host& node(const std::string& label, bool zc = true, size_t table = 8) {
        nodes.push_back(std::make_unique<Host>(label, zc, table, &t));
        seen.emplace_back();
        next_tick.push_back(0);
        return *nodes.back();
    }
    size_t index(const Host& h) const {
        for (size_t i = 0; i < nodes.size(); ++i) if (nodes[i].get() == &h) return i;
        throw std::runtime_error("not a node of this simulation");
    }
    void at(uint64_t when, std::function<void()> fn) { queue.push({when, ++seq, std::move(fn)}); }

    // Put what a node produced on the air, then run its timer as the firmware would.
    void pump(Host& h) {
        const size_t src = index(h);
        const uint32_t wait = h.tick();
        CHECK(wait != 0);                                  // the timer never asks to be called at once
        for (const Packet& p : h.take()) {
            air.push_back({t, src, p.type, p.payload.size(), p.flood});
            tap.push_back(p);
            for (size_t dst = 0; dst < nodes.size(); ++dst) {
                if (dst == src) continue;
                const int copies = random() < duplicate ? 2 : 1;
                for (int k = 0; k < copies; ++k) {
                    if (random() < loss) continue;
                    const uint64_t when = t + delay_min + uint64_t(random() * (delay_max - delay_min));
                    at(when, [this, dst, p] { deliver(dst, p); });
                }
            }
        }
        if (wait != UINT32_MAX) {
            const uint64_t when = (uint64_t(h.now()) + wait) * 1000;
            if (next_tick[src] <= t || when < next_tick[src]) {
                next_tick[src] = when;
                at(when, [this, &h] { pump(h); });
            }
        }
    }
    void deliver(size_t dst, const Packet& p) {
        const Bytes key = digest(cat({p.type}, p.payload), 8);
        auto& table = seen[dst];
        if (std::find(table.begin(), table.end(), key) != table.end()) return;   // duplicate suppression
        table.push_back(key);
        if (table.size() > 160) table.pop_front();
        nodes[dst]->receive(p);
        pump(*nodes[dst]);
    }
    void inject(Host& h, const Packet& p) { h.receive(p); pump(h); }    // replay by an attacker
    void run(double until_s) {
        const uint64_t until = uint64_t(until_s * 1000);
        while (!queue.empty() && queue.top().when <= until) {
            const Event e = queue.top();
            queue.pop();
            t = e.when;
            e.fn();
        }
        t = until;
    }

    size_t count(uint8_t type, size_t len = 0) const {
        return size_t(std::count_if(air.begin(), air.end(), [&](const Air& x) { return x.type == type && (len == 0 || x.len == len); }));
    }
    size_t flooded_bytes() const {
        size_t n = 0;
        for (const Air& x : air) if (x.flood) n += x.len;
        return n;
    }
    const Packet& first(uint8_t type, size_t len) const {
        for (const Packet& p : tap) if (p.type == type && p.payload.size() == len) return p;
        throw std::runtime_error("no such packet was sent");
    }
};

// The phone app's side of a send: wait for the confirmation, retry, and on the
// last attempt reset the path so that the message floods.
struct App {
    Sim& sim;
    Host& host;
    double timeout;
    int attempts;
    bool flood_last;
    std::map<std::string, std::string> results;        // text -> "ok" | "failed"
    std::map<std::string, int> tries;
    std::map<std::string, std::vector<uint32_t>> tokens;

    App(Sim& s, Host& h, double timeout_s = 10, int n = 3, bool flood = true) : sim(s), host(h), timeout(timeout_s), attempts(n), flood_last(flood) {}

    void send(const Bytes& peer, const std::string& text, double when_s) {
        sim.at(uint64_t(when_s * 1000), [this, peer, text] { attempt(peer, text, uint32_t(sim.t / 1000) + 1700000000u, 0); });
    }
    void attempt(const Bytes& peer, const std::string& text, uint32_t timestamp, int n) {
        if (n == attempts - 1 && n > 0 && flood_last) host.contact(peer).has_path = false;
        tokens[text].push_back(host.app_send(peer, text, timestamp, unsigned(n)));
        tries[text] = n + 1;
        // a held send is given the time the engine holds it (SPEC 10.4)
        const uint64_t wait = host.held_for != 0 ? uint64_t(host.held_for) * 1000 : uint64_t(timeout * 1000);
        sim.pump(host);
        sim.at(sim.t + wait, [this, peer, text, timestamp, n] { check(peer, text, timestamp, n); });
    }
    void check(const Bytes& peer, const std::string& text, uint32_t timestamp, int n) {
        if (results.count(text) && results[text] == "ok") return;
        const auto& mine = tokens[text];
        if (std::any_of(mine.begin(), mine.end(), [&](uint32_t tok) { return host.is_confirmed(tok); })) results[text] = "ok";
        else if (n + 1 < attempts) attempt(peer, text, timestamp, n + 1);
        else results[text] = "failed";
    }
    size_t ok() const {
        return size_t(std::count_if(results.begin(), results.end(), [](const auto& r) { return r.second == "ok"; }));
    }
    bool all_ok(size_t n) const { return results.size() == n && ok() == n; }
};

// Two nodes that know each other as contacts.
struct Two { Host& a; Host& b; };
Two pair(Sim& sim, bool a_zc = true, bool b_zc = true, bool paths = true, bool adverts = true) {
    Host& a = sim.node("alice", a_zc);
    Host& b = sim.node("bob", b_zc);
    a.add_contact(b.S, paths);
    b.add_contact(a.S, paths);
    if (adverts) {
        a.advert(b.S, b.zc);
        b.advert(a.S, a.zc);
    }
    return {a, b};
}

// Rules that hold in every run.
void check_rules(const Sim& sim) {
    std::set<Bytes> sealed;
    for (const Air& x : sim.air) {
        if (!x.flood) continue;
        CHECK(!(x.type == REQ && x.len == 42));         // the flood rule: no handshake 1,
        CHECK(!(x.type == RESPONSE && x.len == 42));    // no handshake 2,
        CHECK(!(x.type == TXT && x.len == 10));         // no confirmation
    }
    for (const auto& n : sim.nodes) {
        CHECK(n->violations.empty());
        CHECK(n->engine_floods <= n->app_attempts);
    }
    for (const Packet& p : sim.tap) {                   // SPEC 7: no sealed packet is ever sent twice
        if (p.type == ACK || (p.payload.size() - 4) % 16 == 0) continue;
        CHECK(sealed.insert(cat({p.type}, p.payload)).second);
    }
}
void confirmed_were_delivered(const App& app, const Host& receiver, const Host& sender) {
    const auto got = receiver.delivered(sender.S);
    for (const auto& r : app.results) if (r.second == "ok") CHECK(got.count(r.first) == 1);
}

// ---- helpers for scenarios stepped by hand, without the link ----

std::vector<Packet> give(Host& to, const std::vector<Packet>& packets) {
    for (const Packet& p : packets) to.receive(p);
    return to.take();
}
// Carry packets between two hosts until neither has anything left to say.
void settle(Host& x, Host& y, std::vector<Packet> to_y) {
    std::vector<Packet> to_x;
    while (!to_y.empty() || !to_x.empty()) {
        std::vector<Packet> from_y = give(y, to_y), from_x = give(x, to_x);
        to_x = from_y;
        to_y = from_x;
    }
}
// A sends one text to B and everything that follows is delivered.
uint32_t say(Host& a, Host& b, const std::string& text, uint32_t timestamp = 1700000000u) {
    const uint32_t tok = a.app_send(b.S, text, timestamp, 0);
    settle(a, b, a.take());
    return tok;
}
Packet garbage(const Host& to, const Host& from, uint8_t type, size_t len, uint8_t fill) {
    Bytes payload(len, fill);
    payload[0] = to.S[0];
    payload[1] = from.S[0];
    return {type, payload, false};
}
// A genuine handshake 1 from `from` to `to`, made outside any engine.
Packet handshake1(Host& from, Host& to) {
    zcpfs_hs_out keep;
    Bytes payload(42);
    payload[0] = to.S[0];
    payload[1] = from.S[0];
    CHECK(zcpfs_hs1_make(&from.drbg, from.contact(to.S).ss.data(), from.S.data(), to.S.data(), &keep, payload.data() + 2) == ZCPFS_OK);
    return {REQ, payload, false};
}
size_t of_type(const std::vector<Packet>& packets, uint8_t type, size_t len = 0) {
    return size_t(std::count_if(packets.begin(), packets.end(), [&](const Packet& p) { return p.type == type && (len == 0 || p.payload.size() == len); }));
}
}

TEST(zcpfs_peer_pair, "UNIT-ZCPFS-011", "Two nodes with paths: one handshake, sealed texts both ways, 4-byte ACKs, nothing flooded") {
    {
        Sim sim(1);
        Two n = pair(sim);
        App app_a(sim, n.a), app_b(sim, n.b);
        for (int i = 0; i < 10; ++i) {
            app_a.send(n.b.S, "a to b " + std::to_string(i), 5 + 20 * i);
            app_b.send(n.a.S, "b to a " + std::to_string(i), 15 + 20 * i);
        }
        sim.run(600);
        CHECK(app_a.all_ok(10) && app_b.all_ok(10));
        CHECK(n.b.delivered(n.a.S).size() == 10 && n.a.delivered(n.b.S).size() == 10);
        CHECK(n.a.inbox.size() == 10 && n.b.inbox.size() == 10);
        CHECK(sim.count(RESPONSE, 42) == 1);                // one handshake in all
        CHECK(sim.count(ACK) == 20 && sim.count(ACK, 4) == 20);
        CHECK(sim.flooded_bytes() == 0);
        CHECK(n.a.legacy_sent + n.b.legacy_sent == 0);
        CHECK(n.a.sealed_sent == 10 && n.b.sealed_sent == 10);
        CHECK(n.a.record(n.b.S)->current.used && !n.a.record(n.b.S)->next_used);
        check_rules(sim);
    }
    {   // a session that sat idle for longer than T_IDLE is replaced without a wasted attempt
        Sim sim(17);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "day one", 5);
        app.send(n.b.S, "day three", 5 + 2 * 86400);
        sim.run(3 * 86400 + 60);
        CHECK(app.all_ok(2));
        CHECK(sim.count(RESPONSE, 42) == 2);
        CHECK(app.tries["day three"] == 1);
        CHECK(n.a.record(n.b.S) != nullptr && !n.a.record(n.b.S)->current.used);    // and that one expired too
        check_rules(sim);
    }
}

TEST(zcpfs_peer_first_contact, "UNIT-ZCPFS-012", "Without a path only a probe or a sealed text is flooded; a pinned contact never gets legacy") {
    {
        Sim sim(2);
        Two n = pair(sim, true, true, false);
        App app(sim, n.a, 10, 3, false);
        const std::string text = "first message, long enough to matter first message, long enough to matter";
        app.send(n.b.S, text, 5);
        sim.run(120);
        CHECK(app.all_ok(1));
        size_t probes = 0;
        for (const Air& x : sim.air) {
            if (!x.flood) continue;
            CHECK(x.type != TXT);                           // the message itself never floods
            if (x.type == REQ) { CHECK(x.len == 14); ++probes; }
        }
        CHECK(probes == 1 && app.tries[text] == 1);         // the path return starts the handshake
        CHECK(n.a.reciprocal_paths >= 1);                   // repeated before the handshake (SPEC 11)
        check_rules(sim);
    }
    {   // nothing ever arrives: the message fails, it is not sent as legacy
        Sim sim(15);
        Two n = pair(sim);
        App app(sim, n.a);
        sim.loss = 1.0;
        app.send(n.b.S, "into the void", 5);
        sim.run(120);
        CHECK(app.results["into the void"] == "failed");
        CHECK(n.a.legacy_sent == 0);
        for (const Air& x : sim.air) CHECK(x.type != TXT || (x.len - 4) % 16 != 0);
        CHECK(n.a.hs1_sent >= 3 && n.a.probes_sent == 1);   // direct retries, one probe on the flooded attempt
        check_rules(sim);
    }
    {   // the path is gone when the session comes up: held messages and confirmations wait, nothing floods
        Sim sim(29);
        Two n = pair(sim);
        const uint32_t tok = n.a.app_send(n.b.S, "held", 1700000000u, 0);
        const std::vector<Packet> hs2 = give(n.b, n.a.take());
        n.a.contact(n.b.S).has_path = false;
        CHECK(give(n.a, hs2).empty());
        CHECK(n.a.record(n.b.S)->current.used && n.a.record(n.b.S)->n_msgs == 1);
        n.a.contact(n.b.S).has_path = true;
        sim.t = DATA_RETRY * 1000;
        const std::vector<Packet> resent = n.a.tick_packets();   // the resend of 10.3 carries it
        CHECK(resent.size() == 1 && resent[0].type == TXT && !resent[0].flood);
        settle(n.a, n.b, resent);
        CHECK(n.b.received(n.a.S, "held") == 1 && n.a.is_confirmed(tok));
        // the same for a handshake with nothing held: no confirmation without a path
        n.b.boot("again");
        sim.t = 100000;
        const std::vector<Packet> resync = give(n.b, {garbage(n.b, n.a, TXT, 30, 0x42)});
        CHECK(of_type(resync, REQ, 42) == 1);
        const std::vector<Packet> answer = give(n.a, resync);
        n.b.contact(n.a.S).has_path = false;
        CHECK(give(n.b, answer).empty() && n.b.record(n.a.S)->current.used);
        CHECK(n.a.violations.empty() && n.b.violations.empty());
    }
    {   // with a session and no path the sealed text is what floods, once per attempt
        Sim sim(21);
        Two n = pair(sim);
        const uint32_t first = say(n.a, n.b, "over the path");
        CHECK(n.a.is_confirmed(first));
        n.a.contact(n.b.S).has_path = false;
        int result = 0;
        const uint32_t tok = n.a.app_send(n.b.S, "flooded", 1700000100u, 0, &result);
        const std::vector<Packet> sent = n.a.take();
        CHECK(result == ZCPFS_SENT && sent.size() == 1 && sent[0].type == TXT && sent[0].flood);
        settle(n.a, n.b, sent);
        CHECK(n.b.received(n.a.S, "flooded") == 1 && n.a.is_confirmed(tok));
        CHECK(n.a.contact(n.b.S).has_path);                 // the ACK came back in a path return
        CHECK(n.a.violations.empty() && n.b.violations.empty());
    }
}

TEST(zcpfs_peer_loss, "UNIT-ZCPFS-013", "Loss, reordering, duplicates and a long outage: confirmed means delivered, a resend is delivered once") {
    {
        Sim sim(3);
        Two n = pair(sim);
        App app_a(sim, n.a), app_b(sim, n.b);
        sim.loss = 0.3;
        for (int i = 0; i < 12; ++i) {
            app_a.send(n.b.S, "lossy a " + std::to_string(i), 5 + 40 * i);
            app_b.send(n.a.S, "lossy b " + std::to_string(i), 25 + 40 * i);
        }
        sim.run(1200);
        confirmed_were_delivered(app_a, n.b, n.a);
        confirmed_were_delivered(app_b, n.a, n.b);
        CHECK(app_a.ok() + app_b.ok() >= 16);               // of 24, at 30 % loss
        CHECK(n.a.legacy_sent + n.b.legacy_sent == 0);
        check_rules(sim);
    }
    {
        Sim sim(10);
        Two n = pair(sim);
        App app(sim, n.a, 30);
        sim.delay_min = 100;
        sim.delay_max = 6000;
        sim.duplicate = 0.3;
        for (int i = 0; i < 12; ++i) app.send(n.b.S, "burst " + std::to_string(i), 20 + i);
        sim.run(300);
        CHECK(app.all_ok(12));
        CHECK(n.b.delivered(n.a.S).size() == 12 && n.b.inbox.size() == 12);    // none twice
        check_rules(sim);
    }
    {   // more than K_UNACKED sends vanish: the next one starts with a new handshake
        Sim sim(9);
        Two n = pair(sim);
        App app(sim, n.a, 10, 1);
        app.send(n.b.S, "start", 5);
        sim.run(30);
        sim.loss = 1.0;
        for (int i = 0; i < 12; ++i) app.send(n.b.S, "lost " + std::to_string(i), 40 + 12 * i);
        sim.run(200);
        sim.loss = 0.0;
        App retry(sim, n.a);
        retry.send(n.b.S, "back", 210);
        sim.run(400);
        CHECK(retry.all_ok(1));
        CHECK(n.a.hs1_sent >= 2);
        check_rules(sim);
    }
}

TEST(zcpfs_peer_reboot, "UNIT-ZCPFS-014", "A reboot of either side is repaired: by the receiver's own handshake, or by the sender's next send") {
    {
        Sim sim(4);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "before", 5);
        sim.run(60);
        n.b.boot("new entropy");
        app.send(n.b.S, "after", 70);
        sim.run(200);
        CHECK(app.all_ok(2));
        CHECK(app.tries["after"] == 2);                     // one wasted attempt
        CHECK(n.b.hs1_sent == 1);                           // SPEC 10.2
        check_rules(sim);
    }
    {
        Sim sim(5);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "before", 5);
        sim.run(60);
        n.a.boot("new entropy");
        app.send(n.b.S, "after", 70);
        sim.run(200);
        CHECK(app.all_ok(2));
        CHECK(app.tries["after"] == 1);
        CHECK(sim.count(RESPONSE, 42) == 2);
        // the receiver kept the old session as `previous` and wiped it T_PREV after the switch
        CHECK(n.b.record(n.a.S)->current.used && !n.b.record(n.a.S)->previous.used);
        check_rules(sim);
    }
}

TEST(zcpfs_peer_confirm, "UNIT-ZCPFS-020", "A lost confirmation is repeated when the peer still uses the old session, at most once per interval") {
    Sim sim(28);
    Two n = pair(sim);
    Host& a = n.a;
    Host& b = n.b;
    say(a, b, "one");
    // a starts a second handshake (SPEC 10.2) and its confirmation is lost
    const std::vector<Packet> hs1 = give(a, {garbage(a, b, TXT, 30, 0x55)});
    CHECK(of_type(hs1, REQ, 42) == 1);
    const std::vector<Packet> lost = give(a, give(b, hs1));
    CHECK(lost.size() == 1 && lost[0].type == TXT && lost[0].payload.size() == 10 && !lost[0].flood);
    zcpfs_peer *ra = a.record(b.S), *rb = b.record(a.S);
    CHECK(ra->current.used && ra->previous.used && !ra->prev_timed);
    CHECK(rb->current.used && rb->next_used);
    // b still seals under the old session; a opens it under `previous` and confirms again
    sim.t = 20000;
    b.app_send(a.S, "three", 1700000003u, 0);
    std::vector<Packet> reply = give(a, b.take());
    CHECK(a.received(b.S, "three") == 1);
    CHECK(reply.size() == 2 && of_type(reply, ACK, 4) == 1 && of_type(reply, TXT, 10) == 1);
    CHECK(lost[0].payload != reply[0].payload && lost[0].payload != reply[1].payload);   // sealed anew
    b.app_send(a.S, "four", 1700000004u, 0);              // that one is lost too; no second one at once
    reply = give(a, b.take());
    CHECK(reply.size() == 1 && reply[0].type == ACK);
    sim.t += ZCPFS_T_CONFIRM * 1000;
    const uint32_t five = b.app_send(a.S, "five", 1700000005u, 0);
    reply = give(a, b.take());
    CHECK(reply.size() == 2 && of_type(reply, TXT, 10) == 1);
    CHECK(ra->previous.used && !ra->prev_timed);            // nothing has opened under a's `current` yet
    // this confirmation arrives: b switches, and so a stops keeping the old session
    CHECK(give(b, reply).empty());                          // a confirmation is not acknowledged
    CHECK(b.is_confirmed(five));                            // its ACK matched the session that is now `previous`
    CHECK(rb->current.used && !rb->next_used && rb->previous.used && rb->prev_timed);
    const uint32_t tok = b.app_send(a.S, "six", 1700000006u, 0);
    settle(b, a, b.take());
    CHECK(a.received(b.S, "six") == 1 && b.is_confirmed(tok));
    CHECK(ra->prev_timed && a.confirmations_sent == 3);
    CHECK(a.violations.empty() && b.violations.empty());
}

TEST(zcpfs_peer_replay, "UNIT-ZCPFS-015", "Simultaneous open resolves without a retry; a replayed handshake 1 never disturbs the current session") {
    {
        Sim sim(6);
        Two n = pair(sim);
        App app_a(sim, n.a), app_b(sim, n.b);
        app_a.send(n.b.S, "from a", 5);
        app_b.send(n.a.S, "from b", 5);
        sim.run(120);
        CHECK(app_a.all_ok(1) && app_b.all_ok(1));
        CHECK(app_a.tries["from a"] + app_b.tries["from b"] == 2);
        CHECK(sim.count(RESPONSE, 42) == 1);                // the node with the smaller key stayed initiator
        check_rules(sim);
    }
    {
        Sim sim(7);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "one", 5);
        sim.run(60);
        const Packet old_hs1 = sim.first(REQ, 42);
        sim.seen[sim.index(n.b)].clear();                   // past the duplicate horizon
        sim.inject(n.b, old_hs1);
        CHECK(n.b.record(n.a.S)->next_used);                // a session nobody can confirm
        CHECK(n.b.record(n.a.S)->current.used);
        app.send(n.b.S, "two", 70);
        sim.run(200);
        CHECK(app.all_ok(2) && app.tries["two"] == 1);
        check_rules(sim);
    }
    {   // SPEC 13: a replay between handshake 1 and the first packet costs that packet; 10.2 repairs it
        Sim sim(8);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "one", 5);
        sim.run(60);
        const Packet old_hs1 = sim.first(REQ, 42);
        n.a.boot("entropy 2");
        n.a.app_send(n.b.S, "two", 1700000100u, 0);
        const std::vector<Packet> hs1 = n.a.take();
        CHECK(hs1.size() == 1 && hs1[0].type == REQ);
        const std::vector<Packet> hs2 = give(n.b, hs1);
        CHECK(give(n.b, {old_hs1}).size() == 1);            // displaces the genuine `next`
        const std::vector<Packet> data = give(n.a, hs2);
        const std::vector<Packet> back = give(n.b, data);
        CHECK(n.b.received(n.a.S, "two") == 0);             // the attack worked once
        CHECK(of_type(back, REQ, 42) == 1 && n.b.hs1_sent == 1);
        CHECK(n.a.record(n.b.S)->n_msgs == 1);              // a still means to resend it
        settle(n.b, n.a, back);                             // b's own handshake
        CHECK(n.a.record(n.b.S)->n_msgs == 0);              // but not under another session (SPEC 10.3)
        CHECK(!n.b.record(n.a.S)->next_used);               // 8.4 step 3 wiped the replay's session
        const uint32_t tok = n.a.app_send(n.b.S, "two", 1700000100u, 1);
        settle(n.a, n.b, n.a.take());
        CHECK(n.b.received(n.a.S, "two") == 1 && n.a.is_confirmed(tok));
        CHECK(n.a.violations.empty() && n.b.violations.empty());
    }
}

TEST(zcpfs_peer_legacy, "UNIT-ZCPFS-016", "A legacy peer gets plain legacy; the hint upgrades two zcpfs nodes but cannot pin; an unpinned contact goes back to legacy") {
    {
        Sim sim(11);
        Two n = pair(sim, true, false);
        App app_a(sim, n.a), app_b(sim, n.b);
        app_a.send(n.b.S, "to old", 5);
        app_b.send(n.a.S, "from old", 20);
        sim.run(120);
        CHECK(app_a.all_ok(1) && app_b.all_ok(1));
        CHECK(!n.a.contact(n.b.S).pinned);
        CHECK(sim.count(ACK, 6) == sim.count(ACK));         // legacy ACKs keep 6 bytes
        for (const Air& x : sim.air) CHECK(x.type != TXT || (x.len - 4) % 16 == 0);
        CHECK(sim.count(REQ) + sim.count(RESPONSE) == 0);
        check_rules(sim);
    }
    for (bool paths : {true, false}) {                      // two contacts added by hand, no advert seen
        Sim sim(12);
        Two n = pair(sim, true, true, paths, false);
        App app(sim, n.a, 10, 3, false);
        app.send(n.b.S, "first goes legacy", 5);
        sim.run(60);
        CHECK(app.results["first goes legacy"] == "ok");
        CHECK(n.a.legacy_sent == 1);
        CHECK(n.a.contact(n.b.S).pinned && n.b.contact(n.a.S).pinned);
        app.send(n.b.S, "second is sealed", 70);
        sim.run(140);
        CHECK(app.results["second is sealed"] == "ok");
        CHECK(n.a.legacy_sent == 1 && n.a.sealed_sent == 1);
        check_rules(sim);
    }
    {   // the hint alone pins nothing
        Sim sim(13);
        Two n = pair(sim, true, false, true, false);
        App app(sim, n.a);
        app.send(n.b.S, "hello old node", 5);
        sim.run(60);
        CHECK(app.all_ok(1));
        CHECK(!n.a.contact(n.b.S).pinned);
    }
    {   // an accepted advert without the tag: sessions wiped, legacy again
        Sim sim(14);
        Two n = pair(sim);
        App app(sim, n.a);
        app.send(n.b.S, "sealed", 5);
        sim.run(60);
        CHECK(n.a.record(n.b.S)->current.used);
        n.a.advert(n.b.S, false);
        CHECK(n.a.record(n.b.S) == nullptr);
        n.b.zc = false;                                     // the peer really moved to old firmware
        app.send(n.b.S, "legacy again", 70);
        sim.run(140);
        CHECK(app.results["legacy again"] == "ok");
        CHECK(n.a.legacy_sent == 1);
    }
}

TEST(zcpfs_peer_timers, "UNIT-ZCPFS-017", "A held message is resent a bounded number of times and delivered once; held messages, handshake attempts, old sessions and skipped keys are wiped on time") {
    {   // SPEC 10.3, with exactly one packet lost
        Sim sim(18);
        Two n = pair(sim);
        const uint32_t tok = n.a.app_send(n.b.S, "resend me", 1700000000u, 0);
        const std::vector<Packet> first = give(n.a, give(n.b, n.a.take()));    // the held message; "lost"
        CHECK(first.size() == 1 && first[0].type == TXT);
        CHECK(n.a.tick() == DATA_RETRY && n.a.take().empty());          // not due yet
        sim.t += DATA_RETRY * 1000;
        const std::vector<Packet> again = n.a.tick_packets();
        CHECK(again.size() == 1 && again[0].payload != first[0].payload);       // sealed anew
        CHECK(n.a.tick() == DATA_RETRY);                                        // and once more after another step
        CHECK(again[0].payload.size() == first[0].payload.size());
        settle(n.b, n.a, give(n.b, again));                                     // both copies reach b after all
        settle(n.b, n.a, give(n.b, first));
        CHECK(n.b.inbox.size() == 1 && n.b.received(n.a.S, "resend me") == 1);  // delivered once
        CHECK(std::count(n.a.confirmed.begin(), n.a.confirmed.end(), tok) == 2);    // each copy acknowledged
        sim.t += 10 * DATA_RETRY * 1000;
        CHECK(n.a.tick_packets().empty());                                      // nothing more once confirmed
        CHECK(n.a.record(n.b.S)->n_msgs == 0);
    }
    {   // never acknowledged: N_DATA_TRIES resends, then the plaintext is gone
        Sim sim(19);
        Two n = pair(sim);
        n.a.app_send(n.b.S, "into silence", 1700000000u, 0);
        give(n.a, give(n.b, n.a.take()));
        size_t sent = 0;
        for (int i = 0; i < 10; ++i) {
            sim.t += DATA_RETRY * 1000;
            sent += n.a.tick_packets().size();
        }
        CHECK(sent == ZCPFS_N_DATA_TRIES);
        CHECK(n.a.record(n.b.S)->n_msgs == 0);
    }
    {   // SPEC 10.1: a retry replaces the held attempt, the list is bounded, a burst starts one handshake
        Sim sim(27);
        Two n = pair(sim);
        int result = 0;
        n.a.app_send(n.b.S, "same text", 1700000000u, 0, &result);
        const uint32_t retry = n.a.app_send(n.b.S, "same text", 1700000000u, 1, &result);
        zcpfs_peer *r = n.a.record(n.b.S);
        CHECK(result == ZCPFS_HELD && r->n_msgs == 1 && r->msgs[0].token == retry);
        n.a.app_send(n.b.S, "same text", 1700000001u, 0);  // another timestamp is another message
        n.a.app_send(n.b.S, "same", 1700000000u, 0);       // and so is another text
        CHECK(r->n_msgs == 3);
        for (int i = 0; i < ZCPFS_N_HELD; ++i) n.a.app_send(n.b.S, "more " + std::to_string(i), 1700000002u, 0);
        CHECK(r->n_msgs == ZCPFS_N_HELD);
        const std::vector<Packet> sent = n.a.take();
        CHECK(sent.size() == 1 && of_type(sent, REQ, 42) == 1);
        settle(n.a, n.b, sent);                             // the oldest were dropped, the rest arrive in order
        CHECK(n.b.inbox.size() == ZCPFS_N_HELD);
        for (int i = 0; i < ZCPFS_N_HELD; ++i) CHECK(n.b.inbox[size_t(i)].second == "more " + std::to_string(i));
        CHECK(n.a.confirmed.size() == ZCPFS_N_HELD && r->n_msgs == 0);
    }
    {   // the peer never answers: N_HS_TRIES handshakes, then the message and the attempts are dropped
        Sim sim(22);
        Two n = pair(sim);
        int result = 0;
        n.a.app_send(n.b.S, "nobody home", 1700000000u, 0, &result);
        const std::vector<Packet> first_hs1 = n.a.take();               // kept for the end
        CHECK(result == ZCPFS_HELD && of_type(first_hs1, REQ, 42) == 1);
        CHECK(n.a.held_for == HELD);                                    // what the application is told to wait
        CHECK(n.a.tick() == HS_RETRY);
        sim.t = HS_RETRY * 1000;
        CHECK(of_type(n.a.tick_packets(), REQ, 42) == 1);
        sim.t = 2 * HS_RETRY * 1000;
        CHECK(of_type(n.a.tick_packets(), REQ, 42) == 1);
        zcpfs_peer *r = n.a.record(n.b.S);
        CHECK(r->n_out == ZCPFS_N_HS_TRIES && r->n_msgs == 1);
        CHECK(n.a.tick() == HELD - 2 * HS_RETRY && n.a.take().empty()); // no fourth handshake
        sim.t = (HELD - 1) * 1000;
        CHECK(n.a.tick() == 1 && r->n_msgs == 1 && r->n_out == ZCPFS_N_HS_TRIES);
        sim.t = HELD * 1000;                                            // the application gives up now
        CHECK(n.a.tick() == UINT32_MAX);                                // and nothing is left to wait for
        CHECK(r->n_msgs == 0 && r->n_out == 0 && n.a.take().empty());
        // a handshake 2 that comes after all of that finds nothing to complete
        const std::vector<Packet> late = give(n.b, first_hs1);
        CHECK(of_type(late, RESPONSE, 42) == 1);
        CHECK(give(n.a, late).empty() && !r->current.used);
    }
    for (size_t answered = 0; answered < ZCPFS_N_HS_TRIES; ++answered) {
        // SPEC 8.2: a handshake 2 that answers any attempt still kept completes the handshake
        Sim sim(31);
        Two n = pair(sim);
        const uint32_t tok = n.a.app_send(n.b.S, "any of three", 1700000000u, 0);
        std::vector<Packet> attempts = n.a.take();
        for (uint32_t i = 1; i < ZCPFS_N_HS_TRIES; ++i) {
            sim.t = uint64_t(i) * HS_RETRY * 1000;
            for (const Packet& p : n.a.tick_packets()) attempts.push_back(p);
        }
        CHECK(attempts.size() == ZCPFS_N_HS_TRIES && of_type(attempts, REQ, 42) == ZCPFS_N_HS_TRIES);
        settle(n.a, n.b, {attempts[answered]});
        CHECK(n.b.received(n.a.S, "any of three") == 1 && n.a.is_confirmed(tok));
    }
    {   // a handshake nobody answers, with nothing held: the attempt is wiped HS_WAIT steps after it was sent
        Sim sim(30);
        Two n = pair(sim);
        CHECK(of_type(give(n.b, {garbage(n.b, n.a, TXT, 30, 0x21)}), REQ, 42) == 1);   // SPEC 10.2
        zcpfs_peer *r = n.b.record(n.a.S);
        CHECK(n.b.tick() == HS_WAIT && r->n_out == 1);
        sim.t = (HS_WAIT - 1) * 1000;
        CHECK(n.b.tick() == 1 && r->n_out == 1);
        sim.t = HS_WAIT * 1000;
        CHECK(n.b.tick() == UINT32_MAX && r->n_out == 0 && n.b.take().empty());
    }
    {   // `previous` goes T_PREV after the peer switched; an idle session after T_IDLE
        Sim sim(23);
        Two n = pair(sim);
        say(n.a, n.b, "one");
        n.a.boot("again");
        sim.t = 100000;
        const uint32_t tok = say(n.a, n.b, "two", 1700000100u);
        CHECK(n.a.is_confirmed(tok));
        zcpfs_peer *r = n.b.record(n.a.S);
        CHECK(r->current.used && r->previous.used && r->prev_timed);
        CHECK(n.b.tick() == ZCPFS_T_PREV);
        sim.t += (ZCPFS_T_PREV - 1) * 1000;
        CHECK(n.b.tick() == 1 && r->previous.used);
        sim.t += 1000;
        CHECK(n.b.tick() == ZCPFS_T_IDLE + 1 - ZCPFS_T_PREV && !r->previous.used);
        sim.t += uint64_t(ZCPFS_T_IDLE + 1 - ZCPFS_T_PREV) * 1000;
        CHECK(n.b.tick() == UINT32_MAX && !r->current.used);
    }
    {   // a key kept for a packet that never came is wiped after T_SKIP by the timer alone
        Sim sim(24);
        Two n = pair(sim);
        say(n.a, n.b, "one");
        n.a.app_send(n.b.S, "lost", 1700000001u, 0);
        n.a.take();
        say(n.a, n.b, "three", 1700000002u);
        zcpfs_peer *r = n.b.record(n.a.S);
        CHECK(r->current.s.n_skipped == 1);
        CHECK(n.b.tick() == ZCPFS_T_SKIP);
        sim.t += ZCPFS_T_SKIP * 1000;
        n.b.tick();
        CHECK(r->current.s.n_skipped == 0);
        CHECK(n.b.received(n.a.S, "three") == 1 && n.b.received(n.a.S, "lost") == 0);
    }
}

TEST(zcpfs_peer_table, "UNIT-ZCPFS-018", "Contacts sharing an address byte are told apart; a full record table gives way only to authenticated or local use") {
    {
        Sim sim(16);
        Two n = pair(sim);
        Host *c = nullptr;
        for (int i = 0; c == nullptr; ++i) {                // a contact whose hash byte equals a's
            c = &sim.node("mallory " + std::to_string(i));
            if (c->S[0] != n.a.S[0]) {
                sim.nodes.pop_back();
                sim.seen.pop_back();
                sim.next_tick.pop_back();
                c = nullptr;
            }
        }
        n.b.add_contact(c->S, true);
        c->add_contact(n.b.S, true);
        n.b.advert(c->S, true);
        c->advert(n.b.S, true);
        App app_a(sim, n.a), app_c(sim, *c);
        for (int k = 0; k < 4; ++k) {
            app_a.send(n.b.S, "from a " + std::to_string(k), 5 + 30 * k);
            app_c.send(n.b.S, "from c " + std::to_string(k), 20 + 30 * k);
        }
        sim.run(400);
        CHECK(app_a.all_ok(4) && app_c.all_ok(4));
        CHECK(n.b.delivered(n.a.S).size() == 4 && n.b.delivered(c->S).size() == 4);
        check_rules(sim);
    }
    {
        Sim sim(25);
        Host& b = sim.node("hub", true, 2);                 // room for two contacts
        std::vector<Host *> c;
        for (int i = 0; i < 4; ++i) {
            c.push_back(&sim.node("spoke " + std::to_string(i)));
            b.add_contact(c[size_t(i)]->S, true).pinned = true;
            c[size_t(i)]->add_contact(b.S, true).pinned = true;
        }
        say(*c[0], b, "zero");
        sim.t = 10000;
        say(*c[1], b, "one");
        CHECK(b.record(c[0]->S)->current.used && b.record(c[1]->S)->current.used);

        // packets nobody vouched for: a text that opens nowhere, a replayable probe
        CHECK(give(b, {garbage(b, *c[2], TXT, 30, 0x77)}).empty());             // no record, so no resync
        c[2]->contact(b.S).has_path = false;
        c[2]->app_send(b.S, "probe", 1700000000u, 0);
        const std::vector<Packet> probe = c[2]->take();
        CHECK(probe.size() == 1 && probe[0].payload.size() == 14 && probe[0].flood);
        CHECK(of_type(give(b, probe), PATH) == 1);                              // answered all the same
        CHECK(b.record(c[2]->S) == nullptr);
        zcpfs_contact hinting = Host::view(b.contact(c[2]->S));
        hinting.pinned = 0;
        zcpfs_on_hint(&b.node, &hinting, b.now());                              // nor does a hint get a record
        CHECK(b.take().empty() && b.record(c[2]->S) == nullptr);
        CHECK(b.record(c[0]->S)->current.used && b.record(c[1]->S)->current.used);

        // a verified handshake 1 takes the least recently used session
        CHECK(of_type(give(b, {handshake1(*c[2], b)}), RESPONSE, 42) == 1);
        CHECK(b.record(c[0]->S) == nullptr && b.record(c[2]->S)->next_used && b.record(c[1]->S)->current.used);
        // and the next one takes that unconfirmed record, not the live session
        CHECK(of_type(give(b, {handshake1(*c[3], b)}), RESPONSE, 42) == 1);
        CHECK(b.record(c[2]->S) == nullptr && b.record(c[3]->S)->next_used && b.record(c[1]->S)->current.used);
        // a local send takes the unconfirmed record first too
        int result = 0;
        b.app_send(c[0]->S, "outbound", 1700000000u, 0, &result);
        CHECK(result == ZCPFS_HELD && of_type(b.take(), REQ, 42) == 1);
        CHECK(b.record(c[3]->S) == nullptr && b.record(c[0]->S)->n_msgs == 1 && b.record(c[1]->S)->current.used);

        // with nothing held and no handshake waiting that record is idle, and then anybody's
        CHECK(give(b, {garbage(b, *c[2], TXT, 30, 0x78)}).empty());             // still not idle
        sim.t += uint64_t(HELD + 1) * 1000;
        b.tick();
        b.take();
        CHECK(b.record(c[0]->S)->n_msgs == 0 && b.record(c[0]->S)->n_out == 0);
        CHECK(of_type(give(b, {garbage(b, *c[2], TXT, 30, 0x79)}), REQ, 42) == 1);  // SPEC 10.2
        CHECK(b.record(c[0]->S) == nullptr && b.record(c[2]->S)->n_out == 1 && b.record(c[1]->S)->current.used);
        // a contact that was deleted loses its record at the next timer run
        const Bytes gone = c[1]->S;
        b.contacts.erase(std::find_if(b.contacts.begin(), b.contacts.end(), [&](const Contact& x) { return x.S == gone; }));
        b.tick();
        CHECK(b.record(gone) == nullptr && b.record(c[2]->S) != nullptr);
        CHECK(b.violations.empty());
    }
}

TEST(zcpfs_peer_limits, "UNIT-ZCPFS-019", "Rate limits, the unacknowledged and failure limits, flood-routed handshakes ignored, argument checks") {
    Sim sim(26);
    Two n = pair(sim);
    Host& a = n.a;
    Host& b = n.b;
    int result = 0;

    {   // arguments
        const zcpfs_contact v = Host::view(a.contact(b.S));
        const Bytes text(ZCPFS_TEXT_MAX + 1, 'x');
        uint32_t held = 0;
        CHECK(zcpfs_send_text(&a.node, &v, text.data(), 4, 1, 0, &held) == ZCPFS_ERR_ARG);
        CHECK(zcpfs_send_text(&a.node, &v, text.data(), ZCPFS_TEXT_MAX + 1, 1, 0, &held) == ZCPFS_ERR_ARG);
        zcpfs_contact weak = v;
        const Bytes zero(32, 0);
        weak.ss = zero.data();
        CHECK(zcpfs_send_text(&a.node, &weak, text.data(), 20, 1, 0, &held) == ZCPFS_LEGACY);     // SPEC 4
        zcpfs_rx_out o;
        const Packet junk = garbage(a, b, TXT, 30, 0x11);
        CHECK(zcpfs_rx(&a.node, &weak, TXT, junk.payload.data(), junk.payload.size(), 0, 0, &o) == ZCPFS_RX_LEGACY);
        zcpfs_contact unpinned = v;
        unpinned.pinned = 0;
        CHECK(zcpfs_send_text(&a.node, &unpinned, text.data(), 20, 1, 0, &held) == ZCPFS_LEGACY);
        CHECK(zcpfs_rx(&a.node, &unpinned, TXT, junk.payload.data(), junk.payload.size(), 0, 0, &o) == ZCPFS_RX_LEGACY);
        const Bytes legacy_path = cat({a.S[0], b.S[0]}, legacy_seal(a.contact(b.S).ss, {1}));
        CHECK(zcpfs_rx(&a.node, &unpinned, PATH, legacy_path.data(), legacy_path.size(), 0, 0, &o) == ZCPFS_RX_LEGACY);
        CHECK(zcpfs_rx(&a.node, &v, PATH, legacy_path.data(), legacy_path.size(), 0, 0, &o) == ZCPFS_RX_NEXT);   // SPEC 7.2
        CHECK(zcpfs_rx(&a.node, &v, 0x09, junk.payload.data(), junk.payload.size(), 0, 0, &o) == ZCPFS_RX_LEGACY);
        CHECK(a.record(b.S) == nullptr);                    // none of that left a trace
        zcpfs_node none;
        zcpfs_node_init(&none, a.S.data(), a.a.data(), &a.drbg, nullptr, 0, &a.ops, &a);
        CHECK(zcpfs_send_text(&none, &v, text.data(), 20, 1, 0, &held) == ZCPFS_ERR_FULL);
        CHECK(held == 0);                                   // only a held message has a waiting time
        CHECK(zcpfs_send_text(&a.node, &v, text.data(), ZCPFS_TEXT_MAX, 1, 0, &held) == ZCPFS_HELD);   // the longest text
        CHECK(held == HELD);
        a.take();
        a.boot("clean");
    }

    // SPEC 10.4: the step is one and a half times the caller's send timeout, in whole seconds, 5 at least
    const struct { uint32_t ms, step; } steps[] = {{0, 5}, {1000, 5}, {3400, 6}, {5000, 8}, {6300, 11}, {9200, 15}, {14300, 23}};
    for (const auto& row : steps) {
        a.contact(b.S).timeout_ms = row.ms;
        a.app_send(b.S, "how long", 1700000000u, 0, &result);
        CHECK(result == ZCPFS_HELD && a.held_for == ZCPFS_HELD_STEPS * row.step);
        CHECK(a.tick() == ZCPFS_HS_RETRY_STEPS * row.step);         // the next handshake attempt
        CHECK(a.record(b.S)->msgs[0].due == a.now() + a.held_for);  // held as long as the application waits
        a.take();
        a.boot("clean");
    }
    a.contact(b.S).timeout_ms = TIMEOUT_MS;
    CHECK(a.violations.empty());

    // a handshake 1 that arrives flood-routed is not answered and leaves nothing behind
    Packet flooded = handshake1(a, b);
    flooded.flood = true;
    CHECK(give(b, {flooded}).empty() && b.record(a.S) == nullptr);

    // without a path to the sender there is no way to answer but a flood, so there is no answer
    b.contact(a.S).has_path = false;
    CHECK(give(b, {handshake1(a, b)}).empty() && b.record(a.S) == nullptr);
    b.contact(a.S).has_path = true;

    // a text that does not open starts no handshake while one of our own is under way
    a.app_send(b.S, "waiting", 1700000000u, 0, &result);
    CHECK(result == ZCPFS_HELD && of_type(a.take(), REQ, 42) == 1);
    CHECK(give(a, {garbage(a, b, TXT, 30, 0x33)}).empty());
    a.boot("clean");

    // R_HS: three verified handshakes per contact at once, then one per ten seconds
    for (int i = 0; i < 3; ++i) CHECK(of_type(give(b, {handshake1(a, b)}), RESPONSE, 42) == 1);
    CHECK(give(b, {handshake1(a, b)}).empty());
    sim.t = 9000;
    CHECK(give(b, {handshake1(a, b)}).empty());
    sim.t = 10000;
    CHECK(of_type(give(b, {handshake1(a, b)}), RESPONSE, 42) == 1);
    CHECK(give(b, {handshake1(a, b)}).empty());

    // R_PRE: sixteen verifications at once node-wide, whoever they are from, then four per second
    sim.t = 100000;
    for (uint8_t i = 0; i < ZCPFS_R_PRE_BURST; ++i) CHECK(give(b, {garbage(b, a, REQ, 42, i)}).empty());
    CHECK(give(b, {handshake1(a, b)}).empty());             // genuine, and still dropped
    sim.t += 1000;
    CHECK(of_type(give(b, {handshake1(a, b)}), RESPONSE, 42) == 1);
    b.boot("clean");

    // T_PROBE: one answered probe per contact per interval
    auto probe = [&] {
        uint8_t nonce4[4];
        Bytes payload(14);
        zcpfs_drbg_generate(&a.drbg, nonce4, sizeof(nonce4));
        payload[0] = b.S[0];
        payload[1] = a.S[0];
        zcpfs_probe_make(a.contact(b.S).ss.data(), a.S.data(), b.S.data(), nonce4, payload.data() + 2);
        return Packet{REQ, payload, true};
    };
    sim.t = 200000;
    Packet forged = probe();
    forged.payload[13] ^= 1;
    CHECK(give(b, {forged}).empty());
    CHECK(of_type(give(b, {probe()}), PATH) == 1);
    sim.t += (ZCPFS_T_PROBE - 1) * 1000;
    CHECK(give(b, {probe()}).empty());
    sim.t += 1000;
    CHECK(of_type(give(b, {probe()}), PATH) == 1);
    CHECK(!b.record(a.S)->current.used && !b.record(a.S)->next_used);      // a probe changes no session state

    // T_RESYNC: one receiver-started handshake per contact per interval
    b.boot("clean");
    sim.t = 300000;
    CHECK(of_type(give(b, {garbage(b, a, TXT, 30, 1)}), REQ, 42) == 1);
    CHECK(give(b, {garbage(b, a, TXT, 30, 2)}).empty());                    // one is under way
    sim.t += (ZCPFS_T_RESYNC - 1) * 1000;
    CHECK(give(b, {garbage(b, a, TXT, 30, 3)}).empty());                    // too soon
    sim.t += 1000;
    CHECK(of_type(give(b, {garbage(b, a, TXT, 30, 4)}), REQ, 42) == 1);
    b.contact(a.S).has_path = false;
    b.boot("clean");
    CHECK(give(b, {garbage(b, a, TXT, 30, 5)}).empty());                    // never without a path
    b.contact(a.S).has_path = true;

    // K_UNACKED: eight sends without an ACK, and the ninth waits for a new handshake
    a.boot("fresh");
    b.boot("fresh");
    sim.t = 2000000;
    say(a, b, "hello");
    for (int i = 0; i < ZCPFS_K_UNACKED - 1; ++i) {         // seven unheard, then the peer is heard from:
        a.app_send(b.S, "unanswered " + std::to_string(i), 1700000050u + uint32_t(i), 0, &result);
        CHECK(result == ZCPFS_SENT && a.take().size() == 1);
    }
    CHECK(a.record(b.S)->current.unacked == ZCPFS_K_UNACKED - 1);
    b.app_send(a.S, "still here", 1700000060u, 0);       // any packet of the peer under the session will do,
    give(a, b.take());                                      // text that nobody acknowledges included
    a.take();
    CHECK(a.received(b.S, "still here") == 1 && a.record(b.S)->current.unacked == 0);
    for (int i = 0; i < ZCPFS_K_UNACKED; ++i) {
        a.app_send(b.S, "unheard " + std::to_string(i), 1700000000u + uint32_t(i), 0, &result);
        CHECK(result == ZCPFS_SENT && a.take().size() == 1);
    }
    sim.t += 10000;
    a.app_send(b.S, "ninth", 1700000100u, 0, &result);
    CHECK(result == ZCPFS_HELD && of_type(a.take(), REQ, 42) == 1);
    CHECK(!a.record(b.S)->current.used && a.record(b.S)->n_msgs == 1);

    // N_ACK: the list of expected ACKs keeps the newest sixteen
    a.boot("fresh");
    b.boot("fresh");
    say(a, b, "hello");
    uint32_t oldest = 0, newest = 0;
    std::vector<Packet> newest_packet;
    for (int round = 0; round < 3; ++round) {               // seven sends, only the last one arrives
        for (int i = 0; i < 7; ++i) {
            const uint32_t tok = a.app_send(b.S, "n " + std::to_string(round * 7 + i), 1700000000u, 0, &result);
            CHECK(result == ZCPFS_SENT);
            if (round == 0 && i == 0) oldest = tok;
            if (round == 2 && i == 5) { newest = tok; newest_packet = a.take(); }
            if (i < 6) a.take();
        }
        settle(a, b, a.take());
    }
    a.app_send(b.S, "one more", 1700000000u, 0);
    a.take();
    CHECK(a.record(b.S)->current.n_expect == ZCPFS_N_ACK);  // of nineteen never acknowledged
    CHECK(a.record(b.S)->current.expect[0].token == oldest + 3);
    settle(a, b, newest_packet);                            // late, through a skipped key
    CHECK(a.is_confirmed(newest) && a.record(b.S)->current.n_expect == ZCPFS_N_ACK - 1);

    // F_MAX: a session that has seen that many failed decryptions is wiped at the next event
    a.boot("fresh");
    b.boot("fresh");
    say(a, b, "hello again");
    b.record(a.S)->current.s.fails = ZCPFS_F_MAX;
    b.tick();
    CHECK(!b.record(a.S)->current.used);
    CHECK(a.violations.empty() && b.violations.empty());
}
// ---- login (SPEC 19) ----

struct LoginSetup { Host& client; Host& server; };
// A client that has the server as a pinned contact; the server does not know the client.
LoginSetup login_pair(Sim& sim, bool paths = true, const std::string& client_name = "client") {
    Host& client = sim.node(client_name);
    Host *server = nullptr;
    for (const auto& n : sim.nodes) if (!n->offers.empty()) server = n.get();
    if (server == nullptr) {
        server = &sim.node("repeater");
        server->serve("hunter2");
    }
    client.add_contact(server->S, paths);
    client.advert(server->S, true);
    return {client, *server};
}
bool ends_with(const Bytes& b, const std::string& tail) {
    return b.size() >= tail.size() && std::equal(tail.begin(), tail.end(), b.end() - long(tail.size()));
}
bool on_air(const Sim& sim, const std::string& needle) {
    for (const Packet& p : sim.tap)
        if (std::search(p.payload.begin(), p.payload.end(), needle.begin(), needle.end()) != p.payload.end()) return true;
    return false;
}

TEST(zcpfs_login_roles, "UNIT-ZCPFS-021", "Login: the password never leaves the node; the right one sets up a session and grants its role, a wrong one grants nothing") {
    {
        Sim sim(50);
        LoginSetup n = login_pair(sim);
        CHECK(n.client.login(n.server.S, "hunter2", 1700000000u) == ZCPFS_SENT && n.client.login_wait == LOGIN_WAIT);
        sim.pump(n.client);
        sim.run(30);
        CHECK(n.client.logins.size() == 1 && n.client.logins[0].second.size() == ZCPFS_LOGIN_RESPONSE_LEN);
        CHECK(ends_with(n.client.logins[0].second, "ad") && sim.count(ANON, 78) == 1 && sim.count(RESPONSE, 47) == 1);
        CHECK(n.server.logged_in.size() == 1 && n.server.logged_in[0] == std::make_pair(n.client.S, uint8_t(ROLE_ADMIN)));
        CHECK(sim.count(ANON) == 1 && sim.count(TXT, 10) == 1);             // one login, the client's confirmation
        CHECK(!on_air(sim, "hunter2"));
        CHECK(n.server.record(n.client.S)->current.used && n.server.contact(n.client.S).pinned);
        CHECK(n.server.offers_kept() == 0 && !n.client.node.login.used);    // nothing of the login is left
        // admin text both ways under the login's session, without a handshake
        App app_c(sim, n.client), app_s(sim, n.server);
        app_c.send(n.server.S, "get radio", 31);
        app_s.send(n.client.S, "> 869.618,62.5,7,5", 50);
        sim.run(150);
        CHECK(app_c.all_ok(1) && app_s.all_ok(1));
        CHECK(n.server.received(n.client.S, "get radio") == 1 && n.client.received(n.server.S, "> 869.618,62.5,7,5") == 1);
        CHECK(sim.count(REQ, 42) + sim.count(RESPONSE, 42) == 0);
        for (const Air& x : sim.air) CHECK(x.type != TXT || (x.len - 4) % 16 != 0);
        check_rules(sim);
    }
    {   // a wrong password: the server answers, because it cannot know, and nothing opens
        Sim sim(51);
        LoginSetup n = login_pair(sim);
        n.client.login(n.server.S, "letmein", 1700000000u);
        sim.pump(n.client);
        sim.run(LOGIN_WAIT - 1);
        CHECK(sim.count(RESPONSE) == 1 && n.client.logins.empty() && n.server.logged_in.empty());
        CHECK(sim.count(TXT) == 0);                                         // so no confirmation either
        CHECK(n.client.record(n.server.S) == nullptr && n.server.record(n.client.S) == nullptr);
        CHECK(n.server.offers_kept() == 1 && !n.client.node.login.used);   // one reply per login (19.4)
        sim.run(LOGIN_WAIT + 3);                                            // both sides give up on time
        CHECK(n.server.offers_kept() == 0 && !n.client.node.login.used);
        CHECK(n.server.find(n.client.S.data()) == nullptr);                 // the server never learned the client
        check_rules(sim);
    }
    {   // each password has its role; the empty password only for a client on the list
        Sim sim(52);
        Host& guest = login_pair(sim, true, "guest").client;
        LoginSetup n = login_pair(sim, true, "listed");
        Host& stranger = login_pair(sim, true, "stranger").client;
        n.server.guest_password = "visitor";
        n.server.acl.insert(n.client.S);
        guest.login(n.server.S, "visitor", 1700000000u);
        sim.pump(guest);
        sim.run(20);
        n.client.login(n.server.S, "", 1700000020u);
        sim.pump(n.client);
        sim.run(40);
        stranger.login(n.server.S, "", 1700000040u);
        sim.pump(stranger);
        sim.run(80);
        CHECK(n.server.logged_in.size() == 2);
        CHECK(n.server.logged_in[0] == std::make_pair(guest.S, uint8_t(ROLE_GUEST)));
        CHECK(n.server.logged_in[1] == std::make_pair(n.client.S, uint8_t(ROLE_KNOWN)));
        CHECK(guest.logins.size() == 1 && ends_with(guest.logins[0].second, "gu"));
        CHECK(n.client.logins.size() == 1 && ends_with(n.client.logins[0].second, "kn"));
        CHECK(stranger.logins.empty());
        // a typed password gets the admin and the guest branch, none typed gets the one for the list, or no answer
        CHECK(sim.count(RESPONSE, 89) == 1 && sim.count(RESPONSE, 47) == 1 && sim.count(RESPONSE) == 2);
        CHECK(!on_air(sim, "visitor"));
        check_rules(sim);
    }
    {   // without a path the login floods as a legacy login does, and the reply rides the path return
        Sim sim(53);
        LoginSetup n = login_pair(sim, false);
        n.client.login(n.server.S, "hunter2", 1700000000u);
        sim.pump(n.client);
        sim.run(30);
        CHECK(n.client.logins.size() == 1 && n.server.logged_in.size() == 1);
        size_t floods = 0;
        for (const Air& x : sim.air) if (x.flood) { CHECK(x.type == ANON || x.type == PATH); ++floods; }
        CHECK(floods == 2 && sim.count(RESPONSE) == 0);
        CHECK(n.client.contact(n.server.S).has_path && n.server.record(n.client.S)->current.used);
        check_rules(sim);
    }
    {   // a replayed login of a client the server now knows is not even answered; the session stays
        Sim sim(54);
        LoginSetup n = login_pair(sim);
        n.client.login(n.server.S, "hunter2", 1700000000u);
        sim.pump(n.client);
        sim.run(30);
        const Packet old_login = sim.first(ANON, 78);
        sim.seen[sim.index(n.server)].clear();
        sim.inject(n.server, old_login);
        sim.run(40);
        CHECK(sim.count(RESPONSE) == 1 && n.server.logged_in.size() == 1 && n.client.logins.size() == 1);
        CHECK(n.server.offers_kept() == 0 && n.server.logins_dropped == 1);
        App app(sim, n.client);
        app.send(n.server.S, "still here", 41);
        sim.run(100);
        CHECK(app.all_ok(1) && n.server.logged_in.size() == 1 && n.server.offers_kept() == 0);
        check_rules(sim);
    }
}

TEST(zcpfs_login_limits, "UNIT-ZCPFS-022", "Login limits: answered only while there is room for the offer, one place kept for known clients, a new login replaces an offer that is a step old; offers and waits end on time") {
    Sim sim(55);
    std::vector<Host *> c;
    for (int i = 0; i < 6; ++i) c.push_back(&login_pair(sim, true, "client " + std::to_string(i)).client);
    Host& server = *sim.nodes[1];
    CHECK(server.offers.size() == 4);
    uint32_t stamp = 1700000000u;
    auto attempt = [&](Host& client, const std::string& pw) {       // one login; what the server sent back
        client.login(server.S, pw, ++stamp);
        return give(server, client.take());
    };

    // strangers get every place but one: three wrong logins are answered, the fourth costs no airtime
    for (size_t i = 0; i < 3; ++i) CHECK(of_type(attempt(*c[i], "wrong"), RESPONSE, 47) == 1);
    CHECK(server.offers_kept() == 3 && server.tick() == LOGIN_WAIT);
    CHECK(attempt(*c[3], "hunter2").empty() && server.logins_dropped == 1);
    // the last place is for a client the server knows; nobody can pose as one, the login is signed by its key
    server.acl.insert(c[4]->S);
    CHECK(attempt(*c[4], "hunter2").size() == 1 && server.offers_kept() == 4);
    // a client's new login replaces its offer once that is a step old, as an application's retry always is;
    // sooner it is not answered, so one key gets one answer per step however fast it sends
    CHECK(attempt(*c[0], "wrong").empty() && server.logins_dropped == 2);
    sim.t = (STEP - 1) * 1000;
    CHECK(attempt(*c[0], "wrong").empty());
    sim.t = STEP * 1000;
    CHECK(attempt(*c[0], "wrong").size() == 1 && server.offers_kept() == 4);
    CHECK(attempt(*c[0], "hunter2").empty());
    // offers run out; they never make room
    sim.t = (LOGIN_WAIT - 1) * 1000;
    CHECK(attempt(*c[3], "hunter2").empty());
    sim.t = LOGIN_WAIT * 1000;                              // the others' time is up; c0 retries again, as often as it likes
    CHECK(attempt(*c[0], "wrong").size() == 1 && server.offers_kept() == 1);
    CHECK(server.tick() == LOGIN_WAIT);
    sim.t += LOGIN_WAIT * 1000;
    CHECK(server.tick() == UINT32_MAX && server.offers_kept() == 0);

    // a known client's offer does not use up a stranger's place: three strangers still fit beside it
    CHECK(attempt(*c[4], "hunter2").size() == 1);
    for (size_t i = 0; i < 3; ++i) CHECK(attempt(*c[i], "wrong").size() == 1);
    CHECK(server.offers_kept() == 4);
    sim.t += LOGIN_WAIT * 1000;
    CHECK(server.tick() == UINT32_MAX && server.offers_kept() == 0);

    // a confirmation frees its place at once
    const std::vector<Packet> good = attempt(*c[3], "hunter2");
    for (size_t i = 0; i < 2; ++i) CHECK(attempt(*c[i], "wrong").size() == 1);
    CHECK(good.size() == 1 && server.offers_kept() == 3 && attempt(*c[5], "hunter2").empty());
    settle(*c[3], server, give(*c[3], good));
    CHECK(c[3]->logins.size() == 1 && server.logged_in.size() == 1 && server.logged_in[0].first == c[3]->S);
    CHECK(server.offers_kept() == 2 && attempt(*c[5], "hunter2").size() == 1 && server.offers_kept() == 3);

    // the client: a reply that comes after the wait finds nothing; a new login replaces the one waiting
    sim.t += 100000;
    server.tick();
    c[5]->tick();
    CHECK(server.offers_kept() == 0 && !c[5]->node.login.used);
    const std::vector<Packet> first = attempt(*c[5], "hunter2");
    CHECK(first.size() == 1 && c[5]->tick() == LOGIN_WAIT);
    sim.t += (LOGIN_WAIT - 1) * 1000;
    CHECK(c[5]->tick() == 1 && c[5]->node.login.used);
    sim.t += 1000;
    CHECK(give(*c[5], first).empty() && c[5]->logins.empty() && !c[5]->node.login.used);    // too late
    server.tick();
    const std::vector<Packet> stale = attempt(*c[5], "hunter2");
    sim.t += STEP * 1000;
    const std::vector<Packet> fresh = attempt(*c[5], "hunter2");                             // the retry
    CHECK(stale.size() == 1 && fresh.size() == 1 && server.offers_kept() == 1);
    CHECK(give(*c[5], stale).empty() && c[5]->logins.empty());                               // answers the login it replaced
    settle(*c[5], server, give(*c[5], fresh));
    CHECK(c[5]->logins.size() == 1 && server.logged_in.size() == 2 && server.offers_kept() == 0);

    // a known client's login cannot be replayed into the place kept for it: its timestamp is not new
    c[3]->login(server.S, "hunter2", ++stamp);
    const std::vector<Packet> again = c[3]->take();
    CHECK(give(server, again).size() == 1 && server.offers_kept() == 1);
    sim.t += LOGIN_WAIT * 1000;
    server.tick();
    int dropped = server.logins_dropped;
    CHECK(give(server, again).empty() && server.logins_dropped == dropped + 1 && server.offers_kept() == 0);

    // R_PRE counts logins too: sixteen that do not verify, and the genuine one after them is dropped
    sim.t += 100000;
    server.tick();
    Packet forged = [&] { c[0]->login(server.S, "hunter2", ++stamp); return c[0]->take()[0]; }();
    forged.payload[forged.payload.size() - 1] ^= 1;
    dropped = server.logins_dropped;
    for (int i = 0; i < ZCPFS_R_PRE_BURST; ++i) CHECK(give(server, {forged}).empty());
    CHECK(attempt(*c[0], "hunter2").empty() && server.logins_dropped == dropped + ZCPFS_R_PRE_BURST + 1);
    sim.t += 1000;
    CHECK(attempt(*c[0], "hunter2").size() == 1);

    // arguments and the cases that are not a zcpfs login at all
    zcpfs_login_req req, req_empty;
    uint32_t wait = 7;
    zcpfs_contact v = Host::view(c[1]->contact(server.S));
    const Bytes four(4, 1), big(ZCPFS_LOGIN_PLAIN_MAX + 1, 1), zero(32, 0);
    const uint8_t pw[] = "pw";
    CHECK(zcpfs_login_send(&c[1]->node, &v, pw, 2, big.data(), big.size(), 0, &wait) == ZCPFS_ERR_ARG && wait == 0);
    CHECK(!c[1]->node.login.used && c[1]->take().empty());
    v.pinned = 0;
    CHECK(zcpfs_login_send(&c[1]->node, &v, pw, 2, four.data(), 4, 0, &wait) == ZCPFS_LEGACY);     // SPEC 19.2
    v.pinned = 1;
    v.ss = zero.data();
    CHECK(zcpfs_login_send(&c[1]->node, &v, pw, 2, four.data(), 4, 0, &wait) == ZCPFS_LEGACY);     // SPEC 4
    CHECK(c[1]->take().empty());
    Bytes legacy(1 + 32 + 2 + 16, 0x44);
    legacy[0] = server.S[0];
    CHECK(zcpfs_login_rx(&server.node, legacy.data(), legacy.size(), 0, server.now(), &req) == ZCPFS_LEGACY);
    c[1]->login(server.S, "", ++stamp);
    const Packet typed_nothing = c[1]->take()[0];
    c[1]->login(server.S, "hunter2", ++stamp);
    const Packet genuine = c[1]->take()[0];
    CHECK(genuine.payload.size() == 78 && typed_nothing.payload.size() == 78);
    CHECK(zcpfs_login_rx(&c[2]->node, genuine.payload.data(), genuine.payload.size(), 0, 0, &req) != ZCPFS_OK);   // not a server
    sim.t += LOGIN_WAIT * 1000;
    server.tick();
    CHECK(server.offers_kept() == 0);
    CHECK(zcpfs_login_rx(&server.node, typed_nothing.payload.data(), typed_nothing.payload.size(), 0, server.now(), &req_empty) == ZCPFS_OK);
    CHECK(zcpfs_login_rx(&server.node, genuine.payload.data(), genuine.payload.size(), 0, server.now(), &req) == ZCPFS_OK);
    CHECK(Bytes(req.S_c, req.S_c + 32) == c[1]->S && req.plain_len == 4);
    CHECK(req_empty.empty_password == 1 && req.empty_password == 0);    // all the login says about the password
    Bytes reply(ZCPFS_MAX_PAYLOAD);
    size_t len = 0;
    const Bytes response(ZCPFS_LOGIN_RESPONSE_LEN, 9);
    const zcpfs_branch three[3] = {{pw, 2, response.data(), 1}, {pw, 1, response.data(), 2}, {pw, 0, response.data(), 3}};
    CHECK(zcpfs_login_answer(&server.node, &req, three, 0, TIMEOUT_MS, server.now(), reply.data(), &len) == ZCPFS_ERR_ARG);
    CHECK(zcpfs_login_answer(&server.node, &req, three, 3, TIMEOUT_MS, server.now(), reply.data(), &len) == ZCPFS_ERR_ARG);
    CHECK(server.offers_kept() == 0);                       // a refused answer keeps nothing
    CHECK(zcpfs_login_answer(&server.node, &req, three, 2, TIMEOUT_MS, server.now(), reply.data(), &len) == ZCPFS_OK);
    CHECK(len + 2 == 89 && server.offers_kept() == 1);      // the largest reply there is
    // the offer is kept two steps of the timeout the server gave, here a slower link than the default
    sim.t += STEP * 1000;
    CHECK(zcpfs_login_answer(&server.node, &req, three, 1, 14300, server.now(), reply.data(), &len) == ZCPFS_OK);
    CHECK(len + 2 == 47 && server.tick() == ZCPFS_LOGIN_STEPS * 23);
    CHECK(zcpfs_login_answer(&server.node, &req, three, 1, TIMEOUT_MS, server.now(), reply.data(), &len) == ZCPFS_ERR_FULL);
    zcpfs_forget(&server.node, c[1]->S.data());
    zcpfs_forget(&c[1]->node, server.S.data());
    CHECK(server.offers_kept() == 0 && !c[1]->node.login.used);
    // a table of one has nothing to keep back: a stranger may take it
    zcpfs_offer single[1];
    zcpfs_node_serve(&server.node, single, 1);
    CHECK(zcpfs_login_rx(&server.node, genuine.payload.data(), genuine.payload.size(), 0, server.now(), &req) == ZCPFS_OK);
    CHECK(zcpfs_login_answer(&server.node, &req, three, 1, TIMEOUT_MS, server.now(), reply.data(), &len) == ZCPFS_OK);
    zcpfs_node_serve(&server.node, server.offers.data(), server.offers.size());
    for (const auto& h : sim.nodes) CHECK(h->violations.empty());
}

TEST(zcpfs_login_again, "UNIT-ZCPFS-023", "A client the server already knows logs in again: no handshake is started, the old session stays readable until the new one is used") {
    Sim sim(56);
    LoginSetup n = login_pair(sim);
    Host& client = n.client;
    Host& server = n.server;
    client.login(server.S, "hunter2", 1700000000u);
    settle(client, server, client.take());
    CHECK(client.logins.size() == 1 && server.logged_in.size() == 1);
    CHECK(!client.node.login.used && server.offers_kept() == 0);            // used up, not left to run out
    const uint32_t before = say(client, server, "before", 1700000001u);
    CHECK(client.is_confirmed(before));

    // the server restarts and has lost the session; the client notices nothing and logs in again
    server.boot("restart");
    sim.t = 60000;
    int result = 0;
    const uint32_t queued = server.app_send(client.S, "queued", 1700000059u, 0, &result);   // and has something to say
    CHECK(result == ZCPFS_HELD && of_type(server.take(), REQ, 42) == 1);     // its handshake 1 is lost
    client.login(server.S, "hunter2", 1700000060u);
    const std::vector<Packet> reply = give(server, client.take());
    CHECK(reply.size() == 1 && reply[0].type == RESPONSE && !reply[0].flood);
    const std::vector<Packet> confirmation = give(client, reply);
    CHECK(confirmation.size() == 1 && confirmation[0].payload.size() == 10);
    CHECK(client.record(server.S)->previous.used);                          // the old session, for a late packet
    // the server knows this client as a pinned contact now: its offer is tried before any resync (SPEC 10.2)
    // and what it held for the client goes out the moment the login is confirmed
    const std::vector<Packet> held = give(server, confirmation);
    CHECK(held.size() == 1 && held[0].type == TXT && held[0].payload.size() > 10 && server.hs1_sent == 1);
    CHECK(server.logged_in.size() == 2 && server.record(client.S)->current.used && server.offers_kept() == 0);
    settle(server, client, held);
    CHECK(client.received(server.S, "queued") == 1 && server.is_confirmed(queued));
    const uint32_t after = say(client, server, "after", 1700000061u);
    CHECK(client.is_confirmed(after) && server.received(client.S, "after") == 1);

    // the first packet after a login may be a text instead of the confirmation, if that one was lost
    sim.t = 200000;
    client.login(server.S, "hunter2", 1700000200u);
    give(client, give(server, client.take()));                              // the confirmation is lost
    const uint32_t tok = client.app_send(server.S, "first after login", 1700000201u, 0);
    settle(client, server, client.take());
    CHECK(server.logged_in.size() == 3 && server.received(client.S, "first after login") == 1 && client.is_confirmed(tok));
    CHECK(server.record(client.S)->previous.used && server.record(client.S)->prev_timed);
    CHECK(client.violations.empty() && server.violations.empty());

    // a text that opens nowhere, from a hash that has no offer, is still nobody's
    zcpfs_rx_out o;
    uint8_t who[32];
    const Packet junk = garbage(server, client, TXT, 30, 0x66);
    CHECK(zcpfs_rx_offer(&server.node, junk.payload.data(), junk.payload.size(), server.now(), who, &o) == ZCPFS_RX_NEXT);
}

TEST(zcpfs_login_one_reply, "UNIT-ZCPFS-024",
     "One reply per login: a server's second reply with another guess is not tried; a sealed response does not end a login") {
    {   // a server that guesses: its first reply holds a wrong password, its second the right one
        Sim sim(53);
        LoginSetup n = login_pair(sim);
        n.server.password = "guess one";
        n.client.login(n.server.S, "hunter2", 1700000000u);
        sim.pump(n.client);
        sim.run(20);
        CHECK(sim.count(RESPONSE) == 1 && n.client.logins.empty());
        CHECK(!n.client.node.login.used);                                   // over, though nothing opened
        const Packet& login = sim.first(ANON, 78);
        uint8_t S_c[32], Y_c[32], ss[32], plain[ZCPFS_LOGIN_PLAIN_MAX + 1];
        size_t plain_len = 0;
        int empty = 0;
        CHECK(zcpfs_login_open(n.server.a.data(), n.server.S.data(), login.payload.data() + 1,
                               login.payload.size() - 1, S_c, Y_c, ss, plain, &plain_len, &empty) == ZCPFS_OK);
        const uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN] = {0, 0, 0, 0, 3, 2};
        uint8_t branch[ZCPFS_BRANCH_LEN], reply[ZCPFS_MAX_PAYLOAD];
        size_t reply_len = 0;
        zcpfs_keys keys;
        CHECK(zcpfs_login_branch_make(&n.server.drbg, ss, S_c, n.server.S.data(), Y_c,
                                      reinterpret_cast<const uint8_t *>("hunter2"), 7, response, branch, &keys) == ZCPFS_OK);
        CHECK(zcpfs_login_reply_pack(1, Y_c, branch, reply, &reply_len) == ZCPFS_OK);
        Bytes payload = {n.client.S[0], n.server.S[0]};
        payload.insert(payload.end(), reply, reply + reply_len);
        sim.inject(n.client, Packet{RESPONSE, payload, false});
        sim.run(40);
        CHECK(n.client.logins.empty() && sim.count(TXT) == 0);              // no confirmation tells the server
        check_rules(sim);
    }
    {   // a request's response that arrives while a login waits, and would parse as a login reply
        Sim sim(54);
        LoginSetup n = login_pair(sim);
        n.client.login(n.server.S, "hunter2", 1700000000u);
        const Contact *server = n.client.find(n.server.S.data());
        const uint8_t n6[6] = {1, 0, 0, 0, 0, 0};                         // count byte 1 to the login parser
        Bytes status(40, 0x24);
        uint8_t sealed[ZCPFS_MAX_PAYLOAD];
        size_t sealed_len = 0;
        CHECK(zcpfs_pair_seal(server->ss.data(), n.server.S.data(), n.client.S.data(), RESPONSE, n6,
                              status.data(), status.size(), sealed, &sealed_len) == ZCPFS_OK);
        const uint8_t *branches = nullptr, *echo = nullptr;
        CHECK(zcpfs_login_reply_unpack(sealed + 2, sealed_len - 2, &echo, &branches) == 1);
        n.client.receive(Packet{RESPONSE, Bytes(sealed, sealed + sealed_len), false});
        CHECK(n.client.node.login.used);                                    // opened as a response, the login waits on
        sim.pump(n.client);
        sim.run(30);
        CHECK(n.client.logins.size() == 1);
    }
}

TEST(zcpfs_login_duplicate_branch, "UNIT-ZCPFS-025",
     "A login reply whose two branches share one Y_s is not a reply: no tag is tried, and the real reply still opens") {
    Sim sim(55);
    LoginSetup n = login_pair(sim);
    n.client.login(n.server.S, "hunter2", 1700000000u);
    sim.pump(n.client);                                                     // on the air, not yet delivered
    const Packet& login = sim.first(ANON, 78);
    uint8_t S_c[32], Y_c[32], ss[32], plain[ZCPFS_LOGIN_PLAIN_MAX + 1];
    size_t plain_len = 0;
    int empty = 0;
    CHECK(zcpfs_login_open(n.server.a.data(), n.server.S.data(), login.payload.data() + 1,
                           login.payload.size() - 1, S_c, Y_c, ss, plain, &plain_len, &empty) == ZCPFS_OK);
    const uint8_t response[ZCPFS_LOGIN_RESPONSE_LEN] = {0, 0, 0, 0, 3, 2};
    uint8_t branches[2 * ZCPFS_BRANCH_LEN], reply[ZCPFS_MAX_PAYLOAD];
    size_t reply_len = 0;
    zcpfs_keys keys;
    CHECK(zcpfs_login_branch_make(&n.server.drbg, ss, S_c, n.server.S.data(), Y_c,
                                  reinterpret_cast<const uint8_t *>("hunter2"), 7, response, branches, &keys) == ZCPFS_OK);
    memcpy(branches + ZCPFS_BRANCH_LEN, branches, ZCPFS_BRANCH_LEN);
    branches[2 * ZCPFS_BRANCH_LEN - 1] ^= 1;                                 // same Y_s, another tag
    CHECK(zcpfs_login_reply_pack(2, Y_c, branches, reply, &reply_len) == ZCPFS_OK);
    const uint8_t *echo = nullptr, *b = nullptr;
    CHECK(zcpfs_login_reply_unpack(reply, reply_len, &echo, &b) == ZCPFS_ERR_ARG);
    Bytes payload = {n.client.S[0], n.server.S[0]};
    payload.insert(payload.end(), reply, reply + reply_len);
    n.client.receive(Packet{RESPONSE, payload, false});
    CHECK(n.client.node.login.used && n.client.logins.empty());             // not a reply: the login waits on

    CHECK(zcpfs_login_reply_pack(1, Y_c, branches, reply, &reply_len) == ZCPFS_OK);
    payload = {n.client.S[0], n.server.S[0]};
    payload.insert(payload.end(), reply, reply + reply_len);
    n.client.receive(Packet{RESPONSE, payload, false});
    CHECK(n.client.logins.size() == 1 && !n.client.node.login.used);
}
