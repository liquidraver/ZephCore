// The zcpfs C library (zephcore/lib/zcpfs) against the vectors of its
// specification. Expected values come from vectors.json, which the Python
// reference model in lib/zcpfs/ref generates; zcpfs_vectors.h is that data as
// C tables. Ascon itself is tested in ascon_tests.cpp.
#include "test.h"
#include "zcpfs_vectors.h"
#include <algorithm>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include "zcpfs.h"
#include "zcpfs_internal.h"
}

namespace {
using Bytes = std::vector<uint8_t>;

Bytes raw(const char *data, unsigned len) {
    return Bytes(reinterpret_cast<const uint8_t *>(data), reinterpret_cast<const uint8_t *>(data) + len);
}
// A value of vectors.json by its path.
Bytes V(const std::string& path) {
    for (const auto& row : ZC_VEC_BYTES) if (path == row.path) return raw(row.data, row.len);
    throw std::runtime_error("no vector " + path);
}
long long N(const std::string& path) {
    for (const auto& row : ZC_VEC_NUMS) if (path == row.path) return row.value;
    throw std::runtime_error("no number " + path);
}
std::string T(const std::string& path) {
    for (const auto& row : ZC_VEC_TEXT) if (path == row.path) return row.text;
    throw std::runtime_error("no text " + path);
}
std::string at(const std::string& list, long long index) { return list + "." + std::to_string(index); }

Bytes seq(unsigned start, size_t n) {
    Bytes out(n);
    for (size_t i = 0; i < n; ++i) out[i] = uint8_t(start + i);
    return out;
}
Bytes cat(const Bytes& a, const Bytes& b) { Bytes out(a); out.insert(out.end(), b.begin(), b.end()); return out; }
Bytes sub(const Bytes& a, size_t from, size_t to) { return Bytes(a.begin() + long(from), a.begin() + long(to)); }
Bytes tail(const Bytes& a, size_t from) { return sub(a, from, a.size()); }
bool all_zero(const uint8_t *p, size_t n) { return std::all_of(p, p + n, [](uint8_t b) { return b == 0; }); }
bool all_zero(const Bytes& a) { return all_zero(a.data(), a.size()); }

zcpfs_drbg generator(unsigned start) {
    zcpfs_drbg g;
    const Bytes hw = seq(start, 32), saved(32, 0);
    zcpfs_drbg_seed(&g, hw.data(), saved.data());
    return g;
}
Bytes kdf(const char *label, const std::vector<Bytes>& parts, size_t n) {
    std::vector<zcpfs_part> p;
    for (const auto& part : parts) p.push_back({part.data(), part.size()});
    Bytes out(n);
    zcpfs_kdf(label, p.data(), p.size(), out.data(), out.size());
    return out;
}

// The two ends of the handshake vector.
struct Pair {
    Bytes a_i = V("handshake.initiator.scalar"), S_i = V("handshake.initiator.ed25519_public");
    Bytes a_r = V("handshake.responder.scalar"), S_r = V("handshake.responder.ed25519_public");
    Bytes ss = V("handshake.ss");
    Bytes hdr_ir{S_r[0], S_i[0]}, hdr_ri{S_i[0], S_r[0]};
    Bytes ck_i2r = V("handshake.ck_i2r"), ck_r2i = V("handshake.ck_r2i");
};
zcpfs_session session(const Bytes& ck_s, const Bytes& ck_r, uint32_t now = 0) {
    zcpfs_session s;
    zcpfs_session_init(&s, ck_s.data(), ck_r.data(), now);
    return s;
}
zcpfs_session receiver(const Pair& p, uint32_t now = 0) { return session(Bytes(32, 0), p.ck_i2r, now); }

struct Opened { int rc; Bytes plain; Bytes ack; };
Opened open(zcpfs_session& s, const Bytes& ad, const Bytes& body, uint32_t now = 0) {
    Opened out{0, Bytes(body.size() + 1, 0xEE), Bytes(4, 0)};
    out.rc = zcpfs_open(&s, ad.data(), ad.size(), body.data(), body.size(), out.plain.data(), out.ack.data(), now);
    out.plain.resize(body.size() >= ZCPFS_TAG_LEN ? body.size() - ZCPFS_TAG_LEN : 0);
    return out;
}
struct Sealed { Bytes body; Bytes ack; };
Sealed seal(zcpfs_session& s, const Bytes& ad, const Bytes& plain, uint32_t now = 0) {
    Sealed out{Bytes(plain.size() + ZCPFS_TAG_LEN), Bytes(4)};
    zcpfs_seal(&s, ad.data(), ad.size(), plain.data(), plain.size(), out.body.data(), out.ack.data(), now);
    return out;
}
}

TEST(zcpfs_kdf_auth, "UNIT-ZCPFS-001", "KDF framing and AUTH reproduce the specification vectors") {
    for (long long i = 0; i < N("kdf.#"); ++i) {
        const std::string row = at("kdf", i);
        std::vector<Bytes> parts;
        for (long long j = 0; j < N(row + ".parts.#"); ++j) parts.push_back(V(at(row + ".parts", j)));
        CHECK(kdf(T(row + ".label").c_str(), parts, size_t(N(row + ".n"))) == V(row + ".out"));
    }
    // the framing separates parts: the same bytes split differently give another key
    CHECK(kdf("hs1", {seq(0, 4), seq(4, 4)}, 32) != kdf("hs1", {seq(0, 5), seq(5, 3)}, 32));
    CHECK(kdf("hs1", {seq(0, 8)}, 32) != kdf("hs2", {seq(0, 8)}, 32));
    // a longer output starts with the shorter one
    CHECK(sub(kdf("step", {seq(0, 32)}, 80), 0, 32) == kdf("step", {seq(0, 32)}, 32));

    for (long long i = 0; i < N("auth.#"); ++i) {
        const std::string row = at("auth", i);
        const Bytes k = V(row + ".k"), ad = V(row + ".ad");
        Bytes tag(ZCPFS_TAG_LEN);
        zcpfs_auth(k.data(), ad.data(), ad.size(), tag.data());
        CHECK(tag == V(row + ".tag"));
    }

    const Bytes a = seq(1, 40);
    Bytes b = a;
    CHECK(zcpfs_ct_equal(a.data(), b.data(), a.size()) == 1);
    for (size_t i = 0; i < b.size(); ++i) {
        for (uint8_t bit : {uint8_t(0x01), uint8_t(0x80)}) {
            b[i] ^= bit; CHECK(zcpfs_ct_equal(a.data(), b.data(), a.size()) == 0); b[i] ^= bit;
        }
    }
    zcpfs_wipe(b.data(), b.size());
    CHECK(all_zero(b));
}

TEST(zcpfs_drbg_vectors, "UNIT-ZCPFS-002", "Generator seed, output, key replacement and mixing") {
    const Bytes hw = V("drbg.hw"), saved = V("drbg.saved");
    zcpfs_drbg g;
    zcpfs_drbg_seed(&g, hw.data(), saved.data());
    CHECK(Bytes(g.k, g.k + 32) == V("drbg.k0"));
    for (long long i = 0; i < N("drbg.steps.#"); ++i) {
        const std::string row = at("drbg.steps", i);
        if (T(row + ".op") == "generate") {
            Bytes out(size_t(N(row + ".n")));
            const Bytes before(g.k, g.k + 32);
            zcpfs_drbg_generate(&g, out.data(), out.size());
            CHECK(out == V(row + ".out"));
            CHECK(Bytes(g.k, g.k + 32) != before);
        } else {
            const Bytes x = V(row + ".x");
            zcpfs_drbg_mix(&g, x.data(), x.size());
        }
        CHECK(Bytes(g.k, g.k + 32) == V(row + ".k"));
    }
}

TEST(zcpfs_handshake_vectors, "UNIT-ZCPFS-003", "Handshake packets and keys on both sides; forged and low-order input refused") {
    const Pair p;
    zcpfs_drbg g_i = generator(0x40), g_r = generator(0x50);
    zcpfs_hs_out out;
    Bytes body1(ZCPFS_HS_BODY_LEN), body2(ZCPFS_HS_BODY_LEN), es(32);
    zcpfs_keys keys_r, keys_i;

    CHECK(zcpfs_hs1_make(&g_i, p.ss.data(), p.S_i.data(), p.S_r.data(), &out, body1.data()) == ZCPFS_OK);
    CHECK(Bytes(out.e, out.e + 32) == V("handshake.e_i") && Bytes(out.E, out.E + 32) == V("handshake.E_i"));
    CHECK(Bytes(out.es, out.es + 32) == V("handshake.es"));
    CHECK(cat(p.hdr_ir, body1) == V("handshake.packet_handshake_1"));
    CHECK(tail(body1, 32) == V("handshake.tag1"));

    CHECK(zcpfs_hs1_verify(p.ss.data(), p.a_r.data(), p.S_i.data(), p.S_r.data(), body1.data(), es.data()) == ZCPFS_OK);
    CHECK(es == V("handshake.es"));

    CHECK(zcpfs_hs2_make(&g_r, p.ss.data(), es.data(), p.S_i.data(), p.S_r.data(), body1.data(), body2.data(), &keys_r) == ZCPFS_OK);
    CHECK(cat(p.hdr_ri, body2) == V("handshake.packet_handshake_2"));
    CHECK(sub(body2, 0, 32) == V("handshake.E_r") && tail(body2, 32) == V("handshake.tag2"));
    CHECK(Bytes(keys_r.ck_i2r, keys_r.ck_i2r + 32) == p.ck_i2r && Bytes(keys_r.ck_r2i, keys_r.ck_r2i + 32) == p.ck_r2i);
    CHECK(Bytes(keys_r.feed, keys_r.feed + 32) == V("handshake.drbg_feed"));

    CHECK(zcpfs_hs2_process(p.ss.data(), &out, p.a_i.data(), p.S_i.data(), p.S_r.data(), body2.data(), &keys_i) == ZCPFS_OK);
    CHECK(std::memcmp(&keys_i, &keys_r, sizeof(keys_i)) == 0);

    // every byte of either packet is covered
    for (size_t byte = 0; byte < ZCPFS_HS_BODY_LEN; ++byte) {
        Bytes bad = body1; bad[byte] ^= 1;
        Bytes got(32, 0xEE);
        CHECK(zcpfs_hs1_verify(p.ss.data(), p.a_r.data(), p.S_i.data(), p.S_r.data(), bad.data(), got.data()) != ZCPFS_OK);
        CHECK(all_zero(got));
        bad = body2; bad[byte] ^= 1;
        zcpfs_keys keys;
        std::memset(&keys, 0xEE, sizeof(keys));
        CHECK(zcpfs_hs2_process(p.ss.data(), &out, p.a_i.data(), p.S_i.data(), p.S_r.data(), bad.data(), &keys) != ZCPFS_OK);
        CHECK(all_zero(reinterpret_cast<const uint8_t *>(&keys), sizeof(keys)));
    }
    // the tags bind both identities and the shared secret
    CHECK(zcpfs_hs1_verify(p.ss.data(), p.a_r.data(), p.S_r.data(), p.S_i.data(), body1.data(), es.data()) == ZCPFS_ERR_AUTH);
    Bytes other_ss = p.ss; other_ss[0] ^= 1;
    CHECK(zcpfs_hs1_verify(other_ss.data(), p.a_r.data(), p.S_i.data(), p.S_r.data(), body1.data(), es.data()) == ZCPFS_ERR_AUTH);
    zcpfs_keys keys;
    CHECK(zcpfs_hs2_process(other_ss.data(), &out, p.a_i.data(), p.S_i.data(), p.S_r.data(), body2.data(), &keys) == ZCPFS_ERR_AUTH);
    // a handshake 2 made for another handshake 1 is refused
    zcpfs_hs_out other;
    Bytes other1(ZCPFS_HS_BODY_LEN);
    CHECK(zcpfs_hs1_make(&g_i, p.ss.data(), p.S_i.data(), p.S_r.data(), &other, other1.data()) == ZCPFS_OK);
    CHECK(other1 != body1);
    CHECK(zcpfs_hs2_process(p.ss.data(), &other, p.a_i.data(), p.S_i.data(), p.S_r.data(), body2.data(), &keys) == ZCPFS_ERR_AUTH);

    // an all-zero one-time key gives an all-zero secret and is refused before any tag is checked
    const Bytes zero = V("negative.low_order_ephemeral.E");
    Bytes low(ZCPFS_HS_BODY_LEN, 0);
    CHECK(all_zero(zero) && N("negative.low_order_ephemeral.handshake_1_accepted") == 0);
    CHECK(zcpfs_hs1_verify(p.ss.data(), p.a_r.data(), p.S_i.data(), p.S_r.data(), low.data(), es.data()) == ZCPFS_ERR_WEAK);
    CHECK(zcpfs_hs2_process(p.ss.data(), &out, p.a_i.data(), p.S_i.data(), p.S_r.data(), low.data(), &keys) == ZCPFS_ERR_WEAK);
    CHECK(zcpfs_hs2_make(&g_r, p.ss.data(), es.data(), p.S_i.data(), p.S_r.data(), low.data(), body2.data(), &keys) == ZCPFS_ERR_WEAK);
}

TEST(zcpfs_session_vectors, "UNIT-ZCPFS-004", "Chain steps, sealed packets and ACK values in both directions") {
    const Pair p;
    zcpfs_session i = session(p.ck_i2r, p.ck_r2i), r = session(p.ck_r2i, p.ck_i2r);
    for (const char *direction : {"session.i_to_r", "session.r_to_i"}) {
        const bool forward = std::string(direction) == "session.i_to_r";
        zcpfs_session& sender = forward ? i : r;
        zcpfs_session& peer = forward ? r : i;
        const Bytes hdr = forward ? p.hdr_ir : p.hdr_ri;
        CHECK(N(std::string(direction) + ".#") == 10);
        for (long long n = 0; n < N(std::string(direction) + ".#"); ++n) {
            const std::string row = at(direction, n);
            const Bytes plain = V(row + ".plaintext"), padded = V(row + ".sealed_plaintext"), ad = V(row + ".ad");
            CHECK(ad == cat(hdr, {ZCPFS_PT_TXT}));
            CHECK(zcpfs_fit(ZCPFS_PT_TXT, plain.size(), ZCPFS_DATA_OVERHEAD) == padded.size());
            const Sealed sent = seal(sender, ad, padded, uint32_t(100 + n));
            CHECK(cat(hdr, sent.body) == V(row + ".payload"));
            CHECK(sent.ack == V(row + ".ack4"));
            CHECK(sent.body.size() + 2 <= ZCPFS_MAX_PAYLOAD && (sent.body.size() + 2) % 16 != 4);
            CHECK(sender.t_last == uint32_t(100 + n));
            if (forward && n < N("session.chain_i2r.#")) {
                CHECK(Bytes(sender.ck_s, sender.ck_s + 32) == V(at("session.chain_i2r", n) + ".ck_after"));
            }
            const Opened got = open(peer, ad, sent.body, uint32_t(200 + n));
            CHECK(got.rc == ZCPFS_OK && got.plain == padded && got.ack == sent.ack);
            CHECK(peer.opened == 1 && peer.fails == 0 && peer.n_skipped == 0 && peer.t_last == uint32_t(200 + n));
            CHECK(std::memcmp(peer.ck_r, sender.ck_s, 32) == 0);
        }
    }
    // the empty confirmation packet of 8.4: 10 bytes on the air
    zcpfs_session fresh = session(p.ck_i2r, p.ck_r2i);
    CHECK(cat(p.hdr_ir, seal(fresh, cat(p.hdr_ir, {ZCPFS_PT_TXT}), {}).body) == V("session.confirmation"));
    CHECK(V("session.confirmation").size() == 10);
    // a text whose packet would look block-aligned gets one zero byte
    CHECK(zcpfs_fit(ZCPFS_PT_TXT, V("negative.alignment.plaintext").size(), ZCPFS_DATA_OVERHEAD) ==
          V("negative.alignment.sealed_plaintext").size());
}

TEST(zcpfs_session_window, "UNIT-ZCPFS-005", "Loss and reordering: trial window, skipped keys, their limit and expiry") {
    const Pair p;
    const Bytes ad = V("window.ad");
    CHECK(N("window.W") == ZCPFS_W && N("window.S") == ZCPFS_S);
    std::vector<Bytes> bodies;
    for (long long n = 0; n < N("window.bodies.#"); ++n) bodies.push_back(V(at("window.bodies", n)));

    zcpfs_session r = receiver(p);
    for (long long n = 0; n < N("window.open_order.#"); ++n) {
        const std::string row = at("window.open_order", n);
        const size_t position = size_t(N(row + ".position"));
        const Opened got = open(r, ad, bodies[position]);
        CHECK((got.rc == ZCPFS_OK) == (N(row + ".opens") == 1));
        CHECK(got.plain == Bytes({'m', uint8_t('0' + position / 10), uint8_t('0' + position % 10)}));
        CHECK(r.n_skipped == N(row + ".skipped_after") && r.fails == uint32_t(N(row + ".fails_after")));
    }

    // W positions are tried: the last one opens, the one after it does not and changes nothing
    zcpfs_session far = receiver(p);
    const zcpfs_session before = far;
    Opened got = open(far, ad, bodies[ZCPFS_W]);
    CHECK(got.rc == ZCPFS_ERR_AUTH && N("window.position_W_opens") == 0 && all_zero(got.plain));
    CHECK(far.fails == 1 && far.opened == 0 && far.n_skipped == 0 && std::memcmp(far.ck_r, before.ck_r, 32) == 0);
    CHECK(open(far, ad, bodies[ZCPFS_W - 1]).rc == ZCPFS_OK && N("window.position_W_minus_1_opens") == 1);
    // only the last S skipped positions are kept
    CHECK(far.n_skipped == ZCPFS_S);
    CHECK(open(far, ad, bodies[ZCPFS_W - 2 - ZCPFS_S]).rc == ZCPFS_ERR_AUTH);
    for (size_t position = ZCPFS_W - 1 - ZCPFS_S; position < ZCPFS_W - 1; ++position) {
        CHECK(open(far, ad, bodies[position]).rc == ZCPFS_OK);
    }
    CHECK(far.n_skipped == 0);
    CHECK(open(far, ad, bodies[ZCPFS_W]).rc == ZCPFS_OK);

    // a skipped key is used once and expires T_SKIP after it was stepped over
    zcpfs_session s = receiver(p);
    CHECK(open(s, ad, bodies[2], 1000).rc == ZCPFS_OK && s.n_skipped == 2);
    CHECK(open(s, ad, bodies[0], 1000 + ZCPFS_T_SKIP - 1).rc == ZCPFS_OK && s.n_skipped == 1);
    CHECK(open(s, ad, bodies[0], 1000 + ZCPFS_T_SKIP - 1).rc == ZCPFS_ERR_AUTH);
    CHECK(open(s, ad, bodies[1], 1000 + ZCPFS_T_SKIP).rc == ZCPFS_ERR_AUTH && s.n_skipped == 0);
    // the same across a wrap of the 32-bit clock
    s = receiver(p);
    CHECK(open(s, ad, bodies[1], 0xFFFFFFF0u).rc == ZCPFS_OK && s.n_skipped == 1);
    CHECK(open(s, ad, bodies[0], 0x00000010u).rc == ZCPFS_OK && s.n_skipped == 0);
    s = receiver(p);
    CHECK(open(s, ad, bodies[1], 0xFFFFFFF0u).rc == ZCPFS_OK);
    CHECK(open(s, ad, bodies[0], 0xFFFFFFF0u + ZCPFS_T_SKIP).rc == ZCPFS_ERR_AUTH && s.n_skipped == 0);
}

TEST(zcpfs_session_negative, "UNIT-ZCPFS-006", "Tampered, misaddressed, replayed and truncated packets do not open or move the chain") {
    const Pair p;
    const Bytes body = V("negative.body"), ad = V("negative.ad");
    zcpfs_session s = receiver(p);
    const zcpfs_session before = s;

    Bytes bad = body; bad[0] ^= 1;
    CHECK(open(s, ad, bad).rc == ZCPFS_ERR_AUTH && N("negative.flipped_ciphertext_opens") == 0);
    bad = body; bad.back() ^= 1;
    CHECK(open(s, ad, bad).rc == ZCPFS_ERR_AUTH && N("negative.flipped_tag_opens") == 0);
    CHECK(open(s, {uint8_t(ad[0] ^ 1), ad[1], ad[2]}, body).rc == ZCPFS_ERR_AUTH && N("negative.other_dst_opens") == 0);
    CHECK(open(s, {ad[0], ad[1], ZCPFS_PT_REQ}, body).rc == ZCPFS_ERR_AUTH);
    for (size_t bit = 0; bit < body.size() * 8; ++bit) {
        bad = body; bad[bit / 8] ^= uint8_t(1u << (bit % 8));
        const Opened got = open(s, ad, bad);
        CHECK(got.rc == ZCPFS_ERR_AUTH && all_zero(got.plain));
    }
    CHECK(open(s, ad, sub(body, 0, ZCPFS_TAG_LEN - 1)).rc == ZCPFS_ERR_ARG);
    CHECK(open(s, ad, sub(body, 0, body.size() - 1)).rc == ZCPFS_ERR_AUTH);

    // none of that moved the chain: the intact packet still opens, once
    CHECK(std::memcmp(s.ck_r, before.ck_r, 32) == 0 && s.n_skipped == 0 && s.opened == 0);
    CHECK(s.fails == 5 + body.size() * 8);
    const Opened got = open(s, ad, body);
    CHECK(got.rc == ZCPFS_OK && N("negative.intact_opens") == 1 && N("negative.replay.first_opens") == 1);
    CHECK(got.plain == Bytes({'i', 'n', 't', 'a', 'c', 't'}));
    CHECK(open(s, ad, body).rc == ZCPFS_ERR_AUTH && N("negative.replay.second_opens") == 0);
    // nor does the other direction's chain open it
    zcpfs_session wrong = session(Bytes(32, 0), p.ck_r2i);
    CHECK(open(wrong, ad, body).rc == ZCPFS_ERR_AUTH);
}

TEST(zcpfs_probe_vectors, "UNIT-ZCPFS-007", "Path probe packet, and its binding to direction, identities and nonce") {
    const Pair p;
    const Bytes nonce = V("probe.nonce4");
    Bytes body(ZCPFS_PROBE_BODY_LEN);
    zcpfs_probe_make(p.ss.data(), p.S_i.data(), p.S_r.data(), nonce.data(), body.data());
    CHECK(cat(p.hdr_ir, body) == V("probe.payload") && V("probe.payload").size() == 14);
    CHECK(zcpfs_probe_verify(p.ss.data(), p.S_i.data(), p.S_r.data(), body.data()) == ZCPFS_OK);
    CHECK(zcpfs_probe_verify(p.ss.data(), p.S_r.data(), p.S_i.data(), body.data()) == ZCPFS_ERR_AUTH);
    Bytes other_ss = p.ss; other_ss[31] ^= 1;
    CHECK(zcpfs_probe_verify(other_ss.data(), p.S_i.data(), p.S_r.data(), body.data()) == ZCPFS_ERR_AUTH);
    for (size_t byte = 0; byte < body.size(); ++byte) {
        Bytes bad = body; bad[byte] ^= 1;
        CHECK(zcpfs_probe_verify(p.ss.data(), p.S_i.data(), p.S_r.data(), bad.data()) == ZCPFS_ERR_AUTH);
    }
}

TEST(zcpfs_channel_vectors, "UNIT-ZCPFS-008", "Channel packets, length limits and the never-block-aligned rule") {
    const Bytes secret = V("channel.secret_field"), n6 = V("channel.n6");
    const uint8_t hash = V("channel.channel_hash")[0];
    CHECK(secret.size() == 32);
    for (long long n = 0; n < N("channel.packets.#"); ++n) {
        const std::string row = at("channel.packets", n);
        const Bytes plain = V(row + ".plaintext"), expected = V(row + ".payload");
        const uint8_t type = uint8_t(N(row + ".type"));
        Bytes payload(plain.size() + ZCPFS_CHAN_OVERHEAD + 1), back(payload.size());
        size_t len = 0, plain_len = 0;
        CHECK(zcpfs_chan_seal(secret.data(), hash, type, n6.data(), plain.data(), plain.size(), payload.data(), &len) == ZCPFS_OK);
        payload.resize(len);
        CHECK(payload == expected);
        CHECK(zcpfs_chan_open(secret.data(), hash, type, payload.data(), len, back.data(), &plain_len) == ZCPFS_OK);
        CHECK(plain_len >= plain.size() && plain_len <= plain.size() + 1);
        CHECK(std::equal(plain.begin(), plain.end(), back.begin()) && all_zero(back.data() + plain.size(), plain_len - plain.size()));

        // the packet type, the channel and every byte are covered
        const uint8_t other = type == ZCPFS_PT_GRP_TXT ? ZCPFS_PT_GRP_DATA : ZCPFS_PT_GRP_TXT;
        CHECK(zcpfs_chan_open(secret.data(), hash, other, payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        CHECK(zcpfs_chan_open(secret.data(), uint8_t(hash ^ 1), type, payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        Bytes other_secret = secret; other_secret[20] ^= 1;
        CHECK(zcpfs_chan_open(other_secret.data(), hash, type, payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        for (size_t byte = 0; byte < len; ++byte) {
            Bytes bad = payload; bad[byte] ^= 1;
            CHECK(zcpfs_chan_open(secret.data(), hash, type, bad.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        }
    }
    // every length: fits the packet, never has the length of a legacy channel packet, comes back
    const Bytes text = seq(1, ZCPFS_CHAN_MAX_PLAIN + 1);
    Bytes payload(ZCPFS_MAX_PAYLOAD + 1, 0xEE), back(ZCPFS_MAX_PAYLOAD);
    for (size_t n = 0; n <= ZCPFS_CHAN_MAX_PLAIN; ++n) {
        size_t len = 0, plain_len = 0;
        CHECK(zcpfs_chan_seal(secret.data(), hash, ZCPFS_PT_GRP_TXT, n6.data(), text.data(), n, payload.data(), &len) == ZCPFS_OK);
        CHECK(len <= ZCPFS_MAX_PAYLOAD && len % 16 != 3 && len - n - ZCPFS_CHAN_OVERHEAD <= 1 && payload[ZCPFS_MAX_PAYLOAD] == 0xEE);
        CHECK(zcpfs_chan_open(secret.data(), hash, ZCPFS_PT_GRP_TXT, payload.data(), len, back.data(), &plain_len) == ZCPFS_OK);
        CHECK(std::equal(text.begin(), text.begin() + long(n), back.begin()));
    }
    size_t len = 0;
    CHECK(zcpfs_chan_seal(secret.data(), hash, ZCPFS_PT_GRP_TXT, n6.data(), text.data(), ZCPFS_CHAN_MAX_PLAIN + 1, payload.data(), &len) == ZCPFS_ERR_ARG);
    // legacy lengths and fragments are not tried
    for (size_t n : {size_t(0), size_t(14), size_t(19), size_t(35), size_t(ZCPFS_MAX_PAYLOAD + 1)}) {
        CHECK(zcpfs_chan_open(secret.data(), hash, ZCPFS_PT_GRP_TXT, payload.data(), n, back.data(), &len) == ZCPFS_ERR_ARG);
    }
}

TEST(zcpfs_pairwise_vectors, "UNIT-ZCPFS-009", "Requests, responses and path returns; reserved and block-aligned lengths avoided") {
    const Pair p;
    const Bytes n6 = V("pairwise.n6");
    CHECK(V("pairwise.sender") == p.S_i && V("pairwise.receiver") == p.S_r);
    for (long long n = 0; n < N("pairwise.packets.#"); ++n) {
        const std::string row = at("pairwise.packets", n);
        const Bytes plain = V(row + ".plaintext"), padded = V(row + ".sealed_plaintext"), expected = V(row + ".payload");
        const uint8_t type = uint8_t(N(row + ".type"));
        Bytes payload(plain.size() + ZCPFS_PAIR_OVERHEAD + 1), back(payload.size());
        size_t len = 0, plain_len = 0;
        CHECK(zcpfs_fit(type, plain.size(), ZCPFS_PAIR_OVERHEAD) == padded.size());
        CHECK(zcpfs_pair_seal(p.ss.data(), p.S_i.data(), p.S_r.data(), type, n6.data(), plain.data(), plain.size(), payload.data(), &len) == ZCPFS_OK);
        payload.resize(len);
        CHECK(payload == expected);
        CHECK(zcpfs_pair_open(p.ss.data(), p.S_i.data(), p.S_r.data(), type, payload.data(), len, back.data(), &plain_len) == ZCPFS_OK);
        back.resize(plain_len);
        CHECK(back == padded);

        // direction, packet type, shared secret and every byte are covered
        back.resize(len);
        CHECK(zcpfs_pair_open(p.ss.data(), p.S_r.data(), p.S_i.data(), type, payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        CHECK(zcpfs_pair_open(p.ss.data(), p.S_i.data(), p.S_r.data(), uint8_t(type ^ 8), payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        Bytes other_ss = p.ss; other_ss[7] ^= 1;
        CHECK(zcpfs_pair_open(other_ss.data(), p.S_i.data(), p.S_r.data(), type, payload.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        for (size_t byte = 0; byte < len; ++byte) {
            Bytes bad = payload; bad[byte] ^= 1;
            CHECK(zcpfs_pair_open(p.ss.data(), p.S_i.data(), p.S_r.data(), type, bad.data(), len, back.data(), &plain_len) == ZCPFS_ERR_AUTH);
        }
    }

    // section 7: the lengths a sealed packet may not have
    CHECK(zcpfs_fit(ZCPFS_PT_REQ, 26, ZCPFS_PAIR_OVERHEAD) == 27);       // 42: handshake 1
    CHECK(zcpfs_fit(ZCPFS_PT_RESPONSE, 26, ZCPFS_PAIR_OVERHEAD) == 27);  // 42: handshake 2
    CHECK(zcpfs_fit(ZCPFS_PT_PATH, 26, ZCPFS_PAIR_OVERHEAD) == 26);
    CHECK(zcpfs_fit(ZCPFS_PT_REQ, 4, ZCPFS_DATA_OVERHEAD) == 5);         // 14: probe
    CHECK(zcpfs_fit(ZCPFS_PT_TXT, 4, ZCPFS_DATA_OVERHEAD) == 4);
    CHECK(zcpfs_fit(ZCPFS_PT_TXT, 10, ZCPFS_DATA_OVERHEAD) == 11);       // 20 = 4 + 16
    CHECK(zcpfs_fit(ZCPFS_PT_PATH, 4, ZCPFS_PAIR_OVERHEAD) == 5);        // 20
    CHECK(zcpfs_fit(ZCPFS_PT_ANON, 4, ZCPFS_LOGIN_OVERHEAD) == 4);       // 78
    CHECK(zcpfs_fit(ZCPFS_PT_ANON, 9, ZCPFS_LOGIN_OVERHEAD) == 10);      // 83 = 35 + 48
    const Bytes data = seq(0, ZCPFS_MAX_PAYLOAD);
    Bytes payload(ZCPFS_MAX_PAYLOAD + 1, 0xEE), back(ZCPFS_MAX_PAYLOAD);
    for (uint8_t type : {uint8_t(ZCPFS_PT_REQ), uint8_t(ZCPFS_PT_RESPONSE), uint8_t(ZCPFS_PT_PATH)}) {
        for (size_t n = 0; n <= ZCPFS_MAX_PAYLOAD; ++n) {
            size_t len = 0, plain_len = 0;
            const int rc = zcpfs_pair_seal(p.ss.data(), p.S_i.data(), p.S_r.data(), type, n6.data(), data.data(), n, payload.data(), &len);
            if (n > ZCPFS_MAX_PAYLOAD - ZCPFS_PAIR_OVERHEAD) { CHECK(rc == ZCPFS_ERR_ARG); continue; }
            CHECK(rc == ZCPFS_OK && len <= ZCPFS_MAX_PAYLOAD && len % 16 != 4 && len - n - ZCPFS_PAIR_OVERHEAD <= 1);
            CHECK(!(type != ZCPFS_PT_PATH && len == 42) && payload[ZCPFS_MAX_PAYLOAD] == 0xEE);
            CHECK(zcpfs_pair_open(p.ss.data(), p.S_i.data(), p.S_r.data(), type, payload.data(), len, back.data(), &plain_len) == ZCPFS_OK);
            CHECK(std::equal(data.begin(), data.begin() + long(n), back.begin()) && all_zero(back.data() + n, plain_len - n));
        }
    }
    size_t len = 0;
    CHECK(zcpfs_pair_open(p.ss.data(), p.S_i.data(), p.S_r.data(), ZCPFS_PT_REQ, payload.data(), ZCPFS_PAIR_OVERHEAD - 1, back.data(), &len) == ZCPFS_ERR_ARG);
}

TEST(zcpfs_login_vectors, "UNIT-ZCPFS-010", "Password login: generator, both packets, session keys; a wrong password opens no branch") {
    const Pair p;
    const Bytes S_c = V("login.client"), S_s = V("login.server"), password = V("login.password");
    const Bytes stamp = V("login.login_plaintext"), reply_plain = V("login.reply_plaintext");
    CHECK(S_c == p.S_i && S_s == p.S_r && reply_plain.size() == ZCPFS_LOGIN_RESPONSE_LEN);

    // 19.1: the point both sides derive from the password
    for (long long n = 0; n < N("elligator.#"); ++n) {
        const Bytes in = V(at("elligator", n) + ".in");
        Bytes out(32);
        zcpfs_ell2(out.data(), in.data());
        CHECK(out == V(at("elligator", n) + ".out"));
    }
    CHECK(kdf("pake.gen", {password, S_c, S_s}, 32) == V("login.generator_input"));
    Bytes G(32);
    zcpfs_ell2(G.data(), V("login.generator_input").data());
    CHECK(G == V("login.generator"));

    // client
    zcpfs_drbg g_c = generator(0x41), g_s = generator(0x51), g_other = generator(0x52), g_empty = generator(0x41);
    zcpfs_login_out out, out_empty;
    Bytes login(stamp.size() + ZCPFS_LOGIN_OVERHEAD), login_empty(stamp.size() + ZCPFS_LOGIN_OVERHEAD);
    size_t login_len = 0, empty_len = 0;
    CHECK(zcpfs_login_make(&g_c, p.ss.data(), S_c.data(), S_s.data(), password.data(), password.size(),
                           stamp.data(), stamp.size(), &out, login.data(), &login_len) == ZCPFS_OK);
    login.resize(login_len);
    CHECK(Bytes(out.y, out.y + 32) == V("login.y_c") && Bytes(out.Y, out.Y + 32) == V("login.Y_c"));
    CHECK(cat({S_s[0]}, login) == V("login.packet_login"));
    CHECK(login_len + 1 == stamp.size() + ZCPFS_LOGIN_OVERHEAD && (login_len + 1) % 16 != 3);
    CHECK(zcpfs_login_make(&g_empty, p.ss.data(), S_c.data(), S_s.data(), password.data(), 0,
                           stamp.data(), stamp.size(), &out_empty, login_empty.data(), &empty_len) == ZCPFS_OK);
    login_empty.resize(empty_len);
    CHECK(cat({S_s[0]}, login_empty) == V("login.packet_login_empty_password"));

    // server: who is asking, and of the password only whether there is one
    Bytes got_S(32), got_Y(32), got_ss(32), got_plain(login_len);
    size_t plain_len = 0;
    int empty = -1;
    CHECK(zcpfs_login_open(p.a_r.data(), S_s.data(), login.data(), login_len, got_S.data(), got_Y.data(),
                           got_ss.data(), got_plain.data(), &plain_len, &empty) == ZCPFS_OK);
    got_plain.resize(plain_len);
    CHECK(got_S == S_c && got_Y == V("login.Y_c") && got_ss == p.ss && empty == 0);
    CHECK(cat({0}, got_plain) == V("login.sealed_login_plaintext") && got_plain == stamp);
    Bytes scratch(login_len);
    CHECK(zcpfs_login_open(p.a_r.data(), S_s.data(), login_empty.data(), empty_len, got_S.data(), got_Y.data(),
                           got_ss.data(), scratch.data(), &plain_len, &empty) == ZCPFS_OK);
    CHECK(empty == 1 && plain_len == stamp.size() && Bytes(scratch.begin(), scratch.begin() + long(plain_len)) == stamp);
    for (size_t byte = 0; byte < login_len; ++byte) {
        Bytes bad = login; bad[byte] ^= 1;
        CHECK(zcpfs_login_open(p.a_r.data(), S_s.data(), bad.data(), login_len, got_S.data(), got_Y.data(),
                               got_ss.data(), scratch.data(), &plain_len, &empty) != ZCPFS_OK);
    }
    CHECK(zcpfs_login_open(p.a_r.data(), S_s.data(), login.data(), 72, got_S.data(), got_Y.data(), got_ss.data(),
                           scratch.data(), &plain_len, &empty) == ZCPFS_ERR_ARG);
    // the longest login plaintext that fits, and one byte more
    Bytes big(ZCPFS_LOGIN_PLAIN_MAX + 1, 7), big_login(ZCPFS_MAX_PAYLOAD + 8), big_plain(ZCPFS_LOGIN_PLAIN_MAX + 1);
    zcpfs_login_out big_out;
    CHECK(zcpfs_login_make(&g_c, p.ss.data(), S_c.data(), S_s.data(), password.data(), password.size(),
                           big.data(), ZCPFS_LOGIN_PLAIN_MAX + 1, &big_out, big_login.data(), &login_len) == ZCPFS_ERR_ARG);
    CHECK(zcpfs_login_make(&g_c, p.ss.data(), S_c.data(), S_s.data(), password.data(), password.size(),
                           big.data(), ZCPFS_LOGIN_PLAIN_MAX, &big_out, big_login.data(), &login_len) == ZCPFS_OK);
    CHECK(login_len + 1 == ZCPFS_MAX_PAYLOAD);
    CHECK(zcpfs_login_open(p.a_r.data(), S_s.data(), big_login.data(), login_len, got_S.data(), got_Y.data(),
                           got_ss.data(), big_plain.data(), &plain_len, &empty) == ZCPFS_OK && plain_len == ZCPFS_LOGIN_PLAIN_MAX);

    // one branch with the right password, one with another
    Bytes right(ZCPFS_BRANCH_LEN), wrong(ZCPFS_BRANCH_LEN);
    zcpfs_keys keys_s, keys_wrong, keys_c;
    CHECK(zcpfs_login_branch_make(&g_s, p.ss.data(), S_c.data(), S_s.data(), V("login.Y_c").data(), password.data(),
                                  password.size(), reply_plain.data(), right.data(), &keys_s) == ZCPFS_OK);
    CHECK(right == V("login.branch_right_password") && sub(right, 0, 32) == V("login.Y_s"));
    CHECK(Bytes(keys_s.ck_i2r, keys_s.ck_i2r + 32) == V("login.ck_i2r") && Bytes(keys_s.ck_r2i, keys_s.ck_r2i + 32) == V("login.ck_r2i"));
    const Bytes hunter3{'h', 'u', 'n', 't', 'e', 'r', '3'};
    CHECK(zcpfs_login_branch_make(&g_other, p.ss.data(), S_c.data(), S_s.data(), V("login.Y_c").data(), hunter3.data(),
                                  hunter3.size(), reply_plain.data(), wrong.data(), &keys_wrong) == ZCPFS_OK);
    CHECK(wrong == V("login.branch_other_password"));

    Bytes branches = cat(right, wrong), reply(1 + branches.size() + 1);
    size_t reply_len = 0;
    CHECK(zcpfs_login_reply_pack(2, branches.data(), reply.data(), &reply_len) == ZCPFS_OK);
    reply.resize(reply_len);
    CHECK(cat(p.hdr_ri, reply) == V("login.packet_reply_two_branches"));
    CHECK(reply_len + 2 == 87 && (reply_len + 2) % 16 != 4);
    Bytes single(1 + ZCPFS_BRANCH_LEN + 1);
    size_t single_len = 0;
    CHECK(zcpfs_login_reply_pack(1, right.data(), single.data(), &single_len) == ZCPFS_OK);
    CHECK(single_len + 2 == 45);                            // one branch: 45 bytes as a RESPONSE, not the reserved 42

    // client: the branch of its own password opens and gives the server's session keys
    const uint8_t *first = nullptr;
    CHECK(zcpfs_login_reply_unpack(reply.data(), reply_len, &first) == 2 && first == reply.data() + 1);
    Bytes response(ZCPFS_LOGIN_RESPONSE_LEN);
    CHECK(zcpfs_login_branch_open(p.ss.data(), &out, S_c.data(), S_s.data(), first, response.data(), &keys_c) == ZCPFS_OK);
    CHECK(response == reply_plain && std::memcmp(&keys_c, &keys_s, sizeof(keys_c)) == 0);
    std::memset(&keys_c, 0xEE, sizeof(keys_c));
    CHECK(zcpfs_login_branch_open(p.ss.data(), &out, S_c.data(), S_s.data(), first + ZCPFS_BRANCH_LEN, response.data(), &keys_c) == ZCPFS_ERR_AUTH);
    CHECK(N("login.other_branch_opens") == 0 && all_zero(reinterpret_cast<const uint8_t *>(&keys_c), sizeof(keys_c)));
    for (size_t byte = 0; byte < ZCPFS_BRANCH_LEN; ++byte) {    // the 4-byte tag still covers every byte
        Bytes bad = right; bad[byte] ^= 1;
        CHECK(zcpfs_login_branch_open(p.ss.data(), &out, S_c.data(), S_s.data(), bad.data(), response.data(), &keys_c) != ZCPFS_OK);
    }

    // a client that typed another password gets a login the server answers, and no branch it can open
    zcpfs_drbg g_typo = generator(0x41);
    zcpfs_login_out typo;
    Bytes typo_login(stamp.size() + ZCPFS_LOGIN_OVERHEAD);
    CHECK(zcpfs_login_make(&g_typo, p.ss.data(), S_c.data(), S_s.data(), hunter3.data(), hunter3.size(),
                           stamp.data(), stamp.size(), &typo, typo_login.data(), &login_len) == ZCPFS_OK);
    CHECK(Bytes(typo.Y, typo.Y + 32) != V("login.Y_c"));
    CHECK(zcpfs_login_branch_open(p.ss.data(), &typo, S_c.data(), S_s.data(), right.data(), response.data(), &keys_c) == ZCPFS_ERR_AUTH);
    // and so does one that typed nothing: the empty password is a password like any other to the key exchange
    CHECK(zcpfs_login_branch_open(p.ss.data(), &out_empty, S_c.data(), S_s.data(), right.data(), response.data(), &keys_c) == ZCPFS_ERR_AUTH);

    // malformed replies
    Bytes bad = reply;
    for (uint8_t count : {uint8_t(0), uint8_t(3), uint8_t(255)}) {
        bad[0] = count;
        CHECK(zcpfs_login_reply_unpack(bad.data(), reply_len, &first) == ZCPFS_ERR_ARG);
    }
    CHECK(zcpfs_login_reply_unpack(reply.data(), 0, &first) == ZCPFS_ERR_ARG);
    CHECK(zcpfs_login_reply_unpack(reply.data(), 1 + 2 * ZCPFS_BRANCH_LEN - 1, &first) == ZCPFS_ERR_ARG);
    CHECK(zcpfs_login_reply_pack(0, branches.data(), reply.data(), &reply_len) == ZCPFS_ERR_ARG);
    CHECK(zcpfs_login_reply_pack(3, branches.data(), reply.data(), &reply_len) == ZCPFS_ERR_ARG);
}
