#pragma once
#include "test.h"
#include <mesh/Dispatcher.h>
#include <mesh/StaticPoolPacketManager.h>
#include <algorithm>
#include <deque>

namespace test_support {
struct Clock : mesh::MillisecondClock {
    uint32_t now = 100;
    unsigned long getMillis() override { return now; }
};
struct Radio : mesh::Radio {
    bool receiving = false, ready = true, recv_mode = true, active = false;
    bool accept = true, completion = false, block_final_gate = false;
    unsigned receiving_queries = 0, attempts = 0, finished = 0, recoveries = 0;
    unsigned completion_queries = 0, relaxations = 0, cad_ticks = 0, radio_ticks = 0;
    uint32_t airtime = 100, maintenance = mesh::MAINTENANCE_IDLE;
    int8_t cad_offset = 0;
    std::deque<std::vector<uint8_t>> incoming;
    std::vector<std::vector<uint8_t>> sent;
    int recvRaw(uint8_t *bytes, int size) override {
        if (incoming.empty()) return 0;
        auto raw = incoming.front(); incoming.pop_front();
        CHECK(raw.size() <= unsigned(size));
        std::copy(raw.begin(), raw.end(), bytes); return int(raw.size());
    }
    uint32_t getEstAirtimeFor(int) override { return airtime; }
    float packetScore(float, int) override { return 0.5f; }
    bool startSendRaw(const uint8_t *bytes, int len) override {
        ++attempts;
        if (!accept) return false;
        sent.emplace_back(bytes, bytes + len); active = true; recv_mode = false;
        return true;
    }
    bool isSendComplete() override {
        ++completion_queries;
        if (!completion) return false;
        completion = false; active = false; return true;
    }
    void onSendFinished() override { ++finished; active = false; recv_mode = true; }
    bool isTxActive() const override { return active; }
    bool isInRecvMode() const override { return recv_mode; }
    bool isReceiving() override {
        ++receiving_queries;
        return receiving || (block_final_gate && receiving_queries == 2);
    }
    bool isRadioReady() override { return ready; }
    void recoverRxState() override { ++recoveries; receiving = false; ready = true; recv_mode = true; }
    float getLastSNR() const override { return -2.25f; }
    float getLastRSSI() const override { return -90.0f; }
    bool cadRelaxOnTxStarvation() override { ++relaxations; return true; }
    void cadMaintenance() override { ++cad_ticks; }
    void radioMaintenance() override { ++radio_ticks; }
    int8_t getCadOffset() const override { return cad_offset; }
    uint32_t msUntilNextMaintenance() override { return maintenance; }
};
struct Dispatcher : mesh::Dispatcher {
    uint8_t duty = 0;
    uint32_t window = 1000;
    mesh::DispatcherAction action = ACTION_RELEASE;
    std::vector<mesh::Packet> received;
    std::vector<mesh::Packet*> held;
    std::vector<uint32_t> wakes;
    std::vector<int8_t> offsets;
    unsigned successes = 0, failures = 0;
    Dispatcher(Radio& radio, Clock& clock, mesh::PacketManager& pool) : mesh::Dispatcher(radio, clock, pool) {
        setTxQueuedCallback([](uint32_t delay, void *self) {
            static_cast<Dispatcher*>(self)->wakes.push_back(delay);
        }, this);
    }
    mesh::DispatcherAction onRecvPacket(mesh::Packet *p) override {
        received.push_back(*p);
        if (action == ACTION_MANUAL_HOLD) held.push_back(p);
        return action;
    }
    uint8_t getDutyCyclePercent() const override { return duty; }
    uint32_t getDutyCycleWindowMs() const override { return window; }
    uint32_t getCADFailRetryDelay() const override { return 100; }
    void logTx(mesh::Packet*, int) override { ++successes; }
    void logTxFail(mesh::Packet*, int) override { ++failures; }
    void onCadOffsetChanged(int8_t offset) override { offsets.push_back(offset); }
};
struct Fixture {
    Clock clock;
    Radio radio;
    StaticPoolPacketManager pool;
    Dispatcher dispatcher{radio, clock, pool};
    Fixture() {
        auto p = pool.allocNew(); CHECK(p != nullptr); pool.free(p);
    }
    void begin(uint8_t duty = 0) { dispatcher.duty = duty; dispatcher.begin(); }
    mesh::Packet* packet(uint8_t type = PAYLOAD_TYPE_TXT_MSG, uint8_t route = ROUTE_TYPE_FLOOD) {
        auto p = dispatcher.obtainNewPacket(); CHECK(p != nullptr);
        p->header = (type << 2) | route;
        p->transport_codes[0] = p->transport_codes[1] = 0;
        p->payload_len = 1; p->payload[0] = 0xAA; return p;
    }
    void complete() { radio.completion = true; dispatcher.loop(); }
};
}
