// Regression test for "Mute encrypted" (GitHub issue #1): the per-timeslot
// encryption state of TetraSubscriberTable, which the listen loop uses to
// decide which traffic timeslots may be played. The events are fed in the
// order the decoder delivers them: the MAC-RESOURCE address callback (with the
// channel allocation) first, then the layer 3 PDU of the same MAC-RESOURCE.
//
// Build and run (from the repository root; test_tetra_schwartz.py does this):
//   D=decoder_modules/ch_tetra_demodulator/src
//   g++ -std=c++17 -I$D -I$D/decoder/src e2e/gen/test_listen_mute.cpp -o test_listen_mute
//   ./test_listen_mute

#include <cstdio>
#include <cstring>
#include "subscribers.h"

static int failures = 0;
#define CHECK(cond, what)                                                   \
    do {                                                                    \
        if (!(cond)) { printf("FAIL: %s (line %d)\n", what, __LINE__); failures++; } \
        else { printf("ok:   %s\n", what); }                               \
    } while (0)

using T = TetraSubscriberTable;
static const uint32_t GSSI = 91, TALKER_A = 10000001, TALKER_B = 10000004;

// MAC-RESOURCE to 'ssi' (address type SSI); allocTs 0 = no channel allocation
static void mac(T& t, uint32_t ssi, int allocTs, uint8_t encrMode = 0) {
    uint8_t ca = allocTs ? (uint8_t)(0x10 | (8 >> (allocTs - 1))) : 0;
    t.add(1, ssi, 0, 1, encrMode, ca);
}

static void l3(T& t, int type, int callId, uint32_t addr, uint32_t party, int encrypted = -1) {
    tetra_l3_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.tn = 1;
    ev.addr_ssi = addr;
    ev.addr_type = 1;
    ev.call_id = callId;
    ev.comm_type = type == TETRA_L3_CALL_SETUP ? 1 : -1;
    ev.encrypted_call = encrypted;
    ev.party_ssi = party;
    ev.status = -1;
    ev.sds_pid = -1;
    ev.disconnect_cause = type == TETRA_L3_CALL_RELEASE ? 1 : -1;
    ev.mm_type = -1;
    snprintf(ev.pdu, sizeof(ev.pdu), "test");
    t.addL3(ev);
}

int main() {
    const time_t t0 = 1800000000;

    // 1. encrypted group call on TS2 lasting more than 60 s, several
    //    D-TX-GRANTED without a new channel allocation
    {
        T t;
        t.setTestClock(t0);
        mac(t, GSSI, 2);
        l3(t, TETRA_L3_CALL_SETUP, 101, GSSI, TALKER_A, 1);
        CHECK(t.listenEncState(2) == T::LISTEN_ENCRYPTED, "E2EE call: encrypted after D-SETUP");
        for (int k = 1; k <= 6; k++) {
            t.setTestClock(t0 + 25 * k);
            mac(t, GSSI, 0);
            l3(t, TETRA_L3_TX_CEASED, 101, GSSI, 0);
            mac(t, GSSI, 0);
            l3(t, TETRA_L3_TX_GRANTED, 101, GSSI, (k & 1) ? TALKER_B : TALKER_A);
        }
        CHECK(t.slotOwner(2).ssi == 0, "allocation-based owner is gone after 150 s (the old fail-open path)");
        CHECK(t.listenEncState(2) == T::LISTEN_ENCRYPTED, "E2EE call: still encrypted after 150 s and 6 TX-GRANTED");
        CHECK(t.listenEncState(1) == T::LISTEN_UNKNOWN && t.listenEncState(3) == T::LISTEN_UNKNOWN, "other timeslots: unknown");
        mac(t, GSSI, 0);
        l3(t, TETRA_L3_CALL_RELEASE, 101, GSSI, 0);
        CHECK(t.listenEncState(2) == T::LISTEN_UNKNOWN, "after D-RELEASE: unknown again");
    }

    // 2. call already running when decoding starts (no D-SETUP seen)
    {
        T t;
        t.setTestClock(t0);
        mac(t, GSSI, 0);
        l3(t, TETRA_L3_TX_GRANTED, 202, GSSI, TALKER_A);
        CHECK(t.listenEncState(3) == T::LISTEN_UNKNOWN, "running call, no D-SETUP: unknown (muted)");
        // D-TX-GRANTED with a channel allocation tells the timeslot, not the encryption
        mac(t, GSSI, 3);
        l3(t, TETRA_L3_TX_GRANTED, 202, GSSI, TALKER_B);
        CHECK(t.slotCallId(3) == 202, "TX-GRANTED with allocation: call 202 on TS3");
        CHECK(t.listenEncState(3) == T::LISTEN_UNKNOWN, "TX-GRANTED carries no service information: still unknown");
        // late entry: the network repeats D-SETUP for the running call
        mac(t, GSSI, 3);
        l3(t, TETRA_L3_CALL_SETUP, 202, GSSI, TALKER_B, 0);
        CHECK(t.listenEncState(3) == T::LISTEN_CLEAR, "late-entry D-SETUP (clear): clear");
    }

    // 3. clear call stays clear over 60 s; a new encrypted call on the same timeslot
    {
        T t;
        t.setTestClock(t0);
        mac(t, GSSI, 2);
        l3(t, TETRA_L3_CALL_SETUP, 303, GSSI, TALKER_A, 0);
        t.setTestClock(t0 + 90);
        mac(t, GSSI, 0);
        l3(t, TETRA_L3_TX_GRANTED, 303, GSSI, TALKER_B);
        CHECK(t.listenEncState(2) == T::LISTEN_CLEAR, "clear call: clear after 90 s");
        mac(t, 9, 2);
        l3(t, TETRA_L3_CALL_SETUP, 304, 9, TALKER_A, 1);
        CHECK(t.slotCallId(2) == 304 && t.listenEncState(2) == T::LISTEN_ENCRYPTED, "new E2EE call takes TS2: encrypted");
        // the old call's TX-GRANTED without allocation does not take the timeslot back
        mac(t, GSSI, 0);
        l3(t, TETRA_L3_TX_GRANTED, 303, GSSI, TALKER_A);
        CHECK(t.slotCallId(2) == 304, "stale call without allocation does not take TS2 back");
    }

    // 4. air interface encryption on the address of a running clear call
    {
        T t;
        t.setTestClock(t0);
        mac(t, GSSI, 4);
        l3(t, TETRA_L3_CALL_SETUP, 405, GSSI, TALKER_A, 0);
        CHECK(t.listenEncState(4) == T::LISTEN_CLEAR, "clear call on TS4");
        mac(t, GSSI, 0, 1);
        CHECK(t.listenEncState(4) == T::LISTEN_ENCRYPTED, "MAC-RESOURCE encryption mode != 0 on the call address: encrypted");
    }

    // 5. "Clear all" in the stations table resets the call state
    {
        T t;
        t.setTestClock(t0);
        mac(t, GSSI, 2);
        l3(t, TETRA_L3_CALL_SETUP, 505, GSSI, TALKER_A, 0);
        t.clear();
        CHECK(t.listenEncState(2) == T::LISTEN_UNKNOWN && t.slotCallId(2) == -1, "clear(): unknown");
    }

    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
