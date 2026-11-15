// Host test harness for FineOffsetSensor drift logic. Drives the real store/sensor code with a fake clock.
#include <cassert>
#include <cstdio>
#include <optional>
#include <string>
#include <map>
#include <vector>
#include <atomic>
#include <bitset>
#include <functional>
#define protected public
#define private public
#include "esphome/components/fineoffset/fineoffset.h"
#include "esphome/components/fineoffset/sensor/fineoffset_sensor.h"
#include "esphome/components/host/preferences.h"
#undef protected
#undef private

namespace esphome {
uint32_t g_fake_ms = 0;
}
using namespace esphome;
using namespace esphome::fineoffset;

static int failures = 0;
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while (0)

// Feed one valid packet from `id` through the real record_state() path.
static void rx(FineOffsetComponent& hub, uint8_t id, int16_t temp_dC = 50, uint8_t hum = 60) {
    byte p[5];
    uint16_t t = temp_dC < 0 ? (uint16_t) (-temp_dC) | 0x800 : (uint16_t) temp_dC;
    p[0] = 0x40 | (id >> 4);
    p[1] = (uint8_t) ((id & 0x0F) << 4) | (uint8_t) ((t >> 8) & 0x0F);
    p[2] = t & 0xFF;
    p[3] = hum;
    p[4] = FineOffsetState::crc8ish(p, 4);
    FineOffsetState s(p);
    assert(s.valid && s.plausible && s.sensor_id == id);
    auto& st = hub.store_;
    st.state_buffers_[st.isr_buffer_index_.load()] = s;
    st.has_pending_state_.store(true);
    st.record_state();
}
// One WH2 transmission = two identical packets.
static void tx(FineOffsetComponent& hub, uint8_t id) {
    rx(hub, id);
    rx(hub, id);
}
static void advance(uint32_t ms) { g_fake_ms += ms; }
static const uint32_t MIN = 60 * 1000;

struct Rig {
    FineOffsetComponent hub;
    FineOffsetSensor s;
    FineOffsetConfiguredSensorNoNumber num;
    sensor::Sensor active;
    binary_sensor::BinarySensor online, drifted;
    FineOffsetAcceptDriftButton accept;
    Rig(uint8_t id, bool drift = true) {
        s.set_parent(&hub);
        s.set_sensor_no(id);
        s.set_timeout(30 * MIN);
        s.set_drift(drift);
        num.set_parent(&s);
        accept.set_parent(&s);
        s.set_configured_sensor_no_number(&num);
        s.set_active_sensor_no_sensor(&active);
        s.set_online_binary_sensor(&online);
        s.set_drifted_binary_sensor(&drifted);
        s.setup();
    }
};

int main() {
    setenv("ESPHOME_PREFDIR", "/tmp/fineoffset-host-test-prefs", 1);
    host::setup_preferences();

    printf("1. normal reception, no drift\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        for (int i = 0; i < 60; i++) {
            tx(r.hub, 77);
            advance(MIN);
            r.s.update();
        }
        CHECK(r.s.active_ == 77 && !r.s.is_drifted() && r.online.state);
    }

    printf("2. battery swap: silent 30min + new ID -> drift, flag on, data from new ID\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        r.s.update();
        CHECK(r.online.state);
        for (int i = 0; i < 35; i++) {
            advance(MIN);
            if (i >= 2)
                tx(r.hub, 142);
            r.s.update();
        }
        CHECK(r.s.active_ == 142);
        CHECK(r.s.configured_ == 77);
        CHECK(r.drifted.state);
        CHECK(r.active.state == 142);
        CHECK(r.num.state == 77);
        CHECK(r.online.state);
        CHECK(!r.hub.is_unknown_sensor(142));
        CHECK(!r.hub.is_unknown_sensor(77));
    }

    printf("3. no drift before timeout\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        for (int i = 0; i < 25; i++) {
            advance(MIN);
            tx(r.hub, 142);
            r.s.update();
        }
        CHECK(r.s.active_ == 77 && !r.s.is_drifted());
    }

    printf("4. single noise packet does not trigger drift\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        advance(31 * MIN);
        rx(r.hub, 200);
        r.s.update();
        CHECK(r.s.active_ == 77);
        CHECK(!r.online.state);
    }

    printf("5. silence alone never unlatches\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        for (int i = 0; i < 120; i++) {
            advance(MIN);
            r.s.update();
        }
        CHECK(r.s.active_ == 77 && !r.s.is_drifted() && !r.online.state);
    }

    printf("6. stale unknown (heard before silence began) is not adopted\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 142);
        advance(MIN);
        tx(r.hub, 77);  // neighbor heard, then ours
        advance(31 * MIN);
        r.s.update();
        CHECK(r.s.active_ == 77);
    }

    printf("7. wrong drift self-corrects when configured ID returns\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        advance(31 * MIN);
        tx(r.hub, 142);
        r.s.update();
        CHECK(r.s.active_ == 142 && r.drifted.state);
        advance(MIN);
        tx(r.hub, 77);
        r.s.update();
        CHECK(r.s.active_ == 77 && !r.drifted.state);
        CHECK(r.hub.is_unknown_sensor(142));
    }

    printf("8. accept drift promotes active to configured, persisted\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        {
            Rig r(77);
            tx(r.hub, 77);
            advance(31 * MIN);
            tx(r.hub, 142);
            r.s.update();
            r.accept.press();
            CHECK(r.s.configured_ == 142 && r.s.active_ == 142 && !r.drifted.state && r.num.state == 142);
            CHECK(r.hub.is_unknown_sensor(77));
        }
        {
            Rig r2(77);  // "reboot"
            CHECK(r2.s.configured_ == 142 && r2.s.active_ == 142);
        }
    }

    printf("9. reboot while drifted keeps drift; configured returning after reboot snaps back\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        {
            Rig r(77);
            tx(r.hub, 77);
            advance(31 * MIN);
            tx(r.hub, 142);
            r.s.update();
            CHECK(r.s.is_drifted());
        }
        g_fake_ms = 500;  // millis restarts after reboot
        {
            Rig r2(77);
            CHECK(r2.s.active_ == 142 && r2.s.configured_ == 77 && r2.drifted.state);
            advance(MIN);
            tx(r2.hub, 142);
            r2.s.update();
            CHECK(r2.s.active_ == 142);
            advance(MIN);
            tx(r2.hub, 77);
            r2.s.update();
            CHECK(r2.s.active_ == 77 && !r2.drifted.state);
        }
    }

    printf("10. number override sets both IDs and clears drift; persisted\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        {
            Rig r(77);
            tx(r.hub, 77);
            advance(31 * MIN);
            tx(r.hub, 142);
            r.s.update();
            r.num.make_call().set_value(99).perform();
            CHECK(r.s.configured_ == 99 && r.s.active_ == 99 && !r.drifted.state);
        }
        {
            Rig r2(77);
            CHECK(r2.s.configured_ == 99 && r2.s.active_ == 99);
        }
    }

    printf("11. YAML sensor_no change discards stored override\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        {
            Rig r(77);
            r.num.make_call().set_value(99).perform();
        }
        {
            Rig r2(55);
            CHECK(r2.s.configured_ == 55 && r2.s.active_ == 55);
        }
    }

    printf("12. drift disabled: never drifts; stored drift ignored\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        {
            Rig r(77);
            tx(r.hub, 77);
            advance(31 * MIN);
            tx(r.hub, 142);
            r.s.update();
            CHECK(r.s.is_drifted());
        }
        {
            Rig r2(77, false);
            CHECK(r2.s.active_ == 77 && !r2.s.is_drifted());
            advance(31 * MIN);
            tx(r2.hub, 143);
            r2.s.update();
            CHECK(r2.s.active_ == 77);
        }
    }

    printf("13. never heard since boot: drift only after uptime >= timeout\n");
    {
        global_preferences->reset();
        g_fake_ms = 0;
        Rig r(77);
        advance(5 * MIN);
        tx(r.hub, 142);
        r.s.update();
        CHECK(r.s.active_ == 77);
        advance(26 * MIN);
        tx(r.hub, 142);
        r.s.update();
        CHECK(r.s.active_ == 142);
    }

    printf("14. drifted sensor dies too -> drifts again to the next one\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        tx(r.hub, 77);
        advance(31 * MIN);
        tx(r.hub, 142);
        r.s.update();
        CHECK(r.s.active_ == 142);
        advance(31 * MIN);
        tx(r.hub, 150);
        r.s.update();
        CHECK(r.s.active_ == 150 && r.s.configured_ == 77);
        CHECK(r.hub.is_unknown_sensor(142));
    }

    printf("15. millis() wraparound\n");
    {
        global_preferences->reset();
        g_fake_ms = 0xFFFFFFFFu - 10 * MIN;
        Rig r(77);
        tx(r.hub, 77);
        advance(20 * MIN);
        r.s.update();
        CHECK(r.s.active_ == 77 && r.online.state);
        advance(15 * MIN);
        tx(r.hub, 142);
        r.s.update();
        CHECK(r.s.active_ == 142);
    }

    printf("16. accept when not drifted is a no-op\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        Rig r(77);
        r.accept.press();
        CHECK(r.s.configured_ == 77 && r.s.active_ == 77);
    }

    printf("17. two sensor configs don't steal each other's IDs\n");
    {
        global_preferences->reset();
        g_fake_ms = 1000;
        FineOffsetComponent hub;
        FineOffsetSensor a, b;
        a.set_parent(&hub);
        a.set_sensor_no(77);
        a.set_timeout(30 * MIN);
        a.set_drift(true);
        a.setup();
        b.set_parent(&hub);
        b.set_sensor_no(23);
        b.set_timeout(30 * MIN);
        b.set_drift(true);
        b.setup();
        tx(hub, 77);
        advance(31 * MIN);
        tx(hub, 23);
        a.update();
        b.update();
        CHECK(a.active_ == 77);
        CHECK(b.active_ == 23);
    }

    printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
