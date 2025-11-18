# Runtime Stability Testing Roadmap

## Background
The refactor-pull-based-architecture branch addresses runtime instability caused by:
- Heap fragmentation from `std::shared_ptr` allocations in interrupt context (`IRAM_ATTR`)
- Race conditions with dynamic containers (`std::map`, `std::deque`) 
- Memory leaks from uncontrolled dynamic allocation

The current refactoring eliminates these issues via fixed-size arrays and value semantics.

## Existing Test Coverage (✅ Implemented)
- **Memory stress**: Multiple sensors, boundary sensor IDs (0, 255) - `test/test-memory-stress.yaml`
- **Data freshness**: Different update intervals, selective clearing - `test/test-data-freshness.yaml` 
- **Edge cases**: High-frequency updates (1-2s intervals) - `test/test-edge-cases.yaml`

## Additional Stability Tests Needed

### 1. Interrupt Storm Test
**Purpose**: Verify no stack overflow or watchdog triggers under continuous GPIO interrupts

**Implementation**: 
```yaml
# test-interrupt-storm.yaml
sensor:
  - platform: template
    name: "Interrupt Rate"
    lambda: |-
      static uint32_t last_count = 0;
      uint32_t current = id(fineoffset_radio).store_.cycles_;
      uint32_t rate = current - last_count;
      last_count = current;
      return rate;
    update_interval: 1s
```

**Hardware**: Connect GPIO33 to PWM generator (1kHz-10kHz square wave)
**Software**: Add debug method to artificially trigger interrupts

### 2. Memory Exhaustion Test  
**Purpose**: Ensure no additional heap allocations under memory pressure

**Implementation**:
```yaml
# test-memory-exhaustion.yaml
esphome:
  platformio_options:
    build_flags: 
      - -DBOARD_HAS_PSRAM=0  # Disable PSRAM

sensor:
  - platform: template
    name: "Free Heap"
    lambda: "return ESP.getFreeHeap();"
    update_interval: 5s
    
  # Add 50+ template sensors to consume memory
  - platform: template
    name: "Memory Consumer 1"
    lambda: |-
      static std::vector<uint8_t> buffer(1000); // Consume 1KB
      return buffer.size();
```

### 3. Watchdog Stress Test
**Purpose**: 24-72 hour stability verification

**Implementation**:
```yaml
# test-watchdog-stress.yaml
substitutions:
  test_duration: "24h"  # or 48h, 72h

sensor:
  - platform: uptime
    name: "Uptime Hours"
    
  - platform: template  
    name: "Packet Error Rate"
    lambda: |-
      auto total = id(fineoffset_radio).store_.cycles_;
      auto bad = id(fineoffset_radio).store_.bad_count_;
      return total > 0 ? (float(bad) / total) * 100.0 : 0.0;
    unit_of_measurement: "%"
    update_interval: 60s

text_sensor:
  - platform: fineoffset
    update_interval: 1s  # Very aggressive
```

**Methodology**: Deploy to physical hardware, monitor via Home Assistant/logs

### 4. Real-time Timing Verification
**Purpose**: Verify interrupt timing doesn't drift under load (critical for 433MHz decoding)

**Implementation**:
```cpp
// Add to fineoffset.cpp - timing diagnostics
#ifdef ESPHOME_BUILD_FLAG_DEBUG
static volatile uint32_t max_interrupt_duration_us = 0;
static volatile uint32_t interrupt_count_per_second = 0;

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore* self) {
    uint32_t start_time = micros();
    // ... existing interrupt code ...
    uint32_t duration = micros() - start_time;
    if (duration > max_interrupt_duration_us) {
        max_interrupt_duration_us = duration;
    }
    interrupt_count_per_second++;
}
#endif
```

**Signal generator**: Use actual WH2 sensor or 433MHz OOK signal generator

### 5. Additional Test Scenarios
- **Concurrent Access**: Multiple sensors updating during cleanup cycles
- **Power Cycle Simulation**: Rapid component disable/enable cycles  
- **Invalid Packet Flood**: High rate of CRC-failed packets
- **Sensor ID Wraparound**: Test behavior with IDs at array boundaries (255→0)

## Testing Implementation Shortcuts

### Test Hooks (for unit testing without hardware)
```cpp
// Add to fineoffset.h
#ifdef ESPHOME_BUILD_FLAG_TEST
public:
    void inject_test_packet(const uint8_t* packet);  // Bypass GPIO
    void force_cleanup_cycle();                      // Test cleanup logic
    uint32_t get_consumed_sensor_count();           // Verify cleanup
#endif
```

### Docker-based Load Testing
```bash
# Run multiple test configurations simultaneously
for config in test-*.yaml; do
    docker run --rm -d --name "test-$(basename $config .yaml)" \
        -v "$PWD:/config" esphome/esphome:stable run "$config" &
done
```

## Priority Order
1. **Memory exhaustion** (software-only, immediate)
2. **Interrupt storm** (software simulation first, then hardware)
3. **Watchdog stress** (long-running validation)
4. **Real-time timing** (requires hardware setup)

## Success Criteria
- No heap allocation growth over time
- No watchdog resets under load
- Interrupt processing remains under 50μs
- 24+ hour runtime without memory leaks
- Packet error rate stays consistent under stress

## Notes
The current refactoring should eliminate the primary instability sources by removing heap allocation from interrupt context. These tests will validate the theoretical improvements in practice.