#!/usr/bin/env bash
# Host-side unit tests for FineOffsetSensor drift/override logic.
#
# Links the real component sources against ESPHome's host platform (fake millis()) without
# PlatformIO or an ESP toolchain. Best-effort: relies on ESPHome internals and may need tweaks
# after ESPHome upgrades.
#
# Usage: test/host/run.sh   (needs `esphome` on PATH and g++ with C++20)
set -euo pipefail

repo=$(cd "$(dirname "$0")/../.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# 1. Let ESPHome generate a source tree (core + entity components) from the example config.
(cd "$repo" && esphome compile --only-generate example.yaml >/dev/null)
cp -r "$repo/.esphome/build/fineoffset-example/src" "$work/src"
src=$work/src
rm -rf "$src/main.cpp" "$src/esphome/components/esp32" "$src/esphome/components/fineoffset"
cp -r "$repo/components/fineoffset" "$src/esphome/components/"
find "$src/esphome/components/fineoffset" -name '*.py' -delete

# 2. Swap the ESP32 platform for the host one, with a test-controlled clock.
esphome_dir=$(python3 -c 'import esphome, os; print(os.path.dirname(esphome.__file__))')
mkdir -p "$src/esphome/components/host"
cp "$esphome_dir"/components/host/*.{h,cpp} "$src/esphome/components/host/"
sed -i 's|^uint32_t IRAM_ATTR HOT millis() {|extern uint32_t g_fake_ms;\nuint32_t IRAM_ATTR HOT millis() { return g_fake_ms; }\nuint32_t real_millis_unused() {|' \
  "$src/esphome/components/host/hal.cpp"
sed -i -E 's/^#define USE_(ESP32|ESP_IDF|ARDUINO|ESP32_VARIANT).*$//' "$src/esphome/core/defines.h"
printf '#define USE_HOST\n#define USE_ESPHOME_HOST_MAC_ADDRESS {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}\n' >>"$src/esphome/core/defines.h"
mkdir -p "$work/stub" && printf '#pragma once\n#include <cstdint>\n' >"$work/stub/esp_types.h"

# 3. Build and run.
mapfile -t srcs < <(find "$src/esphome/core" "$src"/esphome/components/{sensor,binary_sensor,number,button,host,fineoffset} \
  -name '*.cpp' ! -path '*text_sensor*' ! -path '*/host/core.cpp')
g++ -std=gnu++20 -O0 -g -DUSE_HOST -DIRAM_ATTR= -I"$src" -I"$work/stub" \
  "${srcs[@]}" "$repo/test/host/stubs.cpp" "$repo/test/host/test_drift.cpp" -o "$work/test_drift"
"$work/test_drift" | grep -v '^\['
