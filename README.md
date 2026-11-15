# esphome-fineoffset
An external component for ESPHome that receives weather information from Fineoffset WH2 sensors.

## Quick Start

### Configuration Example

```yaml
esphome:
  name: fineoffset-example
  
esp32:
  board: esp32dev
  
external_components:
  - source: components

fineoffset:
  - name: FineOffset Radio
    id: fineoffset_radio
    pin: GPIO33

sensor:
  - platform: fineoffset
    fineoffset_id: fineoffset_radio
    name: "Outdoor Sensor"
    sensor_number: 77  # Your WH2 sensor number
    temperature:
      name: "Outside Temperature"
    humidity:
      name: "Outside Humidity"

text_sensor:
  - platform: fineoffset
    name: "Last Sensor Data"
    type: last
```

## Battery Swaps & Changing Sensor Numbers

WH2 sensors pick a **new random sensor number every time they power up**, so a battery swap breaks a hardcoded
`sensor_number`. The sensor platform can follow the new number automatically:

```yaml
sensor:
  - platform: fineoffset
    sensor_number: 77               # configured sensor number (default; overridable from HA)
    timeout: 30min                  # silence before `online` goes off / following is allowed (default 30min)
    follow_new_sensor_number: true  # default false
    configured_sensor_number: { name: Deck Configured Sensor No. }  # number: configured sensor number, editable
    active_sensor_number:     { name: Deck Active Sensor No. }      # text: sensor number readings come from
    online:                   { name: Deck Sensor Online }          # active sensor number heard within `timeout`
    sensor_number_update:     { name: Deck Sensor No. }             # Up-to-date / Update available
    apply_new_sensor_number:  { name: Deck Apply New Sensor No. }   # make the active number the configured one
    temperature:              { name: Deck Temperature }
    humidity:                 { name: Deck Humidity }
```

Rules:

- **Follow:** if the active sensor number is silent for `timeout`, and a sensor number no other sensor config claims has been
  heard at least twice within the last `timeout`, readings switch to it and `sensor_number_update` turns on
  ("Update available").
- **Silence alone never changes anything** beyond `online` turning off.
- **Snap-back:** if the configured sensor number is heard again, the sensor reverts and `sensor_number_update` clears. Wrongly
  following a neighbor's sensor during an outage self-corrects.
- **Apply:** pressing `apply_new_sensor_number` (or setting `configured_sensor_number`) makes it permanent and clears
  `sensor_number_update`.
- Configured and active sensor numbers **persist across reboots**. Changing `sensor_number` in YAML and reflashing discards them.

WH2 packets carry **no battery status**, so `online` is the dead-battery signal. See
[`docs/home-assistant-package.yaml`](docs/home-assistant-package.yaml) for repeating HA alerts, a one-tap "Apply"
notification action, and a dashboard card that only shows the apply button when needed.

## Development & Testing

### Using Docker (Recommended)

No local Python or ESPHome installation required. Use Docker with this convenient alias:

```bash
# Add to ~/.bashrc or ~/.zshrc
alias esphome='docker run -it --rm \
  -v "$PWD:/config" \
  -v "$HOME/.cache/esphome/cache:/cache" \
  -v "$HOME/.cache/esphome/build:/build" \
  -v /etc/localtime:/etc/localtime:ro \
  esphome/esphome:stable'

# Test the example configuration
esphome compile example.yaml

# Validate configuration
esphome config example.yaml
```

### Alternative: Shell Script

Create `scripts/esphome` for a more portable approach:

```bash
#!/bin/bash
docker run -it --rm \
  -v "$PWD:/config" \
  -v "$HOME/.cache/esphome/cache:/cache" \
  -v "$HOME/.cache/esphome/build:/build" \
  -v /etc/localtime:/etc/localtime:ro \
  esphome/esphome:stable "$@"
```

Then use: `./scripts/esphome compile example.yaml`

### Host unit tests

The follow/override logic has host-side tests that link the real component against ESPHome's host platform with a
fake clock (no ESP toolchain needed):

```bash
test/host/run.sh
```

## Hardware Requirements

- ESP32 (recommended) or ESP8266
- 433MHz receiver module (e.g., SYN470R, RXB6)
- FineOffset WH2 compatible weather sensor

## Sensor Compatibility

This component is designed for FineOffset WH2 protocol sensors, including:
- WH2 temperature/humidity sensors
- Compatible third-party sensors using the same protocol

Each sensor transmits on 433MHz using OOK modulation with a unique sensor ID.
