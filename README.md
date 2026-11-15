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
    sensor_no: 77  # Your WH2 sensor ID
    temperature:
      name: "Outside Temperature"
    humidity:
      name: "Outside Humidity"

text_sensor:
  - platform: fineoffset
    name: "Last Sensor Data"
    type: last
```

## Battery Swaps & Sensor ID Drift

WH2 sensors pick a **new random ID every time they power up**, so a battery swap breaks a hardcoded
`sensor_no`. The sensor platform can follow the new ID automatically:

```yaml
sensor:
  - platform: fineoffset
    sensor_no: 77     # configured ID (default; overridable from HA)
    timeout: 30min    # silence before `online` goes off and drift is allowed (default 30min)
    drift: true       # default false
    configured_sensor_no: { name: Deck Configured Sensor ID }  # configured ID, editable from HA
    active_sensor_no:     { name: Deck Active Sensor ID }      # ID readings currently come from
    online:               { name: Deck Sensor Online }         # active ID heard within `timeout`
    drifted:              { name: Deck Sensor Drifted }        # active != configured
    accept_drift:         { name: Deck Accept Drift }          # make the active ID the configured one
    temperature:          { name: Deck Temperature }
    humidity:             { name: Deck Humidity }
```

Rules:

- **Drift:** if the active ID is silent for `timeout`, and an ID no other sensor config claims has been heard at
  least twice within the last `timeout`, readings switch to that ID and `drifted` turns on.
- **Silence alone never changes anything** beyond `online` turning off.
- **Snap-back:** if the configured ID is heard again while drifted, the sensor reverts and `drifted` clears. A wrong
  drift (e.g. onto a neighbor's sensor during an outage) self-corrects.
- **Accept:** pressing `accept_drift` (or setting `configured_sensor_no`) makes it permanent and clears `drifted`.
- Configured and active IDs **persist across reboots**. Changing `sensor_no` in YAML and reflashing discards them.

WH2 packets carry **no battery status**, so `online` is the dead-battery signal. See
[`docs/home-assistant-package.yaml`](docs/home-assistant-package.yaml) for repeating HA alerts on drift/offline,
with a one-tap "Accept" notification action.

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

The drift/override logic has host-side tests that link the real component against ESPHome's host platform with a
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
