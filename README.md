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

## Hardware Requirements

- ESP32 (recommended) or ESP8266
- 433MHz receiver module (e.g., SYN470R, RXB6)
- FineOffset WH2 compatible weather sensor

## Sensor Compatibility

This component is designed for FineOffset WH2 protocol sensors, including:
- WH2 temperature/humidity sensors
- Compatible third-party sensors using the same protocol

Each sensor transmits on 433MHz using OOK modulation with a unique sensor ID.
