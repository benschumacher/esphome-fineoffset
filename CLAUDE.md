# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Code Quality
- **Linting**: Use `pre-commit run --all-files` to run all linting checks
- **C++ Formatting**: C++ code follows clang-format rules in `.clang-format` (120 char limit, 4-space indent)
- **Python Formatting**: Python code uses black, ruff, flake8, and pylint via pre-commit hooks
- **Environment Setup**: Use `script/run-in-env.sh` to run commands in the appropriate virtual environment

### Testing
- **Example Configuration**: Test changes using `example.yaml` which demonstrates the component usage
- **ESPHome Compilation**: Validate with `esphome compile example.yaml`

#### Docker-based Testing (Recommended)
To avoid requiring local Python/ESPHome installation, use Docker:

```bash
# Test compilation
docker run --rm \
  -v "$PWD:/config" \
  -v "$HOME/.cache/esphome/cache:/cache" \
  -v "$HOME/.cache/esphome/build:/build" \
  -v /etc/localtime:/etc/localtime:ro \
  esphome/esphome:stable compile example.yaml

# Check configuration
docker run --rm \
  -v "$PWD:/config" \
  -v "$HOME/.cache/esphome/cache:/cache" \
  -v "$HOME/.cache/esphome/build:/build" \
  -v /etc/localtime:/etc/localtime:ro \
  esphome/esphome:stable config example.yaml
```

## Architecture

### Component Structure
This is an ESPHome external component for receiving weather data from FineOffset WH2 sensors via 433MHz radio. The architecture follows ESPHome's component pattern:

**Core Component (`fineoffset/`)**:
- `FineOffsetComponent`: Main polling component that manages radio reception
- `FineOffsetStore`: Handles interrupt-driven radio signal processing and packet decoding
- `FineOffsetState`: Data structure for sensor readings (temperature, humidity, sensor ID)

**Sensor Support (`sensor/`)**:
- `FineOffsetSensor`: Temperature and humidity sensor entities
- Supports multiple WH2 sensors identified by unique sensor numbers

**Text Sensor Support (`text_sensor/`)**:
- `FineOffsetTextSensor`: Diagnostic text sensors showing last received data
- Types: `last`, `last_bad` (CRC failed), `unknown` (unregistered sensors)

### Radio Signal Processing
The component uses GPIO interrupt handling to decode 433MHz OOK modulated signals:
- Interrupt-driven edge detection for timing analysis
- Manchester-style encoding with specific pulse timing requirements
- CRC8 validation for packet integrity
- Supports multiple concurrent sensors with unique IDs

### Configuration Pattern
Uses ESPHome's standard configuration schema:
```yaml
fineoffset:
  - pin: GPIO33
    id: fineoffset_radio

sensor:
  - platform: fineoffset
    fineoffset_id: fineoffset_radio
    sensor_no: 77  # Unique sensor identifier
    temperature:
      name: "Temperature"
    humidity:
      name: "Humidity"
```

### Key Implementation Details
- Thread-safe atomic operations for interrupt/main thread communication
- State management with automatic cleanup and history tracking
- Support for unknown sensors (for discovery/debugging)
- CRC validation using modified CRC-8 algorithm specific to WH2 protocol
