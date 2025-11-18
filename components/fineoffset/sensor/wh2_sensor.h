#pragma once
#if 0

volatile byte have_data_string = 0;
volatile int have_data_sensor = 0;
volatile int temp;
volatile int hum;
volatile unsigned int cycles = 0;
volatile int str_Sensor_ID;
volatile int str_humidity;
volatile int str_temperature;
volatile unsigned int str_cycles = 0;
volatile bool str_wh2_valid;

static void ICACHE_RAM_ATTR ext_int_1() {
    static unsigned long edgeTimeStamp[3] = { 0, }; // Timestamp of edges
    static bool skip = true;

    // Filter out too short pulses. This method works as a low pass filter.  (borroved from new remote reciever)
    edgeTimeStamp[1] = edgeTimeStamp[2];
    edgeTimeStamp[2] = micros();

    if (skip) {
        skip = false;
        return;
    }

    if (edgeTimeStamp[2] - edgeTimeStamp[1] < 200) {
        // Last edge was too short.
        // Skip this edge, and the next too.
        skip = true;
        return;
    }

    unsigned int pulse = edgeTimeStamp[1] - edgeTimeStamp[0];
    edgeTimeStamp[0] = edgeTimeStamp[1];

    //--------------------------------------------------------
    // 1 is indicated by 500uS pulse
    // wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval) (interval >= 250 && interval <= 750)
    // 0 is indicated by ~1500us pulse
    // wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval) (interval >= 1200 && interval <= 1750)
    // worst case packet length
    // 6 bytes x 8 bits =48
#define IDLE_HAS_TIMED_OUT(interval) (interval > 1199)
    // our expected pulse should arrive after 1ms
    // we'll wh2_accept it if it arrives after
    // 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 751)
#define GOT_PULSE 0x01
#define LOGIC_HI 0x02

    static byte wh2_flags = 0x00;
    static bool wh2_accept_flag = false;
    static byte wh2_packet_state = 0;
    static byte wh2_packet[5] = {0,};
    static bool wh2_valid = false;
    static byte sampling_state = 0;
    static byte packet_no = 0;
    static byte bit_no = 0;
    static byte history = 0x01;

    switch (sampling_state) {
    case 0: // waiting

        if (IS_HI_PULSE(pulse)) {
            wh2_flags = GOT_PULSE | LOGIC_HI;
            sampling_state = 1;
        } 
        else if (IS_LOW_PULSE(pulse)) {
            wh2_flags = GOT_PULSE; // logic low
        }
        else {
            sampling_state = 0;
        }
        break;
    case 1: // observe 1ms of idle time

        if (IDLE_HAS_TIMED_OUT(pulse)) {
            sampling_state = 0;
            wh2_packet_state = 1;
        }
        else if (IDLE_PERIOD_DONE(pulse)) {
            sampling_state = 0;
        }
        else {
            sampling_state = 0;
            wh2_packet_state = 1;
        }
        break;
    }

    if (wh2_flags) {

        // acquire preamble
        if (wh2_packet_state == 1) {
            // shift history right and store new value
            history <<= 1;
            // store a 1 if required (right shift along will store a 0)
            if (wh2_flags & LOGIC_HI) {
                history |= 0x01;
            }

            // check if we have a valid start of frame
            // xxxxx110
            if ((history & B00000111) == B00000110) {
                // need to clear packet, and pulseers
                packet_no = 0;
                // start at 1 becuase only need to acquire 7 bits for first packet byte.
                bit_no = 1;
                wh2_packet[0] = wh2_packet[1] = wh2_packet[2] = wh2_packet[3] = wh2_packet[4] = 0;
                // we've acquired the preamble
                wh2_packet_state = 2;
                history = 0xFF;
            }
            wh2_accept_flag = false;
        }
        // acquire packet
        else if (wh2_packet_state == 2) {
            wh2_packet[packet_no] <<= 1;
            if (wh2_flags & LOGIC_HI) {
                wh2_packet[packet_no] |= 0x01;
            }
            bit_no++;
            if (bit_no > 7) {
                bit_no = 0;
                packet_no++;
            }
            if (packet_no > 4) {
                // start the sampling process from scratch
                wh2_packet_state = 1;
                wh2_accept_flag = true;
            }
        }
        else {
            wh2_accept_flag = false;
        }

        if (wh2_accept_flag) {
            int Sensor_ID = ((wh2_packet[0]&0x0F) << 4) + ((wh2_packet[1]&0xF0) >> 4);
            int humidity = wh2_packet[3];
            int temperature = ((wh2_packet[1] & B00000111) << 8) + wh2_packet[2];
            // make negative
            if (wh2_packet[1] & B00001000) {
                temperature = -temperature;
            }
            uint8_t crc = 0;
            uint8_t len = 4;
            uint8_t addr = 0;
            // Indicated changes are from reference CRC-8 function in OneWire library
            while (len--) {
                uint8_t inbyte = wh2_packet[addr++];
                for (uint8_t i = 8; i; i--) {
                    uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
                    crc <<= 1; // changed from right shift
                    if (mix) crc ^= 0x31; // changed from 0x8C;
                    inbyte <<= 1; // changed from right shift
                }
            }
            if (crc == wh2_packet[4]) {
                wh2_valid = true;
            }
            else {
                wh2_valid = false;
            }

            extern volatile int have_data_sensor;
            // avoid change sensor data during update
            if (wh2_valid == true && have_data_sensor == 0) {
                extern volatile int temp;
                extern volatile int hum;
                extern volatile unsigned int cycles;
                temp = temperature;
                hum = humidity;
                cycles = cycles + 1;
                have_data_sensor = Sensor_ID;
            }

            extern volatile byte have_data_string;
            // avoid change sensor data during update
            if (have_data_string == 0) {
                str_Sensor_ID = Sensor_ID;
                str_humidity = humidity;
                str_temperature = temperature;
                str_cycles = cycles;
                str_wh2_valid = wh2_valid;
                have_data_string = 1;
            }

            wh2_accept_flag = false;
        }
        wh2_flags = 0x00;
    }
    //--------------------------------------------------------

}

class WH2Sensor : public PollingComponent {
public:
    Sensor *sensor1 = new Sensor();
    Sensor *sensor2 = new Sensor();
    Sensor *sensor3 = new Sensor();

    WH2Sensor() : PollingComponent(1000) {
        ESP_LOGD("custom", "WH2Sensor::WH2Sensor()");
    }

    float get_setup_priority() const override {
        return esphome::setup_priority::DATA;
    }

    void setup() override {
        //For ESP8266,if you are using rx pin for reciever set to 3 below
        //For Sonoff Bridge with direct HW patch use pin 4
        ESP_LOGD("custom", "WH2Sensor.setup()");
        //Serial.end();
        attachInterrupt(33, ext_int_1, CHANGE);
    }
ext_int_1
    void update() override {
        // This is the actual sensor reading logic.
        extern volatile int have_data_sensor;
        extern volatile int temp;
        extern volatile int hum;
        extern volatile unsigned int cycles;

        if (have_data_sensor != 0) {
            ESP_LOGD("custom", "have_data_sensor: %d, temp: %d, hum: %d, cycles: %u",
                        have_data_sensor, temp, hum, cycles);
        }
        if (have_data_sensor == 77 || have_data_sensor == 1122) {
            sensor1->publish_state(temp * 0.1f);
            sensor2->publish_state(float(hum));
            sensor3->publish_state(cycles);
            have_data_sensor = 0;
        } else if (have_data_sensor) {
            have_data_sensor = 0;
        }
    }
protected:
    ISRInternalGPIOPin pin_;
};

class WH2LastSensorData : public PollingComponent, public TextSensor {
public:

    // constructor
    WH2LastSensorData() : PollingComponent(1000) {
        ESP_LOGD("custom", "WH2LastSensorData()");
        sensor_data_.reserve(100);
    }

    float get_setup_priority() const override {
        return esphome::setup_priority::DATA;
    }

    void setup() override {
        // This will be called by App.setup()
        ESP_LOGD("custom", "WH2LastSensorData.setup()");
    }

    void update() override {
        extern volatile byte have_data_string;
        extern volatile int str_Sensor_ID;
        extern volatile int str_humidity;
        extern volatile int str_temperature;
        extern volatile unsigned int str_cycles;
        extern volatile bool str_wh2_valid;

        // This will be called every "update_interval" milliseconds.
        if (have_data_string == 1) {
            if (str_wh2_valid) {
                char temperture_buf[7] = {0, }; // 7 bytes is enough for -100.0 with a '\0'

                snprintf(temperture_buf, sizeof(temperture_buf), "%.1f", (str_temperature * 0.1f));
                sensor_data_.clear();
                sensor_data_ += "| Sensor ID:  ";
                sensor_data_ += to_string(str_Sensor_ID);
                sensor_data_ += " | humidity: ";
                sensor_data_ += to_string(str_humidity);
                sensor_data_ += "% | temperature: ";
                sensor_data_ += to_string(temperture_buf);
                sensor_data_ += "°C | ";
                sensor_data_ += to_string(str_wh2_valid ? "OK " : "BAD");
                // "cycles: ";
                // my_data += to_string(str_cycles);
                // my_data += "| ";

                // Publish state
                publish_state(sensor_data_);
                have_data_string = 0;
            }
            else if (!str_wh2_valid) {
                ESP_LOGD("custom", "BAD skipped");
            }
        }
    }

protected:
    std::string sensor_data_;
};

class WH2BadSensorData : public PollingComponent, public TextSensor {
public:

    // constructor
    WH2BadSensorData() : PollingComponent(1000) {
        ESP_LOGD("custom", "WH2BadSensorData()");
        sensor_data_.reserve(100);
    }

    float get_setup_priority() const override {
        return esphome::setup_priority::DATA;
    }

    void setup() override {
        // This will be called by App.setup()
        ESP_LOGD("custom", "WH2BadSensorData.setup()");
    }

    // This will be called every "update_interval" milliseconds.
    void update() override {
        extern volatile byte have_data_string;
        extern volatile int str_Sensor_ID;
        extern volatile int str_humidity;
        extern volatile int str_temperature;
        extern volatile unsigned int str_cycles;
        extern volatile bool str_wh2_valid;

        if (have_data_string == 1) {
            if (!str_wh2_valid) {
                char temperture_buf[7] = {0, }; // 7 bytes is enough for -100.0 with a '\0'

                snprintf(temperture_buf, sizeof(temperture_buf), "%.1f", (str_temperature * 0.1f));
                sensor_data_.clear();
                sensor_data_ += "| Sensor ID:  ";
                sensor_data_ += to_string(str_Sensor_ID);
                sensor_data_ += " | humidity: ";
                sensor_data_ += to_string(str_humidity);
                sensor_data_ += "% | temperature: ";
                sensor_data_ += to_string(temperture_buf);
                sensor_data_ += "°C | ";
                sensor_data_ += to_string(str_wh2_valid ? "OK " : "BAD");
                // "cycles: ";
                // my_data += to_string(str_cycles);
                // my_data += "| ";

                // Publish state
                publish_state(sensor_data_);
                have_data_string = 0;
            }
            else if (str_wh2_valid) {
                ESP_LOGD("custom", "OK skipped");
            }
        }
    }

protected:
    std::string sensor_data_;
};

#endif
