#pragma once

#include <numeric>
#include <cmath>
#include <sstream>  // Include this for std::stringstream
#include <iomanip>  // Include this for std::setw and std::setfill
#include <bitset>   // Include this for std::bitset

#include <array>
#include <string>

#include "esphome.h"


//hier stand das array

class HeliosKwlComponent : public PollingComponent, public UARTDevice {
 private:

  static constexpr uint8_t SYSTEM = 0x01;
  static constexpr uint8_t ADDRESS = 0x22;
  static constexpr uint8_t MAINBOARD = 0x11;
  static constexpr uint8_t FAN_SPEED_REGISTER = 0x29;
  static constexpr uint8_t STATE_FLAG_REGISTER = 0xA3;

//calcualting temperature from NTC sensor value, instead of reading from a table array. basis was an array of 256 matching values.

double calculateTemperature(int x) {
    return std::round((0.00001034 * std::pow(x, 3) - 0.00348 * std::pow(x, 2) + 0.70127 * x - 45.78) * 10) / 10;
}

 public:
  using Datagram = std::array<uint8_t, 6>;

HeliosKwlComponent(UARTComponent* parent, Sensor* act_fan_speed, Sensor* sensor_temperature_outside,
                   Sensor* sensor_temperature_exhaust, Sensor* sensor_temperature_inside,
                   Sensor* sensor_temperature_incoming, BinarySensor* power_state,
                   BinarySensor* bypass_state, BinarySensor* heating_indicator,
                   BinarySensor* fault_indicator, BinarySensor* service_reminder)
    : PollingComponent(10 * 1000),
      UARTDevice(parent),
      m_act_fan_speed{act_fan_speed},
      m_sensor_temperature_outside{sensor_temperature_outside},
      m_sensor_temperature_exhaust{sensor_temperature_exhaust},
      m_sensor_temperature_inside{sensor_temperature_inside},
      m_sensor_temperature_incoming{sensor_temperature_incoming},
      m_power_state{power_state},
      m_bypass_state{bypass_state},
      m_heating_indicator{heating_indicator},
      m_fault_indicator{fault_indicator},
      m_service_reminder{service_reminder} {}


void setup() override {
    set_interval("update_sensors", 10000, [this]() {
        this->update();
    });
}

bool datagrams_sent = false;

//startup of KWL unit, 7 seconds after ESP32 has started. Does not work in Setup();therefore put in update() with a tested delay of 7 seconds (7000ms
// polling the registers for the sensors and flags, based on Variable Hex Values 0xXX

// Vermeiden Sie Code-Wiederholung
void publish_sensor_value(Sensor* sensor, uint8_t register_address, std::function<double(int)> calculate) {
    if (const auto value = poll_register(register_address)) {
        sensor->publish_state(calculate(*value));
    }
}

void send_initial_datagram() {
    ESP_LOGI("custom", "setup()");
    std::array<uint8_t, 6> datagram = {0x01, 0x21, 0x11, 0xa3, 0x01, 0xd7};
    for (uint8_t byte : datagram) write(byte);
    flush();
    ESP_LOGI("custom", "Datagram 1 sent: %lu ms", millis());
}

void update() override {
    if (!datagrams_sent && millis() >= 7000) {
        send_initial_datagram();
        datagrams_sent = true;
    }

    publish_sensor_value(m_sensor_temperature_outside, 0x32, [this](int x) { return this->calculateTemperature(x); });
    publish_sensor_value(m_sensor_temperature_exhaust, 0x33, [this](int x) { return this->calculateTemperature(x); });
    publish_sensor_value(m_sensor_temperature_inside, 0x34, [this](int x) { return this->calculateTemperature(x); });
    publish_sensor_value(m_sensor_temperature_incoming, 0x35, [this](int x) { return this->calculateTemperature(x); });

    if (const auto value = poll_register(FAN_SPEED_REGISTER)) {
        m_act_fan_speed->publish_state(count_ones(*value));
    }
    if (const auto value = poll_register(STATE_FLAG_REGISTER)) {
        m_power_state->publish_state(*value & (0x01 << 0));
        m_bypass_state->publish_state(*value & (0x01 << 3));
        m_heating_indicator->publish_state(*value & (0x01 << 5));
        m_fault_indicator->publish_state(*value & (0x01 << 6));
        m_service_reminder->publish_state(*value & (0x01 << 7));
    }
}


//Hier stand der code, der jetzt in LOG.h steht


//orginial code, setting the fan speed based on the speed value, will be replaced by new concept

void set_fan_speed(float speed) {
    static const uint8_t speed_bytes[] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};
    if (speed == 0.f) {
        set_state_flag(0, false);
    } else {
        assert(speed >= 1.f && speed <= 8.f);
        const uint8_t speed_byte = speed_bytes[static_cast<int>(speed) - 1];
        if (set_value(FAN_SPEED_REGISTER, speed_byte)) {
            ESP_LOGD("custom", "Wrote speed: %02x", speed_byte);
            set_state_flag(0, true);
        } else {
            ESP_LOGE("custom", "Failed to set fan speed");
        }
    }
}

//

void set_state_flag(uint8_t bit, bool state) {
    if (auto value = poll_register(0xA3)) {               
        if (state == ((*value >> bit) & 0x01)) {          
            ESP_LOGD("custom", "State flag already set"); 
        } else {
            if (state) {
                *value |= 0x01 << bit;                    
            } else {
                *value &= ~(0x01 << bit);               
            }
            if (set_value(0xA3, *value)) {
                ESP_LOGD("custom", "Wrote state flag to: %x", *value);  
            } else {
                ESP_LOGE("custom", "Failed to set state flag");         
            }
        }
    } else {
        ESP_LOGE("custom", "Unable to poll register 0xA3");            
    }
}

  
private:
  optional<uint8_t> poll_register(uint8_t address) {
    flush_read_buffer();
    Datagram temp = {SYSTEM, ADDRESS, MAINBOARD, 0x00, address};
    temp[5] = checksum(temp.cbegin(), temp.cend());

    write_array(temp);
    flush();
    if (const auto response = read_array<6>()) {
      const auto& array = *response;

      if (check_crc(array.cbegin(), array.cend())) {
        if (array[1] == MAINBOARD && array[2] == ADDRESS && array[3] == address) {
          return array[4];
        } else {
          ESP_LOGE("custom", "Wrong response from mainboard");
        }
      } else {
        ESP_LOGE("custom", "Bad checksum for response");
      }
    }
    return {};
  }

  // bool set_value(uint8_t address, uint8_t value) {
  //   Datagram temp = {SYSTEM, ADDRESS, MAINBOARD, address, value};
  //   temp[5] = checksum(temp.cbegin(), temp.cend());

  //   // To the mainboard
  //   int retry = 3;
  //   do {
  //     // Flush read buffer
  //     flush_read_buffer();
  //     // Write
  //     write_array(temp);
  //     flush();
  //   } while (read() != temp[5] && retry-- > 0);

  //   return retry >= 0;
  // }

bool set_value(uint8_t address, uint8_t value) {
    Datagram temp = {SYSTEM, ADDRESS, MAINBOARD, address, value};
    temp[5] = checksum(temp.cbegin(), temp.cend());

    // Convert the datagram to a single line of hex values
    std::stringstream ss;
    for (const auto &byte : temp) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    ESP_LOGD("custom", "Datagram: %s", ss.str().c_str());

    // To the mainboard
    int retry = 3;
    do {
      // Flush read buffer
      flush_read_buffer();
      // Write
      write_array(temp);
      flush();
    } while (read() != temp[5] && retry-- > 0);

    return retry >= 0;
}

  void flush_read_buffer() {
    // Flush read buffer and wait for the bus to be quiet for 10 ms
    uint32_t last_time = millis();
    while (millis() - last_time < 10) {
      while (available()) {
        read();
        last_time = millis();
      }
      yield();
    }
  }

  template <typename Iterator>
  static bool check_crc(const Iterator begin, const Iterator end) {
    const auto crc = checksum(begin, std::prev(end));
    return *std::prev(end) == crc;
  }

  template <typename Iterator>
  static uint8_t checksum(const Iterator begin, const Iterator end) {
    return std::accumulate(begin, end, 0);
  }

  static uint8_t count_ones(uint8_t byte) {
    static const uint8_t NIBBLE_LOOKUP[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
    return NIBBLE_LOOKUP[byte & 0x0F] + NIBBLE_LOOKUP[byte >> 4];
  }


private:
  Sensor* m_act_fan_speed{nullptr};
  Sensor* m_sensor_temperature_outside{nullptr};
  Sensor* m_sensor_temperature_exhaust{nullptr};
  Sensor* m_sensor_temperature_inside{nullptr};
  Sensor* m_sensor_temperature_incoming{nullptr};

  BinarySensor* m_power_state{nullptr};
  BinarySensor* m_bypass_state{nullptr};
  BinarySensor* m_heating_indicator{nullptr};
  BinarySensor* m_fault_indicator{nullptr};
  BinarySensor* m_service_reminder{nullptr};
};

// constexpr double HeliosKwlComponent::TEMPERATURE[256];

class HeliosKwlSpeedOutput : public Component, public FloatOutput {
 public:
  HeliosKwlSpeedOutput(HeliosKwlComponent* component) : m_component{component} {}

  void setup() override {}

  void write_state(float state) override { 
    int speed_level = round(state * 8);
    m_component->set_fan_speed(esphome::clamp(speed_level, 0, 8)); 
  }

 private:
  HeliosKwlComponent* m_component;
};

class HeliosKwlWinterModeOutput : public Component, public BinaryOutput {
 public:
  HeliosKwlWinterModeOutput(HeliosKwlComponent* component) : m_component{component} {}

  void setup() override {}

  void write_state(bool state) override { m_component->set_state_flag(3, state); }

 private:
  HeliosKwlComponent* m_component;
};


