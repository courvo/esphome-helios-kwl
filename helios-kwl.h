#pragma once

#include <numeric>
#include <cmath>
#include <sstream>  // Include this for std::stringstream
#include <iomanip>  // Include this for std::setw and std::setfill
#include <bitset>   // Include this for std::bitset

#include <array>
#include <string>

//# include helios-variables.h

#include "esphome.h"

struct VariableInfo {
    uint8_t variable;
    std::string variableName;
    uint8_t Data_binaryValue;
    uint8_t Data_hexValue;
    int Data_bitPosition;
    std::string Data_Definition;
    std::array<int, 2> values;
    std::string description;
};


std::array<VariableInfo, 87> variableInfos = {{
    {0x06, "I/O port", 0b00000001, 0x01, 0, "Speed 1", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b00000010, 0x02, 1, "Speed 2", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b00000100, 0x04, 2, "Speed 3", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b00001000, 0x08, 3, "Speed 4", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b00010000, 0x10, 4, "Speed 5", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b00100000, 0x20, 5, "Speed 6", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b01000000, 0x40, 6, "Speed 7", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x06, "I/O port", 0b10000000, 0x80, 7, "Speed 8", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x07, "I/O port", 0b00010000, 0x10, 5, "Reheating", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x08, "I/O port", 0b00000001, 0x01, 1, "Damper Motor Position", {0, 1}, "0 = Winter, 1 = Summer (Read-only)"},
    {0x08, "I/O port", 0b00000010, 0x02, 2, "Fault Information Relay", {0, 1}, "0 = Open, 1 = Closed (Read-only)"},
    {0x08, "I/O port", 0b00000100, 0x04, 3, "Input Fan", {0, 1}, "0 = On, 1 = Off"},
    {0x08, "I/O port", 0b00001000, 0x08, 4, "Preheating", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x08, "I/O port", 0b00010000, 0x10, 5, "Exhaust Fan", {0, 1}, "0 = On, 1 = Off"},
    {0x08, "I/O port", 0b00100000, 0x20, 6, "Fireplace/Boost Switch", {0, 1}, "0 = Open, 1 = Closed (Read-only)"},
    {0x29, "CURRENT FAN SPEED", 0b00000001, 0x01, -1, "", {0, 0}, "Speed 1"},
    {0x29, "CURRENT FAN SPEED", 0b00000011, 0x03, -1, "", {0, 0}, "Speed 2"},
    {0x29, "CURRENT FAN SPEED", 0b00000111, 0x07, -1, "", {0, 0}, "Speed 3"},
    {0x29, "CURRENT FAN SPEED", 0b00001111, 0x0F, -1, "", {0, 0}, "Speed 4"},
    {0x29, "CURRENT FAN SPEED", 0b00011111, 0x1F, -1, "", {0, 0}, "Speed 5"},
    {0x29, "CURRENT FAN SPEED", 0b00111111, 0x3F, -1, "", {0, 0}, "Speed 6"},
    {0x29, "CURRENT FAN SPEED", 0b01111111, 0x7F, -1, "", {0, 0}, "Speed 7"},
    {0x29, "CURRENT FAN SPEED", 0b11111111, 0xFF, -1, "", {0, 0}, "Speed 8"},
    {0x2A, "MAXIMUM CURRENT MEASURED MOISTURE CONTENT", 0, 0, -1, "", {0, 0}, "Read-only 33H = 0% RH, FFH = 100% RH calculation formula: (x-51) / 2.04"},
    {0x2B, "CURRENT CURRENT MEASURED CO2 CONCENTRATION upper", 0, 0, -1, "", {0, 0}, "Read-only The CO2 concentration in 16-bit upper byte directly indicates the concentration in PPM"},
    {0x2C, "MAXIMUM CURRENT MEASURED CO2 CONCENTRATION lower", 0, 0, -1, "", {0, 0}, "Read-only The CO2 concentration in 16-bit subbyte directly indicates the concentration in PPM"},
    {0x2D, "MILLIAMPER/VOLTAGE MESSAGE", 0, 0, 1, "Sensor 1", {0, 1}, "0 = Not installed, 1 = Installed"},
    {0x2D, "MILLIAMPER/VOLTAGE MESSAGE", 0, 0, 2, "Sensor 2", {0, 1}, "0 = Not installed, 1 = Installed"},
    {0x2D, "MILLIAMPER/VOLTAGE MESSAGE", 0, 0, 3, "Sensor 3", {0, 1}, "0 = Not installed, 1 = Installed"},
    {0x2D, "MILLIAMPER/VOLTAGE MESSAGE", 0, 0, 4, "Sensor 4", {0, 1}, "0 = Not installed, 1 = Installed"},
    {0x2D, "MILLIAMPER/VOLTAGE MESSAGE", 0, 0, 5, "Sensor 5", {0, 1}, "0 = Not installed, 1 = Installed"},
    {0x2E, "CO2 SENSORS INSTALLED ON THE MACHINE", 0, 0, -1, "", {0, 0}, "Read-only Current mA/voltage message to the machine on a scale of 00H to FFH"},
    {0x2F, "MEASURED% RH CONCENTRATION FROM SENSOR 1", 0, 0, -1, "", {0, 0}, "Read-only calculation formula: (x-51) / 2.04"},
    {0x30, "MEASURED% RH CONCENTRATION FROM SENSOR 2", 0, 0, -1, "", {0, 0}, "Read-only calculation formula: (x-51) / 2.04"},
    {0x32, "OUTDOOR TEMPERATURE", 0, 0, -1, "", {0, 0}, "Read-only Outdoor temperature on the NTC sensor scale"},
    {0x33, "EXHAUST TEMPERATURE", 0, 0, -1, "", {0, 0}, "Read-only Exhaust air temperature on the NTC sensor scale"},
    {0x34, "InDOOR TEMPERATURE", 0, 0, -1, "", {0, 0}, "Read-only Exhaust air temperature on the NTC sensor scale"},
    {0x35, "SUPPLY AIR TEMPERATURE", 0, 0, -1, "", {0, 0}, "Read-only Supply air temperature on the NTC sensor scale"},
    {0x36, "FAULT CONDITION ERROR NUMBER", 0, 0, -1, "05, 06, 07, 08, 09, 0A", {0, 0}, "Read-only The number of the last fault: 05 = Supply air sensor fault, 06 = Carbon dioxide alarm, 07 = Outdoor sensor fault, 08 = Exhaust air sensor fault, 09 = Danger of the water coil freezing, 0A = Exhaust air sensor fault"},
    {0x55, "POST HEATING ON COUNTER", 0, 0, -1, "", {0, 0}, "Post-heating on time in seconds, counter (as a percentage: X / 2.5)"},
    {0x56, "POST HEATING OFF TIME", 0, 0, -1, "", {0, 0}, "Post-heating off-time in seconds, counter (as a percentage: X / 2.5)"},
    {0x57, "POST HEATING TARGET VALUE", 0, 0, -1, "", {0, 0}, "Read-only Target temperature of the air blown into the ventilation zone on the NTC sensor scale"},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 0, "CO2 Higher Speed Request", {0, 1}, ""},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 1, "CO2 Lower Speed Request", {0, 1}, ""},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 2, "% RH Lower Speed Request", {0, 1}, ""},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 3, "Switch Lower Speed Request", {0, 1}, ""},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 6, "CO2 Alarm", {0, 1}, ""},
    {0x6D, "FLAGS 2 flag variables", 0, 0, 7, "Cell Freeze Alarm", {0, 1}, ""},
    {0x6F, "FLAGS 4 flag variables", 0, 0, 4, "Risk of Water Coil Freezing", {0, 1}, "0 = No, 1 = Risk of Freezing"},
    {0x6F, "FLAGS 4 flag variables", 0, 0, 7, "Slave/Master Selection", {0, 1}, "0 = Slave, 1 = Master"},
    {0x70, "FLAGS 5 flag variables", 0, 0, 7, "Preheating Status Flag", {0, 1}, "0 = On, 1 = Off"},
    {0x71, "FLAGS 6 flag variable", 0, 0, 4, "Remote Control", {0, 1}, "0 = Not Working, 1 = Functioning"},
    {0x71, "FLAGS 6 flag variable", 0, 0, 5, "Fireplace Switch Activation", {0, 1}, "Read the variable and set this to one"},
    {0x71, "FLAGS 6 flag variable", 0, 0, 6, "Fireplace/Boost Function", {0, 1}, "0 = Not Working, 1 = Functioning (Read-only)"},
    {0x71, "FLAGS 6 flag variable", 0, 0, 7, "Service Reminder", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x79, "FIREPLACE/POWER SWITCH COUNTER", 0, 0, -1, "", {0, 0}, "Read-only Remaining time of the function in minutes, descending"},
    {0x8F, "TRANSMISSION ALLOWED ONLY IN WRITING", 0, 0, -1, "", {0, 0}, "The modules are allowed to send data to the RS-485 bus. DATA = always 0."},
    {0x91, "TRANSMISSION PROHIBITED", 0, 0, -1, "", {0, 0}, "Write only Modules are prohibited from sending data to the RS-485 bus. DATA = always"},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 0, "Power Key", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 1, "CO2 Key", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 2, "% RH Key", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 3, "Post Heating Key", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 4, "Filter Guard LED", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 5, "Post Heating LED", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 6, "Fault LED", {0, 1}, ""},
    {0xA3, "SELECT VARIABLES: INDICATOR LIGHTS", 0, 0, 7, "Service Reminder LED", {0, 1}, ""},
    {0xA4, "POST HEATING SETPOINT", 0, 0, -1, "", {0, 0}, "Post-heating target value on the NTC sensor scale."},
    {0xA5, "MAX FAN SPEED", 0, 0, -1, "", {0, 0}, "01, 03, 07, 0F, 1F, 3F, 7F, FF Maximum fan speed that can be set during adjustments. Allowed values: 01 = speed 1, ..., FF = speed 8"},
    {0xA6, "SERVICE REMINDER INTERVAL", 0, 0, -1, "", {0, 0}, "Service reminder interval in months."},
    {0xA7, "PREHEATING SWITCHING TEMPERATURE", 0, 0, -1, "", {0, 0}, "Preheating switching temperature on the NTC sensor scale."},
    {0xA8, "SUPPLY AIR FAN STOP TEMPERATURE", 0, 0, -1, "", {0, 0}, "Supply air fan stop temperature on the NTC sensor scale."},
    {0xA9, "BASIC FAN SPEED", 0, 0, -1, "", {0, 0}, "01, 03, 07, 0F, 1F, 3F, 7F, FF Allowed values: 01 = speed 1, ..., FF = speed 8"},
    {0xAA, "PROGRAM VARIABLE", 0, 0, 0, "Adjustment Interval", {0, 0}, "4-bit Adjustment interval in 4-bit"},
    {0xAA, "PROGRAM VARIABLE", 0, 0, 4, "Automatic Humidity Level Search", {0, 1}, "0 = Off, 1 = On"},
    {0xAA, "PROGRAM VARIABLE", 0, 0, 5, "Boost/Fireplace Switch Status", {0, 1}, "0 = Fireplace, 1 = Boost Switch"},
    {0xAA, "PROGRAM VARIABLE", 0, 0, 6, "Water/Radiator Model", {0, 1}, "0 = Electricity, 1 = Water"},
    {0xAA, "PROGRAM VARIABLE", 0, 0, 7, "Cascade Control", {0, 1}, "0 = Off, 1 = On"},
    {0xAB, "", 0, 0, -1, "", {0, 0}, "The monthly counter of the service reminder indicates the time remaining in months for the next service alarm. Downward."},
    {0xAE, "BASIC HUMIDITY LEVEL", 0, 0, -1, "", {0, 0}, "33H = 0% RH, FFH = 100% RH Read-only calculation formula: (x-51) / 2.04"},
    {0xAF, "BYPASS OPERATING TEMPERATURE", 0, 0, -1, "", {0, 0}, "Bypass operating temperature on the NTC sensor scale."},
    {0xB0, "DC SUPPLY AIR FAN CONTROL SETPOINT", 0, 0, -1, "", {0, 0}, "DC supply air fan control setpoint as a percentage."},
    {0xB1, "DC EXHAUST FAN CONTROL SETPOINT", 0, 0, -1, "", {0, 0}, "DC exhaust fan control setpoint as a percentage."},
    {0xB2, "CELL ANTI-FREEZE TEMPERATURE HYSTERESIS", 0, 0, -1, "", {0, 0}, "03H ≅ 1 °C Hysteresis of cell antifreeze temperatures, 03H ≅ 1 °C."},
    {0xB3, "CARBON DIOXIDE CONTROL SETPOINT 16 BIT", 0, 0, -1, "", {0, 0}, "B3 - - - Carbon dioxide control setpoint in 16-bit, the upper byte directly indicates the concentration PPM."},
    {0xB4, "CARBON DIOXIDE CONTROL SETPOINT 16 BIT", 0, 0, -1, "", {0, 0}, "Carbon dioxide control setpoint in 16-bit, lower byte directly indicates PPM concentration."},
    {0xB5, "VARIABLE PROGRAM2", 0, 0, 0, "Maximum Speed Limit", {0, 1}, "0 = With adjustments, 1 = Always On"},

}};

class HeliosKwlComponent : public PollingComponent, public UARTDevice {
 private:
  static constexpr uint8_t ADDRESS = 0x22;
  static constexpr uint8_t MAINBOARD = 0x11;

double calculateTemperature(int x) {
    return std::round((0.00001034 * std::pow(x, 3) - 0.00348 * std::pow(x, 2) + 0.70127 * x - 45.78) * 10) / 10;
}

 public:
  using Datagram = std::array<uint8_t, 6>;

HeliosKwlComponent(UARTComponent* parent, Sensor* fan_speed, Sensor* sensor_temperature_outside,
                   Sensor* sensor_temperature_exhaust, Sensor* sensor_temperature_inside,
                   Sensor* sensor_temperature_incoming, BinarySensor* power_state,
                   BinarySensor* bypass_state, BinarySensor* heating_indicator,
                   BinarySensor* fault_indicator, BinarySensor* service_reminder)
    : PollingComponent(10 * 1000),
      UARTDevice(parent),
      m_fan_speed{fan_speed},
      m_sensor_temperature_outside{sensor_temperature_outside},
      m_sensor_temperature_exhaust{sensor_temperature_exhaust},
      m_sensor_temperature_inside{sensor_temperature_inside},
      m_sensor_temperature_incoming{sensor_temperature_incoming},
      m_power_state{power_state},
      m_bypass_state{bypass_state},
      m_heating_indicator{heating_indicator},
      m_fault_indicator{fault_indicator},
      m_service_reminder{service_reminder} {}


  void setup() override { ESP_LOGI("custom", "setup()"); 
  
   // start KWL unit at startup of ESPHome (turn fan on)
    for (uint8_t byte : {0x01, 0x21, 0x11, 0xa3, 0x01, 0xd7}) {
        write(byte);
    }
    flush();
  }





void update() override {
    if (const auto value = poll_register(0x32)) {
        m_sensor_temperature_outside->publish_state(calculateTemperature(*value));   
    }
    if (const auto value = poll_register(0x33)) {
        m_sensor_temperature_exhaust->publish_state(calculateTemperature(*value));   
    }
    if (const auto value = poll_register(0x34)) {
        m_sensor_temperature_inside->publish_state(calculateTemperature(*value));    
    }
    if (const auto value = poll_register(0x35)) {
        m_sensor_temperature_incoming->publish_state(calculateTemperature(*value));  
    }

    if (const auto value = poll_register(0x29)) {
        m_fan_speed->publish_state(count_ones(*value));                    
    }
    if (const auto value = poll_register(0xA3)) {
        m_power_state->publish_state(*value & (0x01 << 0));                 
        m_bypass_state->publish_state(*value & (0x01 << 3));               
        m_heating_indicator->publish_state(*value & (0x01 << 5));          
        m_fault_indicator->publish_state(*value & (0x01 << 6));             
        m_service_reminder->publish_state(*value & (0x01 << 7));           
    }
}


uint32_t last_packet_time = 0;
uint8_t last_request_sender = 0;
uint8_t last_request_recipient = 0;
uint8_t previous_checksum = 0;  // Global variable to store the checksum of the previous packet

std::string get_module_name(uint8_t module) {
    return (module == 0x10) ? "All MotherB." :
           (module >= 0x11 && module <= 0x1F) ? "MotherB. IO " + std::to_string(module) :
           (module == 0x20) ? "All Remotes" :
           (module >= 0x21 && module <= 0x2F) ? "Remote Contr." + std::to_string(module) :
           "Reserved Module " + std::to_string(module);
}

std::string packetToHexString(const std::vector<uint8_t>& packet) {
    std::stringstream ss;
    for (const auto& byte : packet) {
        ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
    }
    return ss.str();
}

void loop() override {
    std::vector<uint8_t> packet_buffer;
    while (available()) {
        uint8_t byte = read();
        if (byte == 0x01) {
            packet_buffer.push_back(byte);
            for (int i = 0; i < 5 && available(); i++) {
                packet_buffer.push_back(read());
            }
            std::string packetHexString = packetToHexString(packet_buffer);
            interpret_packet(packet_buffer, packetHexString);
        } else if (byte == previous_checksum) {
            std::string packetHexString = packetToHexString({byte});
            interpret_packet({byte}, packetHexString);
        } else if (byte >= 0xE0 && byte <= 0xE5) {  // Check if the byte is an error
            ESP_LOGI("custom", "ERROR x%02x", byte);
            continue;
        } else {
            ESP_LOGI("custom", "Single byte: %c", byte);
            continue;
        }
        packet_buffer.clear();
    }
}

enum class SearchType {
    VariableName,
    DataDefinition
};

std::string findInfo(uint8_t packetNumber, uint8_t data, SearchType type) {
    for (const auto& info : variableInfos) {
        if (info.variable == packetNumber) {
            if (type == SearchType::VariableName) {
                return info.variableName;
            } else if (type == SearchType::DataDefinition && info.Data_hexValue == data) {
                return info.Data_Definition;
            }
        }
    }
    return (type == SearchType::VariableName) ? "" : std::to_string(data);
}

std::string createLogMessage(const std::string& packetHexString, const std::string& packet_type, const std::string& sender, const std::string& recipient, const std::string& variable, uint8_t data, uint8_t checksum, int time_since_last_packet, const std::string& dataDefinition) {
    std::stringstream logMessage;
    logMessage << std::left << std::setw(20) << packetHexString << "\t" 
               << std::setw(10) << packet_type << "\tFrom: " << std::setw(20) << sender << "\tTo: " << std::setw(20) << recipient << "\t";
    
    if (std::all_of(variable.begin(), variable.end(), ::isxdigit)) {
        logMessage << "Variable: ";
    }
    logMessage << std::setw(30) << variable << "\t";
    
    if (dataDefinition == std::to_string(data)) {
        logMessage << "Data: ";
    }
    
    logMessage << std::setw(20) << dataDefinition << "\tChecksum: " << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(checksum) << "\tCycleTime: " << std::dec << time_since_last_packet << " ms";
    return logMessage.str();
}

void interpret_packet(const std::vector<uint8_t>& packet, const std::string& packetHexString) {
    uint32_t current_time = millis();
    uint32_t time_since_last_packet = current_time - last_packet_time;
    last_packet_time = current_time;

    if (packet.size() == 1 && packet[0] == previous_checksum) {
        ESP_LOGI("custom", "Acknowledged\tChecksum: %02x\tCycleTime: %d ms", packet[0], time_since_last_packet);
    } else {
        uint8_t system = packet[0], sender = packet[1], recipient = packet[2], variable = packet[3], data = packet[4], checksum = packet[5];

        std::string packet_type = "Set Value";
        if (variable == 0) {
            packet_type = "Request ";
            last_request_sender = sender;
            last_request_recipient = recipient;
        } else if (sender == last_request_recipient && recipient == last_request_sender) {
            packet_type = "Response";
        } else if ((recipient == 0x10 || recipient == 0x20) && variable != 0) {
            packet_type = "Broadcast";
        }

        previous_checksum = checksum;

        std::string variableName;
        if (packet_type == "Request ") {
            variableName = findInfo(data, 0, SearchType::VariableName);
            if (variableName.empty()) {
                variableName = std::to_string(data);
            }
        } else {
            variableName = findInfo(variable, 0, SearchType::VariableName);
            if (variableName.empty()) {
                variableName = std::to_string(variable);
            }
        }

        std::string dataDefinition;
        if (packet_type != "Request ") {
            dataDefinition = findInfo(variable, data, SearchType::DataDefinition);
            if (dataDefinition.empty()) {
                std::stringstream ss;
                ss << std::hex << static_cast<int>(data);
                dataDefinition = ss.str();
            }
        }

        std::string logMessage = createLogMessage(packetHexString, packet_type, get_module_name(sender), get_module_name(recipient), variableName, data, checksum, time_since_last_packet, dataDefinition);
        ESP_LOGI("custom", "%s", logMessage.c_str());
    }
}

void set_fan_speed(float speed) {
    if (speed == 0.f) {
        set_state_flag(0, false);                                            
    } else {
        assert(speed >= 0.f && speed <= 8.f);                                
        const uint8_t speed_byte = 0xFF >> (8 - static_cast<int>(speed * 8)); 
        if (set_value(0x29, speed_byte)) {                                    
            ESP_LOGD("custom", "Wrote speed: %02x", speed_byte);              
            set_state_flag(0, true);                                          
        } else {
            ESP_LOGE("custom", "Failed to set fan speed");                     
        }
    }
}

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
    Datagram temp = {0x01, ADDRESS, MAINBOARD, 0x00, address};
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

  bool set_value(uint8_t address, uint8_t value) {
    Datagram temp = {0x01, ADDRESS, MAINBOARD, address, value};
    temp[5] = checksum(temp.cbegin(), temp.cend());

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
  Sensor* m_fan_speed{nullptr};
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

  void write_state(float state) override { m_component->set_fan_speed(state); }

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


