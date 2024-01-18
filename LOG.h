// LOG.h
#ifndef LOG_H
#define LOG_H

#include <vector>
#include <string>
#include <map>
#include <functional>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>

// interpreting the packets sent by the KWL unit, based on Variable Hex Values 0xXX for logging

uint32_t last_packet_time = 0;
uint8_t last_request_sender = 0;
uint8_t last_request_recipient = 0;
uint8_t previous_checksum = 0;  // Global variable to store the checksum of the previous packet

// function to get the name of the module based on the module number, in Hex code

std::string get_module_name(uint8_t module) {
    return (module == 0x10) ? "All KWL units." :
           (module >= 0x11 && module <= 0x1F) ? "KWL " + std::to_string(module - 16) :
           (module == 0x20) ? "All Remotes" :
           (module >= 0x21 && module <= 0x2F) ? "Remote " + std::to_string(module - 32) :
           "Reserved Module " + std::to_string(module);
}
// function to convert the packet to a string of hex values, to still have the raw HEX values in the log

std::string packetToHexString(const std::vector<uint8_t>& packet) {
    std::stringstream ss;
    for (const auto& byte : packet) {
        ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
    }
    return ss.str();
}

// function to interpret the packet and log it

void loop()  {
    std::vector<uint8_t> packet_buffer;
    while (available()) {
        uint8_t byte = read();
        if (byte == 0x01) {
            packet_buffer.push_back(byte);
            for (int i = 0; i < 5 && available(); i++) {
                packet_buffer.push_back(read());
            }
            std::string packetHexString = packetToHexString(packet_buffer);
            interpret_datagram(packet_buffer, packetHexString);
        } else if (byte == previous_checksum) {
            std::string packetHexString = packetToHexString({byte});
            interpret_datagram({byte}, packetHexString);
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

//functionmap
std::map<uint8_t, std::function<std::string(uint8_t)>> variableFunctionMap = {
    {0xA3, [](uint8_t data) {
        std::string status = "SELECT VARIABLES: INDICATOR LIGHTS, ";
        std::vector<std::string> parameters = {
            "Power Key", "CO2 Key", "% RH Key", "Post Heating Key", "Filter Guard LED", "Post Heating LED", "Fault LED", "Service Reminder LED"
        };
        for (int i = 0; i < 8; i++) {
            status += parameters[i] + ": " + ((data & (1 << i)) ? "On" : "Off") + ", ";
        }
        return status;
    }}
    // Add more entries here for other variable hex values
};


// function to find the variable name or data definition based on the Variable Hex Value and Data Hex Value

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

// function to create the log message

std::string createLogMessage(const std::string& packetHexString, const std::string& packet_type, const std::string& sender, const std::string& recipient, const std::string& variable, uint8_t data, uint8_t checksum, int time_since_last_packet, const std::string& dataDefinition) {
    std::stringstream logMessage;
    logMessage << std::left << std::setw(20) << packetHexString 
               << std::setw(10) << packet_type << "\tFrom: " << std::setw(10) << sender << "To: " << std::setw(10) << recipient << "\t";
    
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

// function to interpret the packet and log it, also interpret the type of packet (Request, Response, Broadcast, Acknowledged)

void interpret_datagram(const std::vector<uint8_t>& packet, const std::string& packetHexString) {
    uint32_t current_time = millis();
    uint32_t time_since_last_packet = current_time - last_packet_time;
    last_packet_time = current_time;

    if (packet.size() == 1 && packet[0] == previous_checksum) {
        ESP_LOGI("custom", "Acknowledged\tChecksum: %02x\tCycleTime: %d ms", packet[0], time_since_last_packet);
    } else {
        uint8_t system = packet[0], sender = packet[1], recipient = packet[2], variable = packet[3], data = packet[4], checksum = packet[5];

        std::string packet_type = "* Set Value *";
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


std::vector<VariableInfo> variableInfos =  {

    {0x07, "I/O Reheating", 0b00010000, 0x10, 5, "Reheating", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x08, "I/O Other Info", 0b00000001, 0x01, 1, "Damper Motor Position", {0, 1}, "0 = Winter, 1 = Summer (Read-only)"},
    {0x08, "I/O Other Info", 0b00000010, 0x02, 2, "Fault Information Relay", {0, 1}, "0 = Open, 1 = Closed (Read-only)"},
    {0x08, "I/O Other Info", 0b00000100, 0x04, 3, "Input Fan", {0, 1}, "0 = On, 1 = Off"},
    {0x08, "I/O Other Info", 0b00001000, 0x08, 4, "Preheating", {0, 1}, "0 = Off, 1 = On (Read-only)"},
    {0x08, "I/O Other Info", 0b00010000, 0x10, 5, "Exhaust Fan", {0, 1}, "0 = On, 1 = Off"},
    {0x08, "I/O Other Info", 0b00100000, 0x20, 6, "Fireplace/Boost Switch", {0, 1}, "0 = Open, 1 = Closed (Read-only)"},
    {0x29, "FAN SPEED", 0b00000001, 0x01, -1, "", {0, 0}, "Speed 1"},
    {0x29, "FAN SPEED", 0b00000011, 0x03, -1, "", {0, 0}, "Speed 2"},
    {0x29, "FAN SPEED", 0b00000111, 0x07, -1, "", {0, 0}, "Speed 3"},
    {0x29, "FAN SPEED", 0b00001111, 0x0F, -1, "", {0, 0}, "Speed 4"},
    {0x29, "FAN SPEED", 0b00011111, 0x1F, -1, "", {0, 0}, "Speed 5"},
    {0x29, "FAN SPEED", 0b00111111, 0x3F, -1, "", {0, 0}, "Speed 6"},
    {0x29, "FAN SPEED", 0b01111111, 0x7F, -1, "", {0, 0}, "Speed 7"},
    {0x29, "FAN SPEED", 0b11111111, 0xFF, -1, "", {0, 0}, "Speed 8"},
    {0x2A, "MAX CURRENT MOISTURE CONTENT", 0, 0, -1, "", {0, 0}, "Read-only 33H = 0% RH, FFH = 100% RH calculation formula: (x-51) / 2.04"},
    {0x2B, "CURRENT CO2 CONCENTRATION upper", 0, 0, -1, "", {0, 0}, "Read-only, The CO2 concentration in 16-bit upper byte directly indicates the concentration in PPM"},
    {0x2C, "MAXIMUM CO2 CONCENTRATION lower", 0, 0, -1, "", {0, 0}, "Read-only, The CO2 concentration in 16-bit subbyte directly indicates the concentration in PPM"},
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
    {0x34, "InDoor TEMPERATURE", 0, 0, -1, "", {0, 0}, "Read-only Exhaust air temperature on the NTC sensor scale"},
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
    {0xAB, "Service Reminder Month Counter", 0, 0, -1, "", {0, 0}, "The monthly counter of the service reminder indicates the time remaining in months for the next service alarm. Downward."},
    {0xAF, "BYPASS OPERATING TEMPERATURE", 0, 0, -1, "", {0, 0}, "Bypass operating temperature on the NTC sensor scale."},
    {0xB0, "DC SUPPLY AIR FAN CONTROL SETPOINT", 0, 0, -1, "", {0, 0}, "DC supply air fan control setpoint as a percentage."},
    {0xB1, "DC EXHAUST FAN CONTROL SETPOINT", 0, 0, -1, "", {0, 0}, "DC exhaust fan control setpoint as a percentage."},
    {0xB2, "CELL ANTI-FREEZE TEMPERATURE HYSTERESIS", 0, 0, -1, "", {0, 0}, "03H ≅ 1 °C Hysteresis of cell antifreeze temperatures, 03H ≅ 1 °C."},
    {0xB5, "VARIABLE PROGRAM2", 0, 0, 0, "Maximum Speed Limit", {0, 1}, "0 = With adjustments, 1 = Always On"},

};


#endif // LOG_H