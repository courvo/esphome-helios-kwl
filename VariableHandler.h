#ifndef VARIABLE_HANDLER_H
#define VARIABLE_HANDLER_H

#include <map>
#include <functional>
#include <string>

// Funktionen zur Behandlung spezifischer Variablen

std::string handleFanSpeed(uint8_t data) {
    return "Speed " + std::to_string(countSetBits(data));
}

std::string handleIOReheat(uint8_t data) {
    return (data & (1 << 5)) ? "On" : "Off";
}

std::string handleIODamperMotorPosition(uint8_t data) {
    return (data & 1) ? "Summer" : "Winter";
}

std::string handleIOFaultInformationRelay(uint8_t data) {
    return (data & (1 << 2)) ? "Closed" : "Open";
}

std::string handleIOInputFan(uint8_t data) {
    return (data & (1 << 3)) ? "Off" : "On";
}

std::string handleIOPreheating(uint8_t data) {
    return (data & (1 << 4)) ? "On" : "Off";
}

std::string handleIOExhaustFan(uint8_t data) {
    return (data & (1 << 5)) ? "Off" : "On";
}

std::string handleIOFireplaceBoostSwitch(uint8_t data) {
    return (data & (1 << 6)) ? "Closed" : "Open";
}

std::string handleCurrentFanSpeed(uint8_t data) {
    return "Speed " + std::to_string(countSetBits(data));
}

std::string handleMaxCurrentMeasuredMoistureContent(uint8_t data) {
    return std::to_string((data - 51) / 2.04) + "% RH";
}

std::string handleCurrentMeasuredCO2ConcentrationUpper(uint8_t data) {
    return std::to_string(data) + " PPM";
}

std::string handleMaxCurrentMeasuredCO2ConcentrationLower(uint8_t data) {
    return std::to_string(data) + " PPM";
}

std::string handleMilliamperVoltageMessage(uint8_t data) {
    return (data & 1) ? "Installed" : "Not installed";
}

// ... Weitere Funktionen ...

// Mapping von Funktionen
extern std::map<uint16_t, std::function<std::string(uint8_t)>> functionMap;

#endif // VARIABLE_HANDLER_H



GitHub Copilot: Um die spezifischen Anforderungen für jede Variable zu erfüllen, können wir spezielle Funktionen erstellen, die die Daten entsprechend behandeln. Hier ist ein Beispiel, wie Sie das tun könnten:

```cpp
// Funktionen zur Behandlung spezifischer Variablen
std::string handleFanSpeed(uint8_t data) {
    return "Speed " + std::to_string(countSetBits(data));
}

std::string handleIOReheat(uint8_t data) {
    return (data & (1 << 5)) ? "On" : "Off";
}

std::string handleIODamperMotorPosition(uint8_t data) {
    return (data & 1) ? "Summer" : "Winter";
}

std::string handleIOFaultInformationRelay(uint8_t data) {
    return (data & (1 << 2)) ? "Closed" : "Open";
}

std::string handleIOInputFan(uint8_t data) {
    return (data & (1 << 3)) ? "Off" : "On";
}

std::string handleIOPreheating(uint8_t data) {
    return (data & (1 << 4)) ? "On" : "Off";
}

std::string handleIOExhaustFan(uint8_t data) {
    return (data & (1 << 5)) ? "Off" : "On";
}

std::string handleIOFireplaceBoostSwitch(uint8_t data) {
    return (data & (1 << 6)) ? "Closed" : "Open";
}

std::string handleCurrentFanSpeed(uint8_t data) {
    return "Speed " + std::to_string(countSetBits(data));
}

std::string handleMaxCurrentMeasuredMoistureContent(uint8_t data) {
    return std::to_string((data - 51) / 2.04) + "% RH";
}

std::string handleCurrentMeasuredCO2ConcentrationUpper(uint8_t data) {
    return std::to_string(data) + " PPM";
}

std::string handleMaxCurrentMeasuredCO2ConcentrationLower(uint8_t data) {
    return std::to_string(data) + " PPM";
}

std::string handleMilliamperVoltageMessage(uint8_t data) {
    return (data & 1) ? "Installed" : "Not installed";
}

// Mapping von Funktionen
std::map<uint16_t, std::function<std::string(uint8_t)>> functionMap = {
    {0x0600, handleFanSpeed},
    {0x0700, handleIOReheat},
    {0x0800, handleIODamperMotorPosition},
    {0x0801, handleIOFaultInformationRelay},
    {0x0802, handleIOInputFan},
    {0x0803, handleIOPreheating},
    {0x0804, handleIOExhaustFan},
    {0x0805, handleIOFireplaceBoostSwitch},
    {0x2900, handleCurrentFanSpeed},
    {0x2A00, handleMaxCurrentMeasuredMoistureContent},
    {0x2B00, handleCurrentMeasuredCO2ConcentrationUpper},
    {0x2C00, handleMaxCurrentMeasuredCO2ConcentrationLower},
    {0x2D00, handleMilliamperVoltageMessage},
    // Fügen Sie hier andere Funktionen hinzu...
};

std::string findInfo(uint8_t packetNumber, uint8_t data, SearchType type) {
    for (const auto& info : variableInfos) {
        if (info.variable == packetNumber) {
            if (type == SearchType::VariableName) {
                return info.variableName;
            } else if (type == SearchType::DataDefinition) {
                uint16_t key = (packetNumber << 8) | info.Data_bitPosition;
                if (functionMap.count(key)) {
                    return functionMap[key](data);
                } else if (info.Data_hexValue == data) {
                    return info.Data_Definition;
                }
            }
        }
    }
    return (type == SearchType::VariableName) ? "" : std::to_string(data);
}
```

In diesem Code wird ein `std::map` namens `functionMap` erstellt, das Funktionen speichert, die aufgerufen werden sollen, basierend auf der Kombination von Variablen- und Datenbyte. In der `findInfo` Funktion wird dann die entsprechende Funktion aus `functionMap` aufgerufen, wenn die Kombination von Variablen- und Datenbyte im Mapping gefunden wird.