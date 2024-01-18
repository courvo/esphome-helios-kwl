#include "VariableHandler.h"

std::string handleFanSpeed(uint8_t data) {
    // Implementierung...
}

std::string handleIOReheat(uint8_t data) {
    // Implementierung...
}

// ... Weitere Funktionen ...

std::map<uint16_t, std::function<std::string(uint8_t)>> functionMap = {
    {0x0600, handleFanSpeed},
    {0x0700, handleIOReheat},
    // ... Weitere Mappings ...
};