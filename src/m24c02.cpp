#include "m24c02.h"
#include "log_codes.h"

// List of PCB versions this firmware is compatible with.
// Add entries here as the PCB evolves without requiring firmware changes.
static const PcbVersion COMPATIBLE_VERSIONS[] = {
    {1, 0, 0}, // TODO. Test when we have a newer version of the PCB.
};

bool M24C02::begin() {
    PcbVersion dummy;
    return readPcbVersion(&dummy);
}

bool M24C02::readPcbVersion(PcbVersion *version) {
    uint8_t buf[3];
    if (!i2c_.read(M24C02_ADDRESS, M24C02_VERSION_ADDR, buf, 3)) {
        return false;
    }
    version->major = buf[0];
    version->minor = buf[1];
    version->patch = buf[2];
    return true;
}

bool M24C02::isCompatible(const PcbVersion &version) {
    for (uint8_t i = 0; i < sizeof(COMPATIBLE_VERSIONS) / sizeof(COMPATIBLE_VERSIONS[0]); i++) {
        const PcbVersion &v = COMPATIBLE_VERSIONS[i];
        if (version.major == v.major && version.minor == v.minor && version.patch == v.patch) {
            return true;
        }
    }
    return false;
}
