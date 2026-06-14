#pragma once

#include <cstdint>

/*
 * This file defines base hardware-independent types for CAN communication.
 */

// Can speeds
enum class CanBaudRate : uint8_t {
    Baud500Kbps = 0, // Default value
    Baud1Mbps = 1,   // Next most common speed
    Baud125Kbps = 2,
    Baud250Kbps = 3,
};

// These values are kept in sync just for convenience
enum class CanAfrProtocol : uint8_t {
    None = 0,
    AemNet = 1,
    LinkEcu = 2,
    Haltech = 3,
    EcuMaster = 4,
    Motec = 6,
    Emtron = 7,
};

enum class CanEgtProtocol : uint8_t {
    None = 0,
    AemNet0305 = 1,
    LinkEcu = 2,
    Haltech = 3,
    EcuMasterClassic = 4,
    EcuMasterBlack = 5,
    Motec = 6,
    Emtron = 7,
    AemNet2224 = 8,
};

enum class CanIoProtocol : uint8_t {
    None = 0,
    Haltech = 3,
    EcuMaster = 4,
    Motec = 6,
    Emtron = 7,
    MsIoBox = 9,
};