#pragma once

#include <cstdint>

#include "hal.h"
#include "port.h"
#include "byteswap.h"

void SendAemNetUEGOFormat(Configuration* cfg, uint8_t ch);
void SendAemNetEGTFormat(Configuration* cfg, uint8_t ch);

void ProcessAemNetCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData);