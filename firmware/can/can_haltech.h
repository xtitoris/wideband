#pragma once

#include <cstdint>

#include "hal.h"
#include "port.h"

void SendHaltechAfrFormat(Configuration* configuration, uint8_t ch);
void SendHaltechEgtFormat(Configuration* configuration, uint8_t ch);

void ProcessHaltechCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData);
