#pragma once

#include "hal.h"
#include <cstdint>
#include "byteswap.h"
#include "port.h"

void SendAemNetUEGOFormat(Configuration* cfg, uint8_t ch);
void SendAemNetEGTFormat(Configuration* cfg, uint8_t ch);
