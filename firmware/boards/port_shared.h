#pragma once

#if WB_PROD

#include "hal.h"
#include "../can_base.h"

const CANConfig& GetCanConfig(CanBaudRate baudRate);

#endif // WB_PROD
