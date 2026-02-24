#pragma once

#include "../can_base.h"

#if WB_PROD

#include "hal.h"

const CANConfig& GetCanConfig(CanBaudRate baudRate);

#endif // WB_PROD
