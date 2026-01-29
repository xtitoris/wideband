#include "can.h"
#include "hal.h"

#include "can_motec.h"

#include "util/byteswap.h"
#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "../for_rusefi/wideband_can.h"

void SendMotecAfrFormat(Configuration* configuration, uint8_t ch)
{
}

void SendMotecEgtFormat(Configuration* configuration, uint8_t ch)
{
}

void ProcessMotecCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData)
{
}