#include "can.h"
#include "hal.h"

#include "can_haltech.h"

#include "util/byteswap.h"
#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "../for_rusefi/wideband_can.h"

// Haltech protocol

#define HALTECH_L2C_BASE_ID         0x2B0

namespace haltech
{

// 29 bit ID, 1000kbps default, rate 100 hz, endian big, DLC 8
// ID: WB2A 0x2B0
// ID: WB2B 0x2B4
// ID: WB2C 0x2B6
// ID: WB2D 0x2B8
struct HaltechAfrData1
{
    beuint16_t Lambda1;        // 32767 = Free Air, otherwise y = x/1024
    beuint16_t Lambda2;        // 0.001 Lambda/bit, 0.000 to 65.535 Lambda
    uint8_t RSense1;           // 0-255 Ohm; 1/1
    uint8_t RSense2;           // 0-255 Ohm; 1/1
    uint8_t Flags;             // 1 = Low Battery (< 9V)
                               // 2 = High Battery (> 18V)
                               // 3 = Sensor Short Circuit
                               // 4 = Sensor Open Circuit
                               // 5 = Sensor Cold
                               // 0:3 Sensor 1
                               // 4:7 Sensor 2
    uint8_t VBatt;             // y = x * 20 / 255 Volts
} __attribute__((packed));

static_assert(sizeof(HaltechAfrData1) == 8);

} //namespace haltech


void SendHaltechAfrFormat(Configuration* configuration, uint8_t ch)
{
    if (ch % 2 != 0)
        return; // Haltech protocol sends both channels in one message

    if (configuration->afr[ch].HaltechTx) {
        auto id = HALTECH_L2C_BASE_ID; // WB2A
        switch (configuration->afr[ch].HaltechIdOffset) {
            case 1:
                id += 4; break; // WB2B
            case 2:
                id += 6; break; // WB2C
            case 3:
                id += 8; break; // WB2D
        }

        const auto& sampler1 = GetSampler(ch);
        const auto& sampler2 = GetSampler(ch+1);

        CanTxTyped<haltech::HaltechAfrData1> frame(id, true);
        frame.get().VBatt = sampler1.GetInternalHeaterVoltage() * 255.0 / 20.0f;
        frame.get().Lambda1 = GetLambda(ch) * 1024;
        frame.get().Lambda2 = GetLambda(ch + 1) * 1024;
        frame.get().RSense1 = sampler1.GetSensorInternalResistance();
        frame.get().RSense2 = sampler2.GetSensorInternalResistance();

        // TODO: set flags
        frame.get().Flags = 0;
    }
}

void SendHaltechEgtFormat(Configuration* configuration, uint8_t ch)
{
}

void ProcessHaltechCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData)
{
}
