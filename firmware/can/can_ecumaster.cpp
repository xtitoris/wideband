#include "can.h"
#include "hal.h"

#include "util/byteswap.h"
#include "can_helper.h"
#include "can_ecumaster.h"

#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "../for_rusefi/wideband_can.h"

// EcuMaster protocol

#define ECUMASTER_L2C_TX_PERIOD_MS    10
#define ECUMASTER_L2C_BASE_ID         0x664

namespace ecumaster
{

struct EcuMasterAfrData1
{
    beuint16_t SystemVolts;     // 0.00-655.35 V; 1/100
    uint8_t HeaterPower;        // 0-100 %; 100/255
    uint8_t SensorTemp;         // 0-1020 C; 4/1
    beuint16_t Lambda;  // 0.001 Lambda/bit, 0.000 to 65.535 Lambda
    // [1] - vm short vcc
    // [2] - vm short gnd
    // [3] - un short vcc
    // [4] - un short gnd
    // [5] - iaip short vcc
    // [6] - iaip short gnd
    // [7] - vub low voltage
    // [8] - heater short vcc
    uint8_t Flags1;
    // [9]  - heater short gnd
    // [10] - heater open load
    // [11:13] - calibration state
    // [14:16] - device version
    uint8_t Flags2;
} __attribute__((packed));

static_assert(sizeof(EcuMasterAfrData1) == 8);

struct EcuMasterAfrData2
{
    beint16_t IpCurrent;               // -32.768 to 32.767 mA; 1/1000
    beint16_t OxygenConcentration;     // -327.68 to 327.67 %; 1/100
    beuint16_t Ri;                     //  0.0 to 6553.5 Ohm; 1/10
    uint8_t Reserved[2];
} __attribute__((packed));

static_assert(sizeof(EcuMasterAfrData2) == 8);

} //namespace ecumaster


void SendEcuMasterAfrFormat(Configuration* configuration, uint8_t ch)
{
    if (configuration->afr[ch].EcuMasterTx) {
        auto id = ECUMASTER_L2C_BASE_ID + configuration->afr[ch].EcuMasterIdOffset * 2;
        const auto& sampler = GetSampler(ch);

        CanTxTyped<ecumaster::EcuMasterAfrData1> frame(id, true);
        frame.get().SystemVolts = sampler.GetInternalHeaterVoltage() * 10;
        frame.get().HeaterPower = 0;
        frame.get().SensorTemp = sampler.GetSensorTemperature() / 4;
        frame.get().Lambda = GetLambda(ch) * 1000;

        frame.get().Flags1 = 0; // TODO:

        uint8_t deviceType = 0;
        switch (configuration->sensorType) {
            case SensorType::LSU42:
                deviceType = 0;
                break;
            case SensorType::LSU49:
                deviceType = 1;
                break;
            case SensorType::LSUADV:
                deviceType = 2;
                break;
        }

        frame.get().Flags2 = 0
            | 0x02 << 2 // calibration state = Finished
            | deviceType << 5;

        CanTxTyped<ecumaster::EcuMasterAfrData2> frame2(id + 1, true);
        frame2.get().IpCurrent = sampler.GetPumpNominalCurrent() * 1000;
        frame2.get().OxygenConcentration = 0; // TODO:
        frame2.get().Ri = sampler.GetSensorInternalResistance() * 10;
    }
}

void SendEcuMasterEgtFormat(Configuration* configuration, uint8_t ch)
{
}

void ProcessEcuMasterCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData)
{
}