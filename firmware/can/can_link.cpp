#include "can.h"
#include "hal.h"

#include "can_link.h"

#include "util/byteswap.h"
#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "../for_rusefi/wideband_can.h"

// Link ECU protocol

#define LINKECU_L2C_TX_PERIOD_MS    10
#define LINKECU_L2C_BASE_ID         0x3B6

#define LINKECU_L2C_IN_BASE_ID      0x3BE
#define LINKECU_L2C_SET_IDX_ID      0x3BC

namespace linkecu
{

// 29 bit ID, 1000kbps default, rate 100 hz, endian big, DLC 8
// ID: 0x3B6..0x3BD for AFR channels
struct LinkEcuAfrData1
{
    uint8_t FrameIndex = 50; // Fixed value for AFR frames

    uint8_t ErrorCodes;
    beuint16_t Lambda;      // 0.001 Lambda/bit, 0.000 to 65.535 Lambda
    beuint16_t SensorTemp;  // 0-65535 C; 1/1

    // 0 - Off
    // 1 - Disabled
    // 2 - Initializing
    // 3 - Diagnostic
    // 4 - Calibration
    // 5 - Heating
    // 6 - Operating
    uint8_t Status;

    uint8_t Reserved;
} __attribute__((packed));

static_assert(sizeof(LinkEcuAfrData1) == 8);

struct LinkEcuAfrData2
{
    uint8_t FrameIndex = 51; // Fixed value for AFR frames
    uint8_t Reserved;

    beuint16_t IpCurrent;       // -32.768 to 32.767 mA; 1/1000
    beuint16_t SystemVoltage;   // 0.00-655.35 V; 1/100
    beuint16_t HeaterVoltage;   // 0.00-655.35 V; 1/100
} __attribute__((packed));

static_assert(sizeof(LinkEcuAfrData2) == 8);

} //namespace linkecu


static void SendAck(uint16_t id, uint8_t id_ok, uint8_t bus_freq_ok)
{
    CANTxFrame frame;

#ifdef STM32G4XX
    frame.common.RTR = 0;
#else // Not CAN FD
    frame.RTR = CAN_RTR_DATA;
#endif

    frame.DLC = 8;

    CAN_EXT(frame) = 1;
    CAN_EID(frame) = LINKECU_L2C_BASE_ID + id;

    frame.data8[0] = 24;
    frame.data8[1] = id_ok ? 0x01 : 0xFF;
    frame.data8[2] = bus_freq_ok ? 0x01 : 0xFF;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);
}

void SendLinkAfrFormat(Configuration* configuration, uint8_t ch)
{
    if (configuration->afr[ch].EcuMasterTx) {
        auto id = LINKECU_L2C_BASE_ID + configuration->afr[ch].EcuMasterIdOffset * 2;
        const auto& sampler = GetSampler(ch);
        const auto& heater = GetHeaterController(ch);

        CanTxTyped<linkecu::LinkEcuAfrData1> frame(id, true);
        frame.get().Lambda = GetLambda(ch) * 1000;
        frame.get().SensorTemp = sampler.GetSensorTemperature();

        switch (heater.GetHeaterState()) {
            case HeaterState::Preheat:
            case HeaterState::WarmupRamp:
                frame.get().Status = 5; // Heating
                break;
            case HeaterState::ClosedLoop:
                frame.get().Status = 6; // Operating
                break;
            case HeaterState::Stopped:
            default:
                frame.get().Status = 1; // Disabled
                break;
        }
        frame.get().ErrorCodes = 0; // TODO:

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

        CanTxTyped<linkecu::LinkEcuAfrData2> frame2(id, true);
        frame2.get().IpCurrent = sampler.GetPumpNominalCurrent() * 1000;
        frame2.get().SystemVoltage = sampler.GetInternalHeaterVoltage() * 100;
        frame2.get().HeaterVoltage = heater.GetHeaterEffectiveVoltage() * 100;
    }
}

void SendLinkEgtFormat(Configuration* configuration, uint8_t ch)
{
}

void ProcessLinkCanMessage(const CANRxFrame* frame, Configuration* configuration, struct CanStatusData* statusData)
{
    // Ignore std frames, only listen to ext
    if (!CAN_EXT(*frame))
    {
        return;
    }

    auto id = CAN_ID(*frame);

    if (id == LINKECU_L2C_IN_BASE_ID && frame->DLC == 8 && (frame->data8[0]) == 85)
    {
        // This is status from ECU
        // - Engine speed
        // - Exhaust absolute pressure
        // - Use exhaust pressure

        uint16_t engineRpm = (static_cast<uint16_t>(frame->data8[2]) << 8) | frame->data8[3];
        // Emulate LinkEcu CanLambda
        if (engineRpm > 400)
        {
            statusData->heaterAllow = HeaterAllow::Allowed;
        }
        else if (engineRpm < 10)
        {
            statusData->heaterAllow = HeaterAllow::NotAllowed;
        }

        // x10 kPa (eg 1100 = 110.0kPa)
        // uint16_t exhaustPressure = (static_cast<uint16_t>(frame->data8[4]) << 8) | frame->data8[5];
        // uint8_t useExhaustPressure = frame->data8[6] != 0;
    }
    // Check if it's an "index set" message
    else if (id >= LINKECU_L2C_SET_IDX_ID && id <= LINKECU_L2C_SET_IDX_ID + 7 && frame->DLC == 8 && (frame->data8[0] == 24))
    {
        for (int i = 0; i < AFR_CHANNELS; i++) {
            if (id - configuration->afr[i].LinkIdOffset == LINKECU_L2C_SET_IDX_ID) {
                uint8_t offset = frame->data8[1] & 0x0F;

                // 0 = 100 kbit/s
                // 1 = 125 kbit/s
                // 2 = 250 kbit/s
                // 3 = 500 kbit/s
                // 4 = 1 Mbit/s (default)
                uint8_t bus_freq = frame->data8[1];

                configuration->afr[i].LinkIdOffset = offset;
                SetConfiguration();
                SendAck(id, 1, 0); // id_ok = 1, bus_freq_ok not implemented yet
            }
        }
    }
}