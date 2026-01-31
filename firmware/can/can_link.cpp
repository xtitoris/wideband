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
#include "max3185x.h"
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


struct LinkEcuEgtData1
{
    beint16_t Egt[4];    // 0 to 1250 C; 1/4
} __attribute__((packed));

static_assert(sizeof(LinkEcuEgtData1) == 8);


struct LinkEcuEgtData3
{
    beint16_t Egt[2];    // 0 to 1250 C; 1/4
    uint16_t Reserved;
    uint8_t SupplyVolt; // 0.0 to 25.5 V; 1/10
    uint8_t ChipTemp;   // 0 to 125 C; 1/1
} __attribute__((packed));

static_assert(sizeof(LinkEcuEgtData3) == 8);

struct LinkEcuEgtStatus
{
    uint8_t TcStatus1: 4; // 0=OK, 1=short to VCC, 2=Short to GND, 3=not connnected, 4=unknown error, 5=chip missing
    uint8_t TcStatus2: 4;
    uint8_t TcStatus3: 4;
    uint8_t TcStatus4: 4;
    uint8_t TcStatus5: 4;
    uint8_t TcStatus6: 4;
    uint8_t TcStatus7: 4;
    uint8_t TcStatus8: 4;
    uint8_t TcStatus9: 4;
    uint8_t TcStatus10: 4;
    uint8_t Reserved[3];
} __attribute__((packed));

static_assert(sizeof(LinkEcuEgtStatus) == 8);


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
    auto id = LINKECU_L2C_BASE_ID + configuration->afr[ch].ExtraCanIdOffset * 2;
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

#if (EGT_CHANNELS > 0)

#define LINKECU_TCCXX_BASE_ID         0x705
#define LINKECU_TCCXX_DATA_3_ID       0x705
#define LINKECU_TCCXX_STATUS_ID       0x708

void SendLinkEgtFormat(Configuration* configuration, uint8_t ch)
{
    if (ch != 0)
        return; // Link ECU protocol sends 1-4 channels in one message

    const auto& sampler = GetSampler(ch);
    const auto egtDrivers = getEgtDrivers();

    CanTxTyped<linkecu::LinkEcuEgtData1> frame(LINKECU_TCCXX_BASE_ID, true);
    frame.get().Egt[0] = egtDrivers[0].temperature / 4;

    if (EGT_CHANNELS > 1) {
        frame.get().Egt[1] = egtDrivers[1].temperature / 4;
    }
    if (EGT_CHANNELS > 2) {
        frame.get().Egt[2] = egtDrivers[2].temperature / 4;
    }
    if (EGT_CHANNELS > 3) {
        frame.get().Egt[3] = egtDrivers[3].temperature / 4;
    }

    CanTxTyped<linkecu::LinkEcuEgtData3> frame2(LINKECU_TCCXX_DATA_3_ID, true);
    frame2.get().SupplyVolt = sampler.GetInternalHeaterVoltage() / 10;
    frame2.get().ChipTemp = GetMcuTemperature();

    CanTxTyped<linkecu::LinkEcuEgtStatus> frame3(LINKECU_TCCXX_STATUS_ID, true);
    // TODO: set actual status
}

#endif

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
            if (id - configuration->afr[i].ExtraCanIdOffset == LINKECU_L2C_SET_IDX_ID) {
                uint8_t offset = frame->data8[1] & 0x0F;

                // 0 = 100 kbit/s
                // 1 = 125 kbit/s
                // 2 = 250 kbit/s
                // 3 = 500 kbit/s
                // 4 = 1 Mbit/s (default)
                uint8_t bus_freq = frame->data8[1];

                configuration->afr[i].ExtraCanIdOffset = offset;
                SetConfiguration();
                SendAck(id, 1, 0); // id_ok = 1, bus_freq_ok not implemented yet
            }
        }
    }
}