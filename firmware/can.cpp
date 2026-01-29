#include "hal.h"

#include "can.h"

#include "status.h"
#include "can_helper.h"
#include "can/can_rusefi.h"
#include "can/can_aemnet.h"
#include "can/can_motec.h"
#include "can/can_ecumaster.h"
#include "can/can_haltech.h"
#include "can/can_link.h"

#include "heater_control.h"
#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "port.h"
#include "pump_control.h"

#include <rusefi/math.h>

// this same header is imported by rusEFI to get struct layouts and firmware version
#include "../for_rusefi/wideband_can.h"

static Configuration* configuration;

static THD_WORKING_AREA(waCanTxThread, 512);
void CanTxThread(void*)
{
    int cycle = 0;
    chRegSetThreadName("CAN Tx");

    // Current system time.
    systime_t prev = chVTGetSystemTime();

    while(1)
    {
        // AFR - 100 Hz
        for (int ch = 0; ch < AFR_CHANNELS; ch++)
        {
            SendCanForChannel(ch);
        }

        // EGT - 20 Hz
        if ((cycle % 5) == 0) {
            for (int ch = 0; ch < EGT_CHANNELS; ch++) {
                SendCanEgtForChannel(ch);
            }
        }

        cycle++;
        prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(WBO_TX_PERIOD_MS)));
    }
}

static void SendAck()
{
    CANTxFrame frame;

#ifdef STM32G4XX
    frame.common.RTR = 0;
#else // Not CAN FD
    frame.RTR = CAN_RTR_DATA;
#endif

    CAN_EXT(frame) = 1;
    CAN_EID(frame) = WB_ACK;
    frame.DLC = 0;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);
}

// Start in Unknown state. If no CAN message is ever received, we operate
// on internal battery sense etc.
static HeaterAllow heaterAllow = HeaterAllow::Unknown;
static float remoteBatteryVoltage = 0;

static THD_WORKING_AREA(waCanRxThread, 512);
void CanRxThread(void*)
{
    chRegSetThreadName("CAN Rx");

    while(1)
    {
        CANRxFrame frame;
        msg_t msg = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);

        // Ignore non-ok results...
        if (msg != MSG_OK)
        {
            continue;
        }

        // Ignore std frames, only listen to ext
        if (!CAN_EXT(frame))
        {
            continue;
        }

        // Ignore not ours frames
        if (WB_MSG_GET_HEADER(CAN_ID(frame)) != WB_BL_HEADER)
        {
            continue;
        }

        if (frame.DLC >= 2 && CAN_ID(frame) == WB_MSG_ECU_STATUS)
        {
            // This is status from ECU
            // - battery voltage
            // - heater enable signal
            // - optionally pump control gain

            // data1 contains heater enable bit
            if ((frame.data8[1] & 0x1) == 0x1)
            {
                heaterAllow = HeaterAllow::Allowed;
            }
            else
            {
                heaterAllow = HeaterAllow::NotAllowed;
            }

            // data0 contains battery voltage in tenths of a volt
            float vbatt = frame.data8[0] * 0.1f;
            if (vbatt < 5)
            {
                // provided vbatt is bogus, default to 14v nominal
                remoteBatteryVoltage = 14;
            }
            else
            {
                remoteBatteryVoltage = vbatt;
            }

            if (frame.DLC >= 3) {
                // data2 contains pump controller gain in percent (0-200)
                float pumpGain = frame.data8[2] * 0.01f;
                SetPumpGainAdjust(clampF(0, pumpGain, 1));
            }
        }
        // If it's a bootloader entry request, reboot to the bootloader!
        else if ((frame.DLC == 0 || frame.DLC == 1) && CAN_ID(frame) == WB_BL_ENTER)
        {
            // If 0xFF (force update all) or our ID, reset to bootloader, otherwise ignore
            if (frame.DLC == 0 || frame.data8[0] == 0xFF || frame.data8[0] == GetConfiguration()->afr[0].RusEfiIdx)
            {
                SendAck();

                // Let the message get out before we reset the chip
                chThdSleep(50);

                NVIC_SystemReset();
            }
        }
        // Check if it's an "index set" message
        else if (frame.DLC == 1 && CAN_ID(frame) == WB_MSG_SET_INDEX)
        {
            int offset = frame.data8[0];
            configuration = GetConfiguration();
            for (int i = 0; i < AFR_CHANNELS; i++) {
                configuration->afr[i].RusEfiIdx = offset + i;
            }
            for (int i = 0; i < EGT_CHANNELS; i++) {
                configuration->egt[i].RusEfiIdx = offset + i;
            }
            SetConfiguration();
            SendAck();
        }
    }
}

HeaterAllow GetHeaterAllowed()
{
    return heaterAllow;
}

float GetRemoteBatteryVoltage()
{
    return remoteBatteryVoltage;
}

void InitCan()
{
    configuration = GetConfiguration();

    canStart(&CAND1, &GetCanConfig(configuration->CanMode));
    chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO, CanTxThread, nullptr);
    chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO - 4, CanRxThread, nullptr);
}


// Weak link so boards can override it
__attribute__((weak)) void SendCanForChannel(uint8_t ch)
{
    SendRusefiFormat(configuration, ch);
    SendAemNetUEGOFormat(configuration, ch);
    SendEcuMasterAfrFormat(configuration, ch);
    SendHaltechAfrFormat(configuration, ch);
    SendLinkAfrFormat(configuration, ch);
    SendMotecAfrFormat(configuration, ch);
}

__attribute__((weak)) void SendCanEgtForChannel(uint8_t ch)
{
#if (EGT_CHANNELS > 0)
    // TODO: implement RusEFI protocol?
    SendAemNetEGTFormat(configuration, ch);
#endif
}
