#include "can.h"
#include "hal.h"

#include "can_rusefi.h"

#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "../for_rusefi/wideband_can.h"

void SendRusefiFormat(Configuration* configuration, uint8_t ch)
{
    auto baseAddress = WB_DATA_BASE_ADDR + 2 * configuration->afr[ch].RusEfiIdx;

    const auto& sampler = GetSampler(ch);
    const auto& heater = GetHeaterController(ch);

    auto nernstDc = sampler.GetNernstDc();
    auto pumpDuty = GetPumpOutputDuty(ch);
    auto lambda = GetLambda(ch);

    // Lambda is valid if:
    // 1. Nernst voltage is near target
    // 2. Lambda is >0.6 (sensor isn't specified below that)
    bool lambdaValid =
            LambdaIsValid(ch) &&
            lambda > 0.6f;

    if (configuration->afr[ch].RusEfiTx) {
        CanTxTyped<wbo::StandardData> frame(baseAddress + 0);

        // The same header is imported by the ECU and checked against this data in the frame
        frame.get().Version = RUSEFI_WIDEBAND_VERSION;

        uint16_t lambdaInt = lambdaValid ? (lambda * 10000) : 0;
        frame.get().Lambda = lambdaInt;
        frame.get().TemperatureC = sampler.GetSensorTemperature();
        bool heaterClosedLoop = heater.IsRunningClosedLoop();
        frame.get().Valid = (heaterClosedLoop && lambdaValid) ? 0x01 : 0x00;
    }

    if (configuration->afr[ch].RusEfiTxDiag) {
        CanTxTyped<wbo::DiagData> frame(baseAddress + 1);;

        frame.get().Esr = sampler.GetSensorInternalResistance();
        frame.get().NernstDc = nernstDc * 1000;
        frame.get().PumpDuty = pumpDuty * 255;
        frame.get().status = GetCurrentStatus(ch);
        frame.get().HeaterDuty = GetHeaterDuty(ch) * 255;
    }
}