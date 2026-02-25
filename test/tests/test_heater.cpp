#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "fixed_point.h"
#include "heater_control.h"
#include "status.h"

/*
Heater state-machine test traceability matrix

STATE: Stopped
- Hold checks:
    - HeaterState_Stopped_Checks.InitialStateIsStopped
    - HeaterState_Stopped_Checks.NotAllowedStaysStopped
    - HeaterState_Stopped_Checks.NothingChangesStaysStopped
- Entry checks:
    - HeaterState_Stopped_Checks.UnknownBeforeStabilityThresholdStaysStopped_AfterThresholdStartsPreheat
    - HeaterState_Stopped_Checks.UnknownJitterAroundOnVoltageResetsStabilityTimer
    - HeaterState_Stopped_Checks.AllowedBypassesOnVoltageGate
- Retry timer checks:
    - HeaterState_Stopped_Checks.RetryBlocksRestartAfterUndervoltageFault
    - HeaterState_Stopped_Checks.RetryBlocksRestartAfterWarmupTimeoutFault
    - HeaterState_Stopped_Checks.RetryBlocksRestartAfterUnderheatFault
    - HeaterState_Stopped_Checks.RetryBlocksRestartAfterOverheatFault

STATE: Preheat
- Hold checks:
    - HeaterState_Preheat_Checks.StaysPreheatIfNoExitCondition
    - HeaterState_Preheat_Checks.BoundaryEqualityClosedLoopTempStaysPreheat
- Transition checks:
    - To WarmupRamp: HeaterState_Preheat_Transitions.BeforePreheatTimeoutStaysPreheat_AfterTimeoutToWarmup, .ToWarmupByTemperature
    - To Stopped: HeaterState_Preheat_Transitions.ToStoppedByOvervoltage, .ToStoppedByUndervoltageDelay

STATE: WarmupRamp
- Hold checks:
    - HeaterState_Warmup_Checks.StaysWarmupIfNoExitCondition
- Transition checks:
    - To ClosedLoop: HeaterState_Warmup_Transitions.ToClosedLoopByTemperature
    - To Stopped: HeaterState_Warmup_Transitions.AtWarmupTimeoutStaysWarmup_AfterTimeoutStops
    - Fault exits: HeaterState_Warmup_Transitions.ToStoppedByOvervoltage, .ToStoppedByUndervoltageDelay

STATE: ClosedLoop
- Hold checks:
    - HeaterState_ClosedLoop_Checks.StaysClosedLoopAtNominalTemperature
    - HeaterState_ClosedLoop_Checks.OverheatShortPulseDoesNotTrip
    - HeaterState_ClosedLoop_Checks.UnderheatIgnoredDuringClosedLoopStabilization
    - HeaterState_ClosedLoop_Checks.RecoveryToNormalTempResetsFaultTimers
- Transition checks:
    - To Stopped: HeaterState_ClosedLoop_Transitions.ToStoppedByOverheat
    - To Stopped: HeaterState_ClosedLoop_Transitions.ToStoppedByUnderheatAfterStabilization
    - Fault exits: HeaterState_ClosedLoop_Transitions.ToStoppedByOvervoltage, .ToStoppedByUndervoltageDelay

THRESHOLD / BOUNDARY checks
- HeaterThreshold_Checks.OvervoltageEqualityIsNotFault
- HeaterThreshold_Checks.OnVoltageEqualityActsAsStable
- HeaterThreshold_Checks.OffVoltageEqualityCountsAsLow

OUTPUT / API checks
- Output: HeaterOutput_Preheat_Checks.*, HeaterOutput_WarmupRamp_Checks.*, HeaterOutput_ClosedLoop_Checks.*, HeaterOutput_Stopped_Checks.*
- API: HeaterApi_Checks.DescribeHeaterState, HeaterApi_Checks.TargetTemperatureGetter

TIMER RESET checks
- HeaterTimerReset_Checks.StoppedToPreheat_ResetsPreheatTimer
- HeaterTimerReset_Checks.PreheatToWarmup_ResetsWarmupTimer
- HeaterTimerReset_Checks.WarmupToClosedLoop_ResetsClosedLoopStabilityTimer

STATUS checks
- HeaterStatus_Checks.PreheatTransitionSetsPreheatStatus
- HeaterStatus_Checks.WarmupTransitionSetsWarmupStatus
- HeaterStatus_Checks.ClosedLoopTransitionSetsRunningStatus
- HeaterStatus_Checks.FaultTransitionsSetFaultStatus
- HeaterStatus_Checks.HasFaultReflectsFaultState
*/

namespace {

constexpr float kTargetTempC = 780.0f;
constexpr float kTargetEsr = 300.0f;
constexpr float kGoodVoltage = 12.0f;
constexpr float kVoltageDelta = 0.1f;
constexpr float kSafeHighVoltage = 14.0f;
constexpr float kLowVoltage = 5.5f;
constexpr float kOffVoltageEq = 6.0f;
constexpr float kOnVoltageEq = 11.0f;
constexpr float kOvervoltageThreshold = 23.0f;
constexpr float kOvervoltageFaultVoltage = kOvervoltageThreshold + 1.0f;
constexpr float kAllowedBypassVoltage = 7.0f;
constexpr float kBelowOnVoltage = kOnVoltageEq - kVoltageDelta;
constexpr float kUnknownNoStartVoltage = 9.0f;

constexpr float kMarginSec = 0.1f;
constexpr float kUndervoltFaultDelaySec = 0.5f;
constexpr float kOverUnderheatFaultDelaySec = 0.5f;
constexpr float kVoltageStabilityDelaySec = HEATER_VOLTAGE_STAB_TIME;
constexpr float kWarmupTimeoutSec = HEATER_WARMUP_TIMEOUT;
constexpr float kClosedLoopStabDelaySec = HEATER_CLOSED_LOOP_STAB_TIME;
constexpr float kUndervoltRetrySec = HEATER_UNDERVOLT_RETRY_TIMEOUT;
constexpr float kDidNotHeatRetrySec = HEATER_DIDNOTHEAT_RETRY_TIMEOUT;
constexpr float kUnderheatRetrySec = HEATER_UNDERHEAT_RETRY_TIMEOUT;
constexpr float kOverheatRetrySec = HEATER_OVERHEAT_RETRY_TIMEOUT;
constexpr float kMockPreheatTimeoutSec = 5.0f; // Mock config: PreheatTimeSec = 1 => 5s

constexpr int64_t toUs(float seconds) {
    return static_cast<int64_t>(seconds * 1'000'000.0f);
}

struct MockConfiguration {
    struct HeaterConfig heaterConfig {
        .HeaterSupplyOffVoltage = { 60 }, // 6.0V
        .HeaterSupplyOnVoltage = { 110 }, // 11.0V
        .PreheatTimeSec = { 1 }, // 5 seconds
        .pad = {0},
    };
} mockConfig;

struct MockHeater : public HeaterControllerBase {
    MockHeater() : HeaterControllerBase(0) {}

    MOCK_METHOD(void, SetDuty, (float), (const, override));
};

void configureDut(MockHeater& dut) {
    Timer::setMockTime(toUs(0.0f));
    dut.Configure(kTargetTempC, kTargetEsr, &mockConfig.heaterConfig);
    SetStatus(0, wbo::Status::Preheat);
}

// Prime undervoltage timer with known-good voltage before testing UV delay behavior.
void primeUndervoltTimer(MockHeater& dut) {
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

} // namespace

// ====================
// Output: per-state
// ====================

TEST(HeaterOutput_Preheat_Checks, ConstantVoltage)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(2.0f, dut.GetVoltageForState(HeaterState::Preheat, 0));
    EXPECT_EQ(2.0f, dut.GetVoltageForState(HeaterState::Preheat, 300));
    EXPECT_EQ(2.0f, dut.GetVoltageForState(HeaterState::Preheat, 1000));
}

TEST(HeaterOutput_WarmupRamp_Checks, MonotonicIncreaseAndClamp)
{
    MockHeater dut;
    configureDut(dut);

    float last = dut.GetVoltageForState(HeaterState::WarmupRamp, 300);
    EXPECT_GT(last, 0.0f);

    for (int i = 0; i < 1000; i++) {
        float next = dut.GetVoltageForState(HeaterState::WarmupRamp, 300);
        EXPECT_GE(next, last);
        EXPECT_LE(next, 12.0f);
        last = next;
    }

    EXPECT_NEAR(12.0f, last, 0.05f);
}

TEST(HeaterOutput_ClosedLoop_Checks, DirectionalityAroundTargetEsr)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(7.5f, dut.GetVoltageForState(HeaterState::ClosedLoop, 300));
    EXPECT_GT(dut.GetVoltageForState(HeaterState::ClosedLoop, 400), 7.5f);
    EXPECT_LT(dut.GetVoltageForState(HeaterState::ClosedLoop, 200), 7.5f);
}

TEST(HeaterOutput_ClosedLoop_Checks, MaxVoltageClamp)
{
    MockHeater dut;
    configureDut(dut);

    // Huge ESR error should command high voltage, then clamp to MAX_HEATER_VOLTAGE.
    EXPECT_EQ(12.0f, dut.GetVoltageForState(HeaterState::ClosedLoop, 10000.0f));
}

TEST(HeaterOutput_Stopped_Checks, ZeroVoltage)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(0.0f, dut.GetVoltageForState(HeaterState::Stopped, 0));
    EXPECT_EQ(0.0f, dut.GetVoltageForState(HeaterState::Stopped, 300));
    EXPECT_EQ(0.0f, dut.GetVoltageForState(HeaterState::Stopped, 1000));
}

// ====================
// State: Stopped
// ====================

TEST(HeaterState_Stopped_Checks, InitialStateIsStopped)
{
    MockHeater dut;
    configureDut(dut);
    EXPECT_EQ(HeaterState::Stopped, dut.GetHeaterState());
}

TEST(HeaterState_Stopped_Checks, NotAllowed_StaysStopped)
{
    MockHeater dut;
    configureDut(dut);
    Timer::setMockTime(toUs(0.0f));

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::NotAllowed, kSafeHighVoltage, 500.0f));

    Timer::advanceMockTime(toUs(5.0f));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::NotAllowed, kSafeHighVoltage, 500.0f));
}

TEST(HeaterState_Stopped_Checks, Unknown_BeforeStabilityThresholdStaysStopped_AfterThresholdStartsPreheat)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kVoltageStabilityDelaySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Stopped_Checks, Unknown_JitterAroundOnVoltageResetsStabilityTimer)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kVoltageStabilityDelaySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kBelowOnVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kVoltageStabilityDelaySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Stopped_Checks, Allowed_BypassesOnVoltageGate)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kAllowedBypassVoltage, 500.0f));
}

TEST(HeaterState_Stopped_Checks, NothingChanges_StaysStopped)
{
    MockHeater dut;
    configureDut(dut);

    for (int i = 0; i < 5; i++) {
        Timer::advanceMockTime(toUs(2 * kMarginSec));
        EXPECT_EQ(HeaterState::Stopped,
            dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kUnknownNoStartVoltage, 500.0f));
    }
}

TEST(HeaterState_Stopped_Checks, Retry_BlocksRestartAfterUndervoltageFault)
{
    MockHeater dut;
    configureDut(dut);

    primeUndervoltTimer(dut);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));

    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 700.0f));

    Timer::advanceMockTime(toUs(kUndervoltRetrySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 700.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 700.0f));
}

TEST(HeaterState_Stopped_Checks, Retry_BlocksRestartAfterWarmupTimeoutFault)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(kWarmupTimeoutSec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kDidNotHeatRetrySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Stopped_Checks, Retry_BlocksRestartAfterUnderheatFault)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    Timer::advanceMockTime(toUs(kClosedLoopStabDelaySec + 1.0f));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(kUnderheatRetrySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 600.0f));
}

TEST(HeaterState_Stopped_Checks, Retry_BlocksRestartAfterOverheatFault)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    Timer::advanceMockTime(toUs(kOverheatRetrySec - kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));
}

// ====================
// State: Preheat
// ====================

TEST(HeaterState_Preheat_Checks, StaysPreheatIfNoExitCondition)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(1.0f));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Preheat_Transitions, BeforePreheatTimeoutStaysPreheat_AfterTimeoutToWarmup)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(kMockPreheatTimeoutSec - kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::setMockTime(toUs(kMockPreheatTimeoutSec + kMarginSec));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Preheat_Transitions, ToWarmupByTemperature)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 780.0f));
}

TEST(HeaterState_Preheat_Checks, BoundaryEqualityClosedLoopTempStaysPreheat)
{
    MockHeater dut;
    configureDut(dut);

    // closedLoopTemp = target - 30 = 750. Equality should not transition.
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 750.0f));
}

TEST(HeaterState_Preheat_Transitions, ToStoppedByOvervoltage)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kOvervoltageFaultVoltage, 500.0f));
}

TEST(HeaterState_Preheat_Transitions, ToStoppedByUndervoltageDelay)
{
    MockHeater dut;
    configureDut(dut);

    primeUndervoltTimer(dut);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));

    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));
}

// ====================
// State: WarmupRamp
// ====================

TEST(HeaterState_Warmup_Checks, StaysWarmupIfNoExitCondition)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(1.0f));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Warmup_Transitions, ToClosedLoopByTemperature)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));
}

TEST(HeaterState_Warmup_Transitions, AtWarmupTimeoutStaysWarmup_AfterTimeoutStops)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(kWarmupTimeoutSec));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::setMockTime(toUs(kWarmupTimeoutSec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterState_Warmup_Transitions, ToStoppedByOvervoltage)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kOvervoltageFaultVoltage, 500.0f));
}

TEST(HeaterState_Warmup_Transitions, ToStoppedByUndervoltageDelay)
{
    MockHeater dut;
    configureDut(dut);

    primeUndervoltTimer(dut);

    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kLowVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kLowVoltage, 500.0f));
}

// ====================
// State: ClosedLoop
// ====================

TEST(HeaterState_ClosedLoop_Checks, StaysClosedLoopAtNominalTemperature)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));
}

TEST(HeaterState_ClosedLoop_Checks, OverheatShortPulseDoesNotTrip)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));
    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec - kMarginSec));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));
}

TEST(HeaterState_ClosedLoop_Transitions, ToStoppedByOverheat)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));
}

TEST(HeaterState_ClosedLoop_Checks, UnderheatIgnoredDuringClosedLoopStabilization)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    Timer::advanceMockTime(toUs(1.0f));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(1.0f));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));
}

TEST(HeaterState_ClosedLoop_Transitions, ToStoppedByUnderheatAfterStabilization)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    Timer::advanceMockTime(toUs(kClosedLoopStabDelaySec + 1.0f));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));
}

TEST(HeaterState_ClosedLoop_Checks, RecoveryToNormalTempResetsFaultTimers)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec - 2 * kMarginSec));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    Timer::advanceMockTime(toUs(kOverUnderheatFaultDelaySec - 2 * kMarginSec));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 1000.0f));
}

TEST(HeaterState_ClosedLoop_Transitions, ToStoppedByOvervoltage)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kOvervoltageFaultVoltage, 780.0f));
}

TEST(HeaterState_ClosedLoop_Transitions, ToStoppedByUndervoltageDelay)
{
    MockHeater dut;
    configureDut(dut);

    primeUndervoltTimer(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kLowVoltage, 780.0f));

    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kLowVoltage, 780.0f));
}

// ====================
// Threshold boundary checks
// ====================

TEST(HeaterThreshold_Checks, OvervoltageEqualityIsNotFault)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kOvervoltageThreshold, 500.0f));
}

TEST(HeaterThreshold_Checks, OnVoltageEqualityActsAsStable)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kOnVoltageEq, 500.0f));

    Timer::advanceMockTime(toUs(kVoltageStabilityDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Unknown, kOnVoltageEq, 500.0f));
}

TEST(HeaterThreshold_Checks, OffVoltageEqualityCountsAsLow)
{
    MockHeater dut;
    configureDut(dut);

    primeUndervoltTimer(dut);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kOffVoltageEq, 700.0f));

    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kOffVoltageEq, 700.0f));
}

// ====================
// Misc API checks
// ====================

TEST(HeaterApi_Checks, DescribeHeaterState)
{
    EXPECT_STREQ("Preheat", describeHeaterState(HeaterState::Preheat));
    EXPECT_STREQ("WarmupRamp", describeHeaterState(HeaterState::WarmupRamp));
    EXPECT_STREQ("ClosedLoop", describeHeaterState(HeaterState::ClosedLoop));
    EXPECT_STREQ("Stopped", describeHeaterState(HeaterState::Stopped));
}

TEST(HeaterApi_Checks, TargetTemperatureGetter)
{
    MockHeater dut;
    configureDut(dut);
    EXPECT_EQ(kTargetTempC, dut.GetTargetTemp());
}

// ====================
// Timer reset checks
// ====================

TEST(HeaterTimerReset_Checks, StoppedToPreheat_ResetsPreheatTimer)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(kMockPreheatTimeoutSec * 4));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    // If preheat timer wasn't reset on start, we'd jump to warmup immediately.
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kMockPreheatTimeoutSec - kMarginSec));
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterTimerReset_Checks, PreheatToWarmup_ResetsWarmupTimer)
{
    MockHeater dut;
    configureDut(dut);

    // Large time before transition - warmup timer must reset at transition point.
    Timer::setMockTime(toUs(kWarmupTimeoutSec * 3));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    // If not reset, this would instantly timeout.
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(kWarmupTimeoutSec - kMarginSec));
    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
}

TEST(HeaterTimerReset_Checks, WarmupToClosedLoop_ResetsClosedLoopStabilityTimer)
{
    MockHeater dut;
    configureDut(dut);

    Timer::setMockTime(toUs(kClosedLoopStabDelaySec * 4));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));

    // Underheat immediately after transition must be ignored until closed-loop stabilization expires.
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(kClosedLoopStabDelaySec - kMarginSec));
    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));

    Timer::advanceMockTime(toUs(2 * kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::ClosedLoop, HeaterAllow::Allowed, kGoodVoltage, 600.0f));
}

// ====================
// Channel status checks
// ====================

TEST(HeaterStatus_Checks, PreheatTransitionSetsPreheatStatus)
{
    MockHeater dut;
    configureDut(dut);

    SetStatus(0, wbo::Status::SensorUndervoltage);

    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Stopped, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
    EXPECT_EQ(wbo::Status::Preheat, GetCurrentStatus(0));
}

TEST(HeaterStatus_Checks, WarmupTransitionSetsWarmupStatus)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::WarmupRamp,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kGoodVoltage, 780.0f));
    EXPECT_EQ(wbo::Status::Warmup, GetCurrentStatus(0));
}

TEST(HeaterStatus_Checks, ClosedLoopTransitionSetsRunningStatus)
{
    MockHeater dut;
    configureDut(dut);

    EXPECT_EQ(HeaterState::ClosedLoop,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 780.0f));
    EXPECT_EQ(wbo::Status::RunningClosedLoop, GetCurrentStatus(0));
}

TEST(HeaterStatus_Checks, FaultTransitionsSetFaultStatus)
{
    MockHeater dut;
    configureDut(dut);

    // Warmup timeout -> SensorDidntHeat
    Timer::setMockTime(toUs(kWarmupTimeoutSec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::WarmupRamp, HeaterAllow::Allowed, kGoodVoltage, 500.0f));
    EXPECT_EQ(wbo::Status::SensorDidntHeat, GetCurrentStatus(0));

    // Overvoltage fault -> Overvoltage
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kOvervoltageFaultVoltage, 500.0f));
    EXPECT_EQ(wbo::Status::SensorOvervoltage, GetCurrentStatus(0));

    // Reset controller timers/retry state before undervoltage scenario.
    configureDut(dut);

    // Undervoltage fault -> Undervoltage
    primeUndervoltTimer(dut);
    EXPECT_EQ(HeaterState::Preheat,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));
    Timer::advanceMockTime(toUs(kUndervoltFaultDelaySec + kMarginSec));
    EXPECT_EQ(HeaterState::Stopped,
        dut.GetNextState(HeaterState::Preheat, HeaterAllow::Allowed, kLowVoltage, 700.0f));
    EXPECT_EQ(wbo::Status::SensorUndervoltage, GetCurrentStatus(0));
}

TEST(HeaterStatus_Checks, HasFaultReflectsFaultState)
{
    MockHeater dut;
    configureDut(dut);

    SetStatus(0, wbo::Status::Preheat);
    EXPECT_FALSE(HasFault());

    SetStatus(0, wbo::Status::RunningClosedLoop);
    EXPECT_FALSE(HasFault());

    SetStatus(0, wbo::Status::SensorOverheat);
    EXPECT_TRUE(HasFault());
}
