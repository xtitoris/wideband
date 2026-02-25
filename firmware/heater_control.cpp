#include "heater_control.h"

#include "status.h"
#include "sampling.h"

#include "port.h"

using namespace wbo;

#define HEATER_OVERVOLTAGE_THRESHOLD 23.0f
#define MAX_HEATER_VOLTAGE 12.0f

static const PidConfig heaterPidConfig =
{
    .kP = 0.3f,      // kP
    .kI = 0.3f,      // kI
    .kD = 0.01f,     // kD
    .clamp = 3.0f,      // Integrator clamp (volts)
};

HeaterControllerBase::HeaterControllerBase(int ch)
    : m_pid(heaterPidConfig, HEATER_CONTROL_PERIOD)
    , ch(ch)
{
}

void HeaterControllerBase::Configure(float targetTempC, float targetEsr, struct HeaterConfig* configuration)
{
    m_targetTempC = targetTempC;
    m_targetEsr = targetEsr;
    m_configuration = configuration;
    m_retryTime = 0;

    m_stateTimer.reset();

    m_heaterStableTimer.reset();
    m_undervoltTimer.reset();
    m_underheatTimer.reset();
    m_overheatTimer.reset();
}

bool HeaterControllerBase::IsRunningClosedLoop() const
{
    return heaterState == HeaterState::ClosedLoop;
}

float HeaterControllerBase::GetTargetTemp() const
{
    return m_targetTempC;
}

float HeaterControllerBase::GetHeaterEffectiveVoltage() const
{
    return heaterVoltage;
}

HeaterState HeaterControllerBase::GetHeaterState() const
{
    return heaterState;
}

HeaterState HeaterControllerBase::changeState(HeaterState newState, Status status)
{
    m_stateTimer.reset();
    SetStatus(ch, status);
    return newState;
}

HeaterState HeaterControllerBase::stopWithRetry(Status status, int retryTimeSec)
{
    m_retryTime = retryTimeSec;
    return changeState(HeaterState::Stopped, status);
}

HeaterState HeaterControllerBase::GetNextState(HeaterState currentState, HeaterAllow heaterAllowState, float heaterSupplyVoltage, float sensorTemp)
{
    const float overheatTemp = m_targetTempC + 100;
    const float closedLoopTemp = m_targetTempC - 30;
    const float underheatTemp = m_targetTempC - 100;

    // Common checks that apply to all running states
    // This is here to avoid repeating these checks in every state - if any of these conditions are met, we want to stop heating immediately
    // There should be no code outside of specific states otherwise to keep the state machine logic clear and maintainable

    if (heaterSupplyVoltage > HEATER_OVERVOLTAGE_THRESHOLD)
    {
        // If voltage is dangerously high, immediately stop heating to protect the sensor
        return stopWithRetry(Status::SensorOvervoltage, HEATER_OVERVOLT_RETRY_TIMEOUT);
    }

    // If voltage is back to normal, reset undervoltage timer
    if (heaterSupplyVoltage > m_configuration->HeaterSupplyOffVoltage)
    {
        m_undervoltTimer.reset();
    }

    // Voltage has been too low for too long, stop heating and set fault
    if (m_undervoltTimer.hasElapsedSec(0.5f))
    {
        return stopWithRetry(Status::SensorUndervoltage, HEATER_UNDERVOLT_RETRY_TIMEOUT);
    }

    switch (currentState)
    {
        case HeaterState::Stopped:
        {
            // If retry timer is running, wait until it elapses before allowing to start preheat again
            if ((m_retryTime) && !(m_stateTimer.hasElapsedSec(m_retryTime)))
            {
                break;
            }
            // Disable retry timer
            m_retryTime = 0;

            if (heaterAllowState == HeaterAllow::NotAllowed)
            {
                // ECU has explicitly disallowed heating, stay stopped
                break;
            }

            // If we haven't received any CAN message about whether heating is allowed or not
            // we should wait until heater supply voltage stabilizes above threshold before allowing to start heating
            if (heaterAllowState == HeaterAllow::Unknown)
            {
                // Reset voltage_stable timer if voltage is too low
                if (heaterSupplyVoltage < m_configuration->HeaterSupplyOnVoltage)
                {
                    m_heaterStableTimer.reset();
                }

                if (!m_heaterStableTimer.hasElapsedSec(HEATER_VOLTAGE_STAB_TIME))
                {
                    // Continue to wait for voltage to stabilize before allowing heating
                    break;
                }
            }

            // Otherwise, start preheat
            return changeState(HeaterState::Preheat, Status::Preheat);
            break;
        }
        case HeaterState::Preheat:
        {
            bool startRamp = false;

            rampVoltage = 7.0f;

            #ifdef HEATER_FAST_HEATING_THRESHOLD_T
            // If the sensor is already above a certain temperature, skip preheat and go straight to warmup ramp
            if (sensorTemp > HEATER_FAST_HEATING_THRESHOLD_T)
            {
                // Start ramp at higher voltage to speed up heating for cases like hot restarts where sensor is already warm
                rampVoltage = 9.0f;
                startRamp = true;
            }

            #endif

            // If the sensor is already hot (engine running?), skip preheat and go straight to warmup ramp
            if (sensorTemp > closedLoopTemp)
            {
                startRamp = true;
            }

            // If enough time has elapsed in preheat (condensation phase has passed), start warmup ramp
            if (m_stateTimer.hasElapsedSec(m_configuration->PreheatTimeSec))
            {
                startRamp = true;
            }

            if (startRamp)
            {
                // Reset the timer for the warmup phase
                return changeState(HeaterState::WarmupRamp, Status::Warmup);
            }

            // Stay in preheat
            break;
        }
        case HeaterState::WarmupRamp:
        {
            // Already hot enough, start closed loop
            if (sensorTemp > closedLoopTemp)
            {
                return changeState(HeaterState::ClosedLoop, Status::RunningClosedLoop);
            }

            // If we've been trying to warm up for too long without reaching the closed loop threshold,
            // something is wrong (like heater not working), so stop and set fault
            if (m_stateTimer.hasElapsedSec(HEATER_WARMUP_TIMEOUT))
            {
                return stopWithRetry(Status::SensorDidntHeat, HEATER_DIDNOTHEAT_RETRY_TIMEOUT);
            }

            break;
        }
        case HeaterState::ClosedLoop:
        {
            // Over/under heat timers track how long it's been since
            // temperature was within normal range (then we abort if
            // it's been too long out of range)
            if (sensorTemp <= overheatTemp)
            {
                m_overheatTimer.reset();
            }

            if (sensorTemp >= underheatTemp)
            {
                m_underheatTimer.reset();
            }

            // Check for overheat befor checking for closed loop stable time
            // if we're overheating, we want to stop as soon as possible, even if we haven't been in closed loop for very long
            if (m_overheatTimer.hasElapsedSec(0.5f))
            {
                return stopWithRetry(Status::SensorOverheat, HEATER_OVERHEAT_RETRY_TIMEOUT);
            }

            
            if (!m_stateTimer.hasElapsedSec(HEATER_CLOSED_LOOP_STAB_TIME))
            {
                // give some time for stabilization...
                // looks like heavy ramped Ipump affects sensorTemp measure
                // and right after switch to closed loop sensorTemp drops below underhead threshold
                break;
            }


            else if (m_underheatTimer.hasElapsedSec(0.5f))
            {
                return stopWithRetry(Status::SensorUnderheat, HEATER_UNDERHEAT_RETRY_TIMEOUT);
            }

            break;
        }
    }

    return currentState;
}

float HeaterControllerBase::GetVoltageForState(HeaterState state, float sensorEsr)
{
    float heaterVoltage = 0;

    switch (state)
    {
        case HeaterState::Preheat:
            // Max allowed during condensation phase (preheat) is 2v
            heaterVoltage = 2.0f;
            break;

        case HeaterState::WarmupRamp:
            if (rampVoltage < 12)
            {
                // 0.4 volt per second, divided by battery voltage and update rate
                constexpr float rampRateVoltPerSecond = 0.4f;
                constexpr float heaterFrequency = 1000.0f / HEATER_CONTROL_PERIOD;
                rampVoltage += (rampRateVoltPerSecond / heaterFrequency);
            }

            heaterVoltage = rampVoltage;
            break;

        case HeaterState::ClosedLoop:
            // "nominal" heater voltage is 7.5v, so apply correction around that point (instead of relying on integrator so much)
            // Negated because lower resistance -> hotter

            // TODO: heater PID should operate on temperature, not ESR
            heaterVoltage = 7.5f - m_pid.GetOutput(m_targetEsr, sensorEsr);
            break;

        case HeaterState::Stopped:
            // Something has gone wrong, turn off the heater.
            heaterVoltage = 0;
            break;
    }

    // Limit to MAX_HEATER_VOLTAGE as per specification
    if (heaterVoltage > MAX_HEATER_VOLTAGE) {
        heaterVoltage = MAX_HEATER_VOLTAGE;
    }

    return heaterVoltage;
}

void HeaterControllerBase::Update(const ISampler& sampler, HeaterAllow heaterAllowState)
{
    // Read sensor state
    float sensorEsr = sampler.GetSensorInternalResistance();
    float sensorTemperature = sampler.GetSensorTemperature();

    // TODO: Clean this up, looks like a mess
    // Move supply voltage reading logic into port so that it's cleaner and more testable
    // Also unify voltage reading logic across the codebase, so that everyting is consistent
    #if defined(HEATER_INPUT_DIVIDER)
        // if board has ability to measure heater supply localy - use it
        float heaterSupplyVoltage = sampler.GetInternalHeaterVoltage();
    #elif defined(BOARD_HAS_VOLTAGE_SENSE)
        float heaterSupplyVoltage = GetSupplyVoltage();
    #else // not BOARD_HAS_VOLTAGE_SENSE
        // this board rely on measured voltage from ECU
        float heaterSupplyVoltage = GetRemoteBatteryVoltage();
    #endif

    // Run the state machine
    heaterState = GetNextState(heaterState, heaterAllowState, heaterSupplyVoltage, sensorTemperature);
    heaterVoltage = GetVoltageForState(heaterState, sensorEsr);

    float duty = 0;

    if (heaterVoltage > 0)
    {
        // duty = (V_eff / V_batt) ^ 2
        float voltageRatio = heaterVoltage / heaterSupplyVoltage;
        duty = voltageRatio * voltageRatio;

        #ifdef HEATER_MAX_DUTY
        cycle++;
        // limit PWM each 10th cycle (2 time per second) to measure heater supply voltage through "Heater-"
        if ((cycle % 10) == 0) {
            if (duty > HEATER_MAX_DUTY) {
                duty = HEATER_MAX_DUTY;
            }
        }
        #endif
    }

    // Pipe the output to the heater driver
    SetDuty(duty);
}

const char* describeHeaterState(HeaterState state)
{
    switch (state) {
        case HeaterState::Preheat:
            return "Preheat";
        case HeaterState::WarmupRamp:
            return "WarmupRamp";
        case HeaterState::ClosedLoop:
            return "ClosedLoop";
        case HeaterState::Stopped:
            return "Stopped";
    }

    return "Unknown";
}
