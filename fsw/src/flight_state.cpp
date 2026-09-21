#include "flight_state.h"

// Return the current flight state without processing a new sensor sample.
State FlightStateMachine::state() const noexcept
{
    return current_state_;
}

// Process one sensor sample and return the state after processing it.
State FlightStateMachine::step(const SensorSample& sample) noexcept
{
    switch (current_state_) {
    case State::PAD:
        // A boost transition requires a valid accelerometer reading
        // above the configured threshold.
        if (sample.accel_valid &&
            sample.accel_up_m_s2 > BOOST_ACCEL_THRESHOLD_M_S2) {

            // Count consecutive samples that confirm rocket launch.
            ++confirmation_count_;
        } else {
            // A low or invalid sample breaks the confirmation sequence.
            confirmation_count_ = 0;
        }

        // Transition to BOOST only after enough consecutive confirmations.
        if (confirmation_count_ >= CONFIRMATION_SAMPLES) {
            current_state_ = State::BOOST;

            // The counter is no longer needed after the transition.
            confirmation_count_ = 0;
        }

        break;

    default:
        // Other flight-state transitions will be implemented later.
        break;
    }

    // Return the current state after processing this sample.
    return current_state_;
}