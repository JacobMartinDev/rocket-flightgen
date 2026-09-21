#include <cassert>

#include "flight_state.h"

int main()
{
    // A newly created flight-state machine must begin on the pad.
    FlightStateMachine machine;
    assert(machine.state() == State::PAD);

    // This sample represents strong upward acceleration.
    // It is above the 20 m/s^2 boost threshold.
    SensorSample boost_sample{
        0,        // timestamp in milliseconds
        0.0F,     // altitude in meters
        100.0F,   // upward acceleration
        true,     // barometer reading is valid
        true      // accelerometer reading is valid
    };

    // Four qualifying samples are not enough to confirm launch.
    assert(machine.step(boost_sample) == State::PAD);
    assert(machine.step(boost_sample) == State::PAD);
    assert(machine.step(boost_sample) == State::PAD);
    assert(machine.step(boost_sample) == State::PAD);

    // The fifth consecutive qualifying sample transitions to BOOST.
    assert(machine.step(boost_sample) == State::BOOST);

        // A low sample must reset the launch-confirmation streak.
    FlightStateMachine reset_machine;

    SensorSample low_accel_sample{
        0,        // timestamp in milliseconds
        0.0F,     // altitude in meters
        9.81F,    // acceleration while resting on the pad
        true,     // barometer reading is valid
        true      // accelerometer reading is valid
    };

    // Build a partial confirmation streak.
    assert(reset_machine.step(boost_sample) == State::PAD);
    assert(reset_machine.step(boost_sample) == State::PAD);

    // This low sample must erase the two-sample streak.
    assert(reset_machine.step(low_accel_sample) == State::PAD);

    // Four new high samples are still insufficient.
    assert(reset_machine.step(boost_sample) == State::PAD);
    assert(reset_machine.step(boost_sample) == State::PAD);
    assert(reset_machine.step(boost_sample) == State::PAD);
    assert(reset_machine.step(boost_sample) == State::PAD);

    // The fifth high sample starts BOOST.
    assert(reset_machine.step(boost_sample) == State::BOOST);

        // An invalid accelerometer reading must also reset the streak.
    FlightStateMachine invalid_machine;

    SensorSample invalid_accel_sample{
        0,        // timestamp in milliseconds
        0.0F,     // altitude in meters
        100.0F,   // value is high, but the reading is invalid
        true,     // barometer reading is valid
        false     // accelerometer reading is invalid
    };

    // Build a partial confirmation streak.
    assert(invalid_machine.step(boost_sample) == State::PAD);
    assert(invalid_machine.step(boost_sample) == State::PAD);

    // The invalid reading must reset the two-sample streak.
    assert(invalid_machine.step(invalid_accel_sample) == State::PAD);

    // Four new high samples are still insufficient.
    assert(invalid_machine.step(boost_sample) == State::PAD);
    assert(invalid_machine.step(boost_sample) == State::PAD);
    assert(invalid_machine.step(boost_sample) == State::PAD);
    assert(invalid_machine.step(boost_sample) == State::PAD);

    // The fifth new high sample transitions to BOOST.
    assert(invalid_machine.step(boost_sample) == State::BOOST);
    
    return 0;
}