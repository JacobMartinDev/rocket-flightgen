#pragma once

#include "types.h"

#include <cstdint>

// ---------------------------------------------------------------------------
// Transition tuning.
//
// These are policy, not physics, so they live where a test can reach them.
// The values below are STARTING POINTS -- verify them against your own
// flightgen.py output and change them if the data disagrees.
//
// BOOST_ACCEL_THRESHOLD_M_S2: must sit well above the ~9.81 the accelerometer
//   reads on the pad, and well below what the motor actually produces. Your
//   current motor averages 200 N-s / 1.5 s = 133 N on an 0.8 kg vehicle, so
//   boost specific force is roughly 167 m/s^2. A threshold near 20 leaves a
//   2x margin over the pad reading and an 8x margin under real boost.
//
// CONFIRMATION_SAMPLES: how many consecutive qualifying samples before a
//   transition fires. At 100 Hz, 5 samples is 50 ms. Your burn lasts 1.5 s
//   (150 samples), so 5 costs you almost nothing in detection latency while
//   making it impossible for one noisy reading to throw the state machine off
//   the pad. This is the single most important idea in the whole design.
// ---------------------------------------------------------------------------
constexpr float        BOOST_ACCEL_THRESHOLD_M_S2 = 20.0F;
constexpr std::uint8_t CONFIRMATION_SAMPLES       = 5;

// ---------------------------------------------------------------------------
// The flight state machine.
//
// Contract for step():
//   - no I/O of any kind (no printing, no files, no logging)
//   - no reading the clock -- time arrives inside the sample
//   - no dynamic allocation, ever
//   - no exceptions
//
// Those rules are not stylistic. A step() with no hidden dependencies can be
// driven by a test at thousands of times real speed, and can be recompiled for
// a microcontroller without touching a line. Every constraint above exists to
// protect one of those two properties.
// ---------------------------------------------------------------------------
class FlightStateMachine {
public:
    // Advance the machine by exactly one sample and return the state afterward.
    //
    // This returns the current state on EVERY call, not just when it changes.
    // That is deliberate: it makes the interface level-triggered, so a caller
    // that drops a return value simply learns the same thing next cycle.
    // Edge-triggered designs -- signaling only on change -- have a failure mode
    // where one missed edge leaves the caller permanently out of sync, which is
    // not a risk worth taking in a control loop.
    State step(const SensorSample& sample) noexcept;

    // Read the state without advancing the machine. Tests need this to check
    // the initial state before any sample exists.
    State state() const noexcept;

private:
    // Default member initializers, not a constructor body. The object is fully
    // valid the instant it is constructed, with no code running -- which is what
    // lets you declare one of these as a static or global on a microcontroller
    // without worrying about initialization order.
    State current_state_ = State::PAD;

    // ONE counter, shared by every transition -- not one per transition.
    //
    // The current state determines which transition is being evaluated: PAD only
    // ever looks for BOOST, BOOST only ever looks for COAST. At most one
    // candidate is armed at a time, so one counter suffices.
    //
    // The invariant: reset this to 0 whenever current_state_ changes, and reset
    // it whenever a sample fails the condition. If you later add abort paths
    // (BOOST -> LANDED on motor failure), two candidates become armed at once
    // and this assumption breaks -- revisit it then, not now.
    std::uint8_t confirmation_count_ = 0;
};