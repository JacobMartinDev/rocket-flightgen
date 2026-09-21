#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Flight states.
//
// The underlying type is pinned to uint8_t so this is exactly one byte with the
// same representation on every compiler. That matters because these values are
// externally visible: the ground station consumes them as numeric flight-phase
// codes.
//
// The numeric values below are therefore part of a wire contract, not an
// implementation detail. DO NOT insert a new state in the middle -- every code
// after it would shift by one and the ground station would silently misreport
// the entire flight. Append new states at the end with the next free number.
// ---------------------------------------------------------------------------
enum class State : std::uint8_t {
    PAD    = 0,  // on the rail, motor not yet lit
    BOOST  = 1,  // motor burning, accelerating
    COAST  = 2,  // burnout to apogee, decelerating under gravity
    DROGUE = 3,  // past apogee, drogue deployed, slow descent
    MAIN   = 4,  // main deployed, final descent
    LANDED = 5   // on the ground, no longer moving
};

// One byte, as intended. If someone removes the ": std::uint8_t" above, this
// fails at compile time instead of silently changing the packet format.
static_assert(sizeof(State) == 1, "State must stay one byte: it is serialized");

// ---------------------------------------------------------------------------
// One cycle's worth of sensor data, produced at 100 Hz.
//
// This is a plain data carrier: no constructors, no methods, no logic. It says
// what the hardware saw, nothing about what it means.
//
// ACCELEROMETER CONVENTION -- read this before touching any threshold.
// accel_up_m_s2 is RAW SPECIFIC FORCE along the vehicle's up axis, exactly what
// a real accelerometer reports. Sitting still on the pad it reads about +9.81,
// because the accelerometer senses the normal force of the rail holding it up,
// not "acceleration" in the everyday sense. In free fall it reads ~0.
// Gravity is NOT subtracted here. If you want inertial acceleration, subtract
// it inside step() -- keep this struct honest about being sensor data.
//
// Field order puts the 4-byte members first so there is no padding between
// them. Even so, never memcpy this struct into a telemetry packet: struct
// layout and endianness are not portable. Serialize field by field.
// ---------------------------------------------------------------------------
struct SensorSample {
    std::uint32_t t_ms;             // monotonic ms since power-on
    float         altitude_m;       // barometric altitude, noisy and laggy
    float         accel_up_m_s2;    // specific force, up axis (see note above)

    // Validity flags. A sensor that failed to produce a reading this cycle sets
    // its flag false; the value field is then meaningless and must not be read.
    //
    // Why flags rather than NaN: every comparison against NaN returns false, so
    // both "value > threshold" and "value < threshold" are false at once. That
    // accidentally does the right thing often enough to hide the bug and fail
    // somewhere else later. Explicit flags force step() to decide what a
    // missing reading means -- which is exactly what the fault harness exists
    // to test.
    bool baro_valid;
    bool accel_valid;
};