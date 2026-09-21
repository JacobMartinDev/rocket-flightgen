DRY_MASS_KG = 0.70
PROPELLANT_MASS_KG = 0.10
BURN_TIME_S = 1.50
TOTAL_IMPULSE_NS = 200.0
GRAVITY_M_S2 = 9.80665
SIM_STEP_S = 0.001
OUTPUT_RATE_HZ = 100
MAX_SIM_TIME_S = 120.0

AVERAGE_THRUST_N = TOTAL_IMPULSE_NS / BURN_TIME_S


def thrust_n(t_s):
    # Simplification: constant thrust instead of a real motor's thrust curve.
    if 0.0 <= t_s < BURN_TIME_S:
        return AVERAGE_THRUST_N
    return 0.0


def mass_kg(t_s):
    burn_fraction = min(max(t_s / BURN_TIME_S, 0.0), 1.0)
    remaining_propellant_kg = PROPELLANT_MASS_KG * (1.0 - burn_fraction)
    return DRY_MASS_KG + remaining_propellant_kg


def drag_n(velocity_m_s):
    # Deliberately disabled for the drag-free reference check.
    return 0.0


def simulate():
    step = 0
    velocity_m_s = 0.0
    altitude_m = 0.0
    apogee_m = 0.0
    apogee_time_s = 0.0
    airborne = False

    sample_every = round(1.0 / (OUTPUT_RATE_HZ * SIM_STEP_S))
    max_steps = round(MAX_SIM_TIME_S / SIM_STEP_S)

    # Each row contains: time, altitude, velocity.
    rows = [(0.0, altitude_m, velocity_m_s)]

    while step < max_steps:
        t_s = step * SIM_STEP_S
        mass = mass_kg(t_s)
        thrust = thrust_n(t_s)
        weight = mass * GRAVITY_M_S2

        # Stay on the pad until thrust exceeds weight.
        if not airborne and thrust <= weight:
            acceleration_m_s2 = 0.0
        else:
            airborne = True
            net_force_n = thrust - weight - drag_n(velocity_m_s)
            acceleration_m_s2 = net_force_n / mass

        # Semi-implicit Euler: update velocity before altitude.
        velocity_m_s += acceleration_m_s2 * SIM_STEP_S
        altitude_m += velocity_m_s * SIM_STEP_S

        step += 1
        t_s = step * SIM_STEP_S

        # Track apogee at the full simulation rate.
        if altitude_m > apogee_m:
            apogee_m = altitude_m
            apogee_time_s = t_s

        landed = airborne and altitude_m <= 0.0
        if landed:
            altitude_m = 0.0

        # Collect output at 100 Hz.
        if step % sample_every == 0:
            rows.append((t_s, altitude_m, velocity_m_s))

        if landed:
            break
    else:
        # Runs only if the loop reaches its limit without landing.
        raise RuntimeError(
            f"Rocket did not land within "
            f"{MAX_SIM_TIME_S:.1f} simulated seconds"
        )

    print(f"Apogee: {apogee_m:.2f} m")
    print(f"Time to apogee: {apogee_time_s:.3f} s")
    print(f"Samples: {len(rows)}")

    return rows


if __name__ == "__main__":
    flight_rows = simulate()