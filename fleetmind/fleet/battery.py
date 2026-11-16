"""Battery rules. Every tunable number lives here so you can explain/adjust it in one place."""

MOVE_DRAIN = 0.2          # % per move (matches Person A's sim)
SAFETY = 1.3              # safety factor on the energy estimate
RESERVE = 10.0            # battery % that must still be left after task + trip to charger
CHARGE_BELOW = {'cost': 35.0, 'fcfs': 15.0}   # smart charges early, baseline charges late
RESUME_AT = 90.0          # a charging robot becomes available again at this %


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def energy_needed(d1, d2, d3):
    """Estimated battery % for: robot->pickup (d1), pickup->drop (d2), drop->charger (d3)."""
    return MOVE_DRAIN * SAFETY * (d1 + d2 + d3)


def is_eligible(battery, d1, d2, d3):
    return battery - energy_needed(d1, d2, d3) >= RESERVE


def charge_threshold(mode):
    return CHARGE_BELOW.get(mode, 35.0)


def needs_charge(battery, mode):
    return battery < charge_threshold(mode)
