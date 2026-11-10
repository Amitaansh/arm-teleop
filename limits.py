"""Software limits. GRBL cannot do soft limits without homing, so they live here.

All positions are relative to each axis's zero, in axis order (FL, FR, rear, piston, bogie).
An axis that has no zero yet (referenced[i] False) has no travel limit, and the rack and
pitch limits only apply once all the rails they involve are zeroed.
"""
from __future__ import annotations

import math

from config import (AXIS_NAMES, BARREL_SPAN, FL, FR, FRONT_RACK_MAX_MM, PITCH_DZ, REAR,
                    TRAVEL)


def front_height(p) -> float:
    return (p[FL] + p[FR]) / 2.0


def pitch_deg(front: float, rear: float) -> float:
    x = max(-1.0, min(1.0, (front - rear) / BARREL_SPAN))
    return math.degrees(math.asin(x))


def clip_increment(target, delta, travel=TRAVEL, referenced=None, dz_band=PITCH_DZ,
                   rack_max=FRONT_RACK_MAX_MM):
    """Clip a jog increment so target + delta stays inside the limits.

    target: where the already-queued motion ends.
    - Per axis travel: an axis at its limit stops while the others keep moving.
    - Front pair: if both front rails are meant to move together (same sign) and one is
      clipped, the other is clipped to match, so they stay square.
    - Rack (FL - FR) stays within +-rack_max. Only ever shrinks moves, never adds any.
    - Pitch: (front - rear) is linear in the rail moves, so only the rail components are
      scaled back, exactly to the boundary.
    - If target is already outside a limit (e.g. zeroed in a bad spot), motion back toward
      the inside is allowed and motion further out is blocked. Never jumps.
    Returns (clipped_delta, set_of_limit_names_hit).
    """
    n = len(delta)
    ref = [True] * n if referenced is None else list(referenced)
    d = list(delta)
    hit = set()

    paired = d[FL] * d[FR] > 0 and abs(d[FL] - d[FR]) < 1e-9
    for i, (lo, hi) in enumerate(travel):
        if not ref[i]:
            continue
        lo, hi = min(lo, target[i]), max(hi, target[i])
        want = target[i] + d[i]
        got = min(max(want, lo), hi)
        if abs(got - want) > 1e-9:
            hit.add(AXIS_NAMES[i])
        d[i] = got - target[i]
    if paired and abs(d[FL] - d[FR]) > 1e-9:          # keep the pair square at a limit
        m = min(abs(d[FL]), abs(d[FR]))
        s = 1.0 if d[FL] > 0 else -1.0
        d[FL] = d[FR] = s * m

    if ref[FL] and ref[FR]:
        r0 = target[FL] - target[FR]
        r1 = r0 + d[FL] - d[FR]
        lo, hi = min(-rack_max, r0), max(rack_max, r0)
        if r1 > hi + 1e-9:                             # FL too high vs FR
            excess = r1 - hi
            if d[FL] > 0:
                cut = min(excess, d[FL]); d[FL] -= cut; excess -= cut
            if excess > 0 and d[FR] < 0:
                cut = min(excess, -d[FR]); d[FR] += cut
            hit.add("rack")
        elif r1 < lo - 1e-9:                           # FL too low vs FR
            excess = lo - r1
            if d[FL] < 0:
                cut = min(excess, -d[FL]); d[FL] += cut; excess -= cut
            if excess > 0 and d[FR] > 0:
                cut = min(excess, d[FR]); d[FR] -= cut
            hit.add("rack")

    if ref[FL] and ref[FR] and ref[REAR]:
        lo, hi = dz_band
        dz0 = front_height(target) - target[REAR]
        ddz = front_height(d) - d[REAR]
        dz1 = dz0 + ddz
        if (ddz > 0 and dz1 > hi) or (ddz < 0 and dz1 < lo):
            bound = hi if ddz > 0 else lo
            f = min(1.0, max(0.0, (bound - dz0) / ddz))
            for i in (FL, FR, REAR):
                d[i] *= f
            hit.add("pitch")

    return d, hit
