"""Software limits. GRBL cannot do soft limits without homing, so they live here.

All positions are relative to the zero pose, in axis order (front, rear, piston, bogie).
"""
from __future__ import annotations

import math

from config import AXIS_NAMES, BARREL_SPAN, FRONT, REAR, PITCH_DZ, TRAVEL


def pitch_deg(front: float, rear: float) -> float:
    x = max(-1.0, min(1.0, (front - rear) / BARREL_SPAN))
    return math.degrees(math.asin(x))


def clip_increment(target, delta, travel=TRAVEL, dz_band=PITCH_DZ):
    """Clip a jog increment so target + delta stays inside axis travel and the pitch band.

    target: where the already-queued motion ends.
    - Per axis: an axis at its limit stops while the others keep moving.
    - Pitch: (front - rear) is linear in the two rail moves, so only the front and
      rear components are scaled back, exactly to the boundary.
    - If target is already outside a limit (e.g. re-zeroed in a bad spot), motion
      back toward the inside is allowed and motion further out is blocked. Never jumps.
    Returns (clipped_delta, set_of_limit_names_hit).
    """
    d = list(delta)
    hit = set()

    for i, (lo, hi) in enumerate(travel):
        lo, hi = min(lo, target[i]), max(hi, target[i])
        want = target[i] + d[i]
        got = min(max(want, lo), hi)
        if abs(got - want) > 1e-9:
            hit.add(AXIS_NAMES[i])
        d[i] = got - target[i]

    lo, hi = dz_band
    dz0 = target[FRONT] - target[REAR]
    ddz = d[FRONT] - d[REAR]
    dz1 = dz0 + ddz
    if (ddz > 0 and dz1 > hi) or (ddz < 0 and dz1 < lo):
        bound = hi if ddz > 0 else lo
        f = min(1.0, max(0.0, (bound - dz0) / ddz))
        d[FRONT] *= f
        d[REAR] *= f
        hit.add("pitch")

    return d, hit
