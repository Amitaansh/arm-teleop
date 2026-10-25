"""Teleop configuration. Units: mm, mm/s, seconds.

Axis order is fixed everywhere in the code: (front, rear, piston, bogie).
GRBL axis letters: front pair = X (both front drives share one STEP/DIR), rear = Y,
piston = Z, bogie = A.
"""
import math

# ---- Serial link to the Mega 2560 running grbl-Mega-5X
PORT = "/dev/ttyACM0"        # Mega 2560 shows up as ttyACM*; clones with CH340 as ttyUSB*
BAUD = 115200                # GRBL default
STATUS_HZ = 20               # '?' polling rate

# ---- Keyboard (Linux evdev)
KEYBOARD_DEVICE = None       # None = auto-detect. Or e.g. "/dev/input/by-id/usb-...-event-kbd"
GRAB_KEYBOARD = True         # exclusive grab: keys do not reach the terminal.
                             # Ctrl-C on THIS keyboard then does nothing; quit with Esc.

# ---- Axes
AXIS_NAMES = ("front", "rear", "piston", "bogie")
AXIS_LETTERS = ("X", "Y", "Z", "A")
FRONT, REAR, PISTON, BOGIE = range(4)

# Travel relative to the ZERO pose (rails bottomed, piston retracted, bogie centred)
TRAVEL = (
    (0.0, 550.0),            # front pair
    (0.0, 550.0),            # rear
    (0.0, 500.0),            # piston   (PLACEHOLDER until the actuator is fixed)
    (-224.0, 224.0),         # bogie: yaw bearing angle asin(X_B / 600) <= 22 deg
)

# Pitch band on (front - rear): theta = asin((front - rear) / 600)
BARREL_SPAN = 600.0
PITCH_MIN_DEG, PITCH_MAX_DEG = -35.0, 40.0
PITCH_DZ = (BARREL_SPAN * math.sin(math.radians(PITCH_MIN_DEG)),
            BARREL_SPAN * math.sin(math.radians(PITCH_MAX_DEG)))

# ---- Speeds
# Per-axis speed at gear 3 (mm/s). Keep these at or below GRBL's $110-$113 (which are mm/min).
VMAX = (20.0, 20.0, 40.0, 100.0)
GEARS = {1: 0.10, 2: 0.40, 3: 1.00}   # fraction of VMAX
DEFAULT_GEAR = 1
UNZEROED_GEAR = 1                     # before zeroing: limits are unknown, so fine speed only

# ---- Jog streaming
JOG_DT = 0.10                # seconds of motion per jog command
PREFILL = 2                  # jog commands sent back to back when a motion starts
MAX_QUEUED_BLOCKS = 4        # max planner blocks inside GRBL. Bounds how far the arm
                             # coasts if the Jetson side dies mid-jog (~0.4 s of motion).
LOOP_HZ = 50
ZERO_HOLD_S = 1.0            # hold Z this long to set zero (stops accidental re-zero)
DISPLAY_HZ = 5

# ---- Key map (evdev key names). Vector = direction per axis (front, rear, piston, bogie).
# Held keys add together; each axis is then capped to [-1, +1].
MOTION_KEYS = {
    "KEY_UP":   (+1, +1, 0, 0),      # height up: front + rear together
    "KEY_DOWN": (-1, -1, 0, 0),
    "KEY_I":    (+1, -1, 0, 0),      # pitch up (nose up): front up, rear down
    "KEY_K":    (-1, +1, 0, 0),      # pitch down
    "KEY_W":    (+1, 0, 0, 0),       # front pair only
    "KEY_S":    (-1, 0, 0, 0),
    "KEY_E":    (0, +1, 0, 0),       # rear only
    "KEY_D":    (0, -1, 0, 0),
    "KEY_U":    (0, 0, -1, 0),       # piston in
    "KEY_O":    (0, 0, +1, 0),       # piston out
    "KEY_J":    (0, 0, 0, -1),       # bogie left
    "KEY_L":    (0, 0, 0, +1),       # bogie right
}

COMMAND_KEYS = {
    "KEY_1": "gear1",
    "KEY_2": "gear2",
    "KEY_3": "gear3",
    "KEY_SPACE": "stop",             # stop; motion stays off until all motion keys are released
    "KEY_Z": "zero",                 # hold ZERO_HOLD_S at the zero pose
    "KEY_C": "clear_alarm",          # $X unlock after a GRBL alarm (also drops the zero)
    "KEY_ESC": "quit",
}
