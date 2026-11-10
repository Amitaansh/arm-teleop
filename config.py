"""Teleop configuration. Units: mm, mm/s, seconds.

Axis order is fixed everywhere in the code: (front_left, front_right, rear, piston, bogie).
Every motor has its own GRBL axis: front-left = X, front-right = B (RAMPS E1 pins),
rear = Y, piston = Z, bogie = A.
"""
import math

# ---- Serial link to the Mega 2560 running grbl-Mega-5X
PORT = "/dev/ttyACM0"        # Mega 2560 shows up as ttyACM*; clones with CH340 as ttyUSB*
BAUD = 115200                # GRBL default
STATUS_HZ = 20               # '?' polling rate

# ---- Inputs. All of them can be on at once; held keys from every source add together.
# 1. A keyboard plugged into the Jetson (Linux evdev). Needs the 'input' group.
USE_KEYBOARD = True          # --no-keyboard turns it off
KEYBOARD_DEVICE = None       # None = auto-detect. Or e.g. "/dev/input/by-id/usb-...-event-kbd"
GRAB_KEYBOARD = True         # exclusive grab: keys do not reach the terminal.
                             # Ctrl-C on THIS keyboard then does nothing; quit with Esc.
# 2. Remote input, see inputs.py:
#    - a web page (http://<jetson>:WEB_PORT): type into a browser on the Jetson's desktop
#      through AnyDesk, or on the C2 laptop; a gamepad plugged into that computer works too
#    - UDP packets, for a joystick bridge or the GUI
NET_LISTEN = "127.0.0.1"     # "0.0.0.0" also accepts other machines (C2 laptop). None = off
WEB_PORT = 8080              # web page; None = off
NET_PORT = 5005              # UDP; None = off
NET_TOKEN = None             # shared secret: if set, clients must send the same one
NET_TIMEOUT_S = 0.5          # nothing from a client for this long = all its keys released

# Gamepad on the web page (browser Gamepad API, "standard" mapping): stick directions and
# buttons act as the keys below. Axes: 0 = left X, 1 = left Y (up is -), 2 = right X,
# 3 = right Y. Buttons: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 LT, 7 RT, 8 back, 9 start,
# 12-15 d-pad up/down/left/right.
GAMEPAD_DEADZONE = 0.5
GAMEPAD_AXES = {             # axis: (key when pushed -, key when pushed +)
    1: ("KEY_UP", "KEY_DOWN"),       # left stick up/down: height
    0: ("KEY_J", "KEY_L"),           # left stick left/right: bogie
    3: ("KEY_I", "KEY_K"),           # right stick up/down: pitch
    2: ("KEY_U", "KEY_O"),           # right stick left/right: piston in/out
}
GAMEPAD_BUTTONS = {
    12: "KEY_W", 13: "KEY_S",        # d-pad up/down: front pair
    4: "KEY_E", 6: "KEY_D",          # LB / LT: rear up / down
    0: "KEY_1", 1: "KEY_2", 3: "KEY_3",   # A / B / Y: gears
    2: "KEY_SPACE",                  # X: stop
    8: "KEY_C",                      # back: reset drives
    9: "KEY_H",                      # start (hold 1 s): home everything
}

# ---- Axes
AXIS_NAMES = ("front_left", "front_right", "rear", "piston", "bogie")
AXIS_LABELS = ("F-left", "F-right", "rear", "piston", "bogie")
AXIS_LETTERS = ("X", "B", "Y", "Z", "A")   # GRBL letter of each axis, in the order above
GRBL_AXIS_ORDER = "XYZAB"                  # order GRBL reports MPos in (grbl-Mega-5X, N_AXIS 5)
FL, FR, REAR, PISTON, BOGIE = range(5)
N_AXES = 5
FRONT = (FL, FR)                           # the two front rails carry the crossbar together
RAILS = (FL, FR, REAR)

# Travel relative to the ZERO pose (rails bottomed, piston retracted, bogie centred).
# After homing, each axis's limits are the tighter of these and its measured stops.
TRAVEL = (
    (0.0, 550.0),            # front left
    (0.0, 550.0),            # front right
    (0.0, 550.0),            # rear
    (0.0, 500.0),            # piston   (PLACEHOLDER until the actuator is fixed)
    (-224.0, 224.0),         # bogie: yaw bearing angle asin(X_B / 600) <= 22 deg
)

# Pitch band on (front - rear), front = mean of the two front rails:
# theta = asin((front - rear) / 600)
BARREL_SPAN = 600.0
PITCH_MIN_DEG, PITCH_MAX_DEG = -35.0, 40.0
PITCH_DZ = (BARREL_SPAN * math.sin(math.radians(PITCH_MIN_DEG)),
            BARREL_SPAN * math.sin(math.radians(PITCH_MAX_DEG)))

# Front rails out of square (front_left - front_right). The crossbar spans them on pitch
# bearings only, so the two may differ by only a little. Enforced once both are zeroed.
FRONT_RACK_MAX_MM = 5.0
TRIM_SPEED = 2.0             # mm/s cap when one front rail moves without the other (R/F, T/G)

# ---- Drive alarms (ALM) and motor free (MF)
# ALM+ of all five drives joined -> A9 (GRBL reset/abort input, 4.7k pull-up to 5 V), ALM- -> GND.
# MF+ of each drive -> its axis's GRBL enable pin (X D38, B D30, Y A2, Z A8, A D24), MF- -> GND.
ALM_CONDUCTS_ON_FAULT = True   # the ALM output switches on (pulls A9 low) when a drive faults.
                               # If the status shows DRIVE ALM while every drive LED is
                               # green, the output works the other way round: set False.
AUTO_RESET = True              # reset the drives by themselves after a stall / drive alarm
AUTO_RESET_MAX = 3             # at most this many auto resets per AUTO_RESET_WINDOW_S;
AUTO_RESET_WINDOW_S = 60.0     # after that, something is jammed: wait for C
DRIVE_FREE_S = (0.3, 1.0)      # how long MF frees the motors, one entry per reset attempt
DRIVE_SETTLE_S = 0.3           # wait after re-energising before checking ALM again
WAKE_AXIS = BOGIE              # GRBL only re-energises the motors when it starts a move, so a
WAKE_MM = 0.05                 # reset ends with a tiny out-and-back jog on this axis (mm)

# Optional fallback: a relay that cuts the drives' 24-48 V, switched from a GRBL output.
# The ESS manual lists stall (Err5) and over-current (Err1) as "power down reset". If freeing
# the motor with MF does not clear the red LED, wire a relay to GRBL's coolant pin (check
# cpu_map.h for the pin) so M8 cuts drive power, and set POWER_CYCLE_OFF_CMD = "M8".
POWER_CYCLE_OFF_CMD = None     # None = no relay fitted
POWER_CYCLE_ON_CMD = "M9"
POWER_CYCLE_OFF_S = 1.0        # power off this long
POWER_CYCLE_BOOT_S = 1.5       # then wait this long for the drives to boot

# ---- Auto homing on the hard stops. See homing.py.
# Each motor is driven into its hard stop until its drive stalls (ALM), then the drives are
# reset. Directions: + = up / out / right, - = down / in / left. Check them before homing.
HOME_HOLD_S = 1.0              # hold a home key this long to start
HOME_JOBS = {                  # what each home key does: parts, in order
    "all": ("rails", "piston", "bogie"),
    "rails": ("rails",),
    "piston": ("piston",),
    "bogie": ("bogie",),
}
HOME_ENDS = {                  # which stops each part seeks, in order ("max" = +, "min" = -)
    "rails": ("max", "min"),   # every rail motor: top stop (measures travel), then bottom (zero)
    "piston": ("min",),        # in only. ("max", "min") also measures the stroke
    "bogie": ("max", "min"),   # right stop, then left stop; zero = midway between them
}
HOME_ZERO = ("min", "min", "min", "min", "center")   # per axis: zero at the min stop, or midway
HOME_MIN_STOP_MM = (0.0, 0.0, 0.0, 0.0, -224.0)      # where the min stop is from zero, for "min"
                                                     # (and for "center" if only "min" is homed)
HOME_PARK_MM = (None, None, None, None, 0.0)         # move here after homing (None = stay at stop)
HOME_SPEED = (10.0, 10.0, 10.0, 20.0, 30.0)          # mm/s per axis while seeking a stall
HOME_SEARCH_EXTRA_MM = 50.0    # give up after an axis's full travel plus this, with no stall
HOME_SQUARE_SEARCH_MM = 10.0   # how far one front rail may move alone to reach its own stop.
                               # More than this out of square: trim with R/F or T/G first. This is
                               # also the most the crossbar can rack during homing if a stop is off
HOME_ALONE_MAX_MM = 350.0      # how far the front pair or the rear may move without the other
                               # (pitch changes 1 deg per ~10 mm). More: stop, not a level start?
HOME_CONFIRM_MM = 6.0          # an abort without ALM showing in the status is checked again by
                               # seeking at most this much further: a real stop stalls again
                               # within one trip distance. Keep it ~3 x the trip distance
HOME_TOP_MARGIN_MM = 10.0      # limits stop this far short of each measured max stop (and both
                               # bogie stops). Must cover 3 x the drive's trip distance + clearance
HOME_BOTTOM_MARGIN_MM = 1.0    # and this far above each min stop (the zero itself is exact)

# ---- Speeds
# Per-axis speed at gear 3 (mm/s). Keep these at or below GRBL's $110-$114 (which are mm/min).
VMAX = (20.0, 20.0, 20.0, 40.0, 100.0)
GEARS = {1: 0.10, 2: 0.40, 3: 1.00}   # fraction of VMAX
DEFAULT_GEAR = 1
UNZEROED_GEAR = 1                     # until every axis is zeroed: limits are unknown, fine only

# ---- Jog streaming
JOG_DT = 0.10                # seconds of motion per jog command
PREFILL = 2                  # jog commands sent back to back when a motion starts
MAX_QUEUED_BLOCKS = 4        # max planner blocks inside GRBL. Bounds how far the arm
                             # coasts if the Jetson side dies mid-jog (~0.4 s of motion).
LOOP_HZ = 50
ZERO_HOLD_S = 1.0            # hold Z this long to set zero (stops accidental re-zero)

# ---- Display and log
DISPLAY_HZ = 5
LOG_LINES = 8                # recent log lines shown under the key menu
LOG_DIR = "logs"             # every run writes logs/teleop-<date>-<time>.log. None = off

# ---- Key map (evdev key names). Vector = direction per axis (FL, FR, rear, piston, bogie).
# Held keys add together; each axis is then capped to [-1, +1].
MOTION_KEYS = {
    "KEY_UP":   (+1, +1, +1, 0, 0),    # height up: all three rails together
    "KEY_DOWN": (-1, -1, -1, 0, 0),
    "KEY_I":    (+1, +1, -1, 0, 0),    # pitch up (nose up): front up, rear down
    "KEY_K":    (-1, -1, +1, 0, 0),    # pitch down
    "KEY_W":    (+1, +1, 0, 0, 0),     # front pair only
    "KEY_S":    (-1, -1, 0, 0, 0),
    "KEY_E":    (0, 0, +1, 0, 0),      # rear only
    "KEY_D":    (0, 0, -1, 0, 0),
    "KEY_R":    (+1, 0, 0, 0, 0),      # front-left only (trim, TRIM_SPEED)
    "KEY_F":    (-1, 0, 0, 0, 0),
    "KEY_T":    (0, +1, 0, 0, 0),      # front-right only (trim, TRIM_SPEED)
    "KEY_G":    (0, -1, 0, 0, 0),
    "KEY_U":    (0, 0, 0, -1, 0),      # piston in
    "KEY_O":    (0, 0, 0, +1, 0),      # piston out
    "KEY_J":    (0, 0, 0, 0, -1),      # bogie left
    "KEY_L":    (0, 0, 0, 0, +1),      # bogie right
}

COMMAND_KEYS = {
    "KEY_1": "gear1",
    "KEY_2": "gear2",
    "KEY_3": "gear3",
    "KEY_SPACE": "stop",             # stop; motion stays off until all motion keys are released
    "KEY_Z": "zero",                 # hold ZERO_HOLD_S at the zero pose
    "KEY_H": "home:all",             # hold HOME_HOLD_S: home everything (Space aborts)
    "KEY_F1": "home:rails",          # hold: home the three rails only
    "KEY_F2": "home:piston",         # hold: home the piston only
    "KEY_F3": "home:bogie",          # hold: home the bogie only
    "KEY_C": "clear_alarm",          # reset the drives (MF) + clear GRBL's alarm; drops the zero
    "KEY_ESC": "quit",               # Jetson keyboard only (remote clients can't quit teleop)
    "KEY_F12": "sim_fault",          # --sim only: simulate a drive stall
}

# The key menu on screen and on the web page. Every key above should appear here.
KEY_HELP = (
    (("KEY_UP", "KEY_DOWN"), "height: all rails"),
    (("KEY_I", "KEY_K"), "pitch up / down"),
    (("KEY_W", "KEY_S"), "front pair up / down"),
    (("KEY_E", "KEY_D"), "rear up / down"),
    (("KEY_R", "KEY_F"), "front-left only (trim)"),
    (("KEY_T", "KEY_G"), "front-right only (trim)"),
    (("KEY_U", "KEY_O"), "piston in / out"),
    (("KEY_J", "KEY_L"), "bogie left / right"),
    (("KEY_1", "KEY_2", "KEY_3"), "gear fine / mid / fast"),
    (("KEY_SPACE",), "stop, abort homing"),
    (("KEY_Z",), "hold: zero at this pose"),
    (("KEY_H",), "hold: home everything"),
    (("KEY_F1",), "hold: home rails"),
    (("KEY_F2",), "hold: home piston"),
    (("KEY_F3",), "hold: home bogie"),
    (("KEY_C",), "reset drives / clear alarm"),
    (("KEY_ESC",), "quit"),
    (("KEY_F12",), "simulate a stall (--sim)"),
)
