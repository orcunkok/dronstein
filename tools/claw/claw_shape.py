"""The Hiwonder (LewanSoul) claw's measured dimensions, shared by build_claw.py (its look) and claw_physics.py (its
MuJoCo parts), so both are made from the same numbers. Frame: Pod/Lucky, Y up, metres: pivots on the X axis at
+-PIVOT_X, pivot axes along Z, the four plates of a finger stacked along Z, the fingers hanging down."""

import math

# ---------------------------------------------------------------- dimensions (m)
PIVOT_X = 0.0128          # pivots 25.6 mm apart; their gears mesh
# The finger's centreline in the left finger's frame (+x inward, -y toward the tip), from the pivot up the neck and
# out along the curve to the tip screw: the outer edge of the drawing offset inward by half the band.
CENTERLINE = (
    (0.0, 0.0), (0.0001, -0.0135), (0.0002, -0.0162), (-0.0008, -0.0218), (-0.0038, -0.0262), (-0.006, -0.0293),
    (-0.008, -0.0322), (-0.0098, -0.0353), (-0.0113, -0.0381), (-0.0126, -0.0411), (-0.0137, -0.0438),
    (-0.0146, -0.0467), (-0.0154, -0.0497), (-0.0159, -0.0526), (-0.0163, -0.0554), (-0.0165, -0.0585),
    (-0.0167, -0.0614), (-0.0166, -0.0641), (-0.0164, -0.0671), (-0.0159, -0.07), (-0.0154, -0.073),
    (-0.0147, -0.076), (-0.0139, -0.0789), (-0.0129, -0.0817), (-0.0117, -0.0845), (-0.0102, -0.0874),
    (-0.0084, -0.0903), (-0.0064, -0.0932), (-0.0041, -0.0962), (-0.0014, -0.0992), (0.0056, -0.1062),
)
HALF_OUT = 0.006          # outer edge to the centreline
PATH_STEP = 0.00075       # the centreline is sampled every 0.75 mm along its length (157 points)
# Centreline to the inner edge at each sample, in 0.1 mm: 6 mm up the plain neck, then the drawing's stepped teeth
# (4 to 6.2 mm), measured along the band's normal.
HALF_IN = (
    60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 63, 63, 63, 63, 63, 60, 60, 59, 59, 62, 61, 63, 64, 63, 61,
    60, 56, 54, 49, 50, 50, 55, 61, 62, 56, 45, 40, 42, 46, 52, 57, 56, 52, 45, 41, 40, 42, 46, 53, 57, 55, 49, 45,
    43, 40, 44, 53, 58, 58, 53, 49, 43, 42, 44, 49, 54, 59, 61, 54, 47, 43, 41, 45, 49, 55, 59, 57, 52, 47, 42, 43,
    46, 55, 58, 61, 56, 50, 44, 41, 43, 48, 53, 58, 60, 53, 47, 42, 41, 43, 49, 55, 58, 56, 52, 45, 43, 44, 47, 52,
    57, 58, 55, 50, 44, 41, 44, 47, 50, 57, 58, 51, 47, 40, 40, 41, 47, 53, 56, 54, 49, 41, 39, 40, 43, 47, 51, 54,
    50, 43, 39, 39, 39, 40, 48, 54, 50, 47, 40, 40, 44, 51, 59, 62, 62,
)
TIP_CAP = ((0.0073, -0.1077), 0.0053)    # the round fingertip: centre and radius, offset from the tip screw
PLATE_T = 0.002           # plate thickness
FLANGE_Z, FLANGE_T = 0.024, 0.002     # the holder's two flanges (centres) and their thickness: 50 mm apart outside
LAYER_Z = (-0.029, -0.0098, 0.0098, 0.029)   # finger plates: the outer two just outside the flanges, 60 mm overall
STANDOFF_R = 0.0028       # brass hex standoffs between the plates
SERVO_BOX = ((0.0545, 0.020, 0.0405), (0.0, 0.0075, 0.0))   # the LDX-335MG's body inside the holder: size, centre
SCREW_HEAD = (0.00275, 0.0016)     # M3 button head screwed into each standoff end: radius, height
STANDOFFS = ((-0.0043, -0.0253), (-0.0172, -0.0616), (0.0056, -0.1062))   # the three screws, on the centreline
HUB_R = 0.0102            # plate round the pivot
INNER_START = 29          # the two inner plates start at this sample (21.8 mm along the band): they only reach the first pillar
GEAR_TEETH, GEAR_ROOT, GEAR_TIP = 16, 0.0113, 0.0134   # sector gear on the top plate (the two gears mesh)
GEAR_SECTOR = (-20.0, 110.0)       # degrees, in the left finger's frame: the gear's teeth span the lower inner side
OPEN_DEG = 72.3           # each finger's swing from closed to fully open, measured on the drawing (Claw.cs uses it)
BRACKET_TOP, BRACKET_BOTTOM, BRACKET_WIDTH = 0.0214, -0.0121, 0.057   # the holder: top and bottom (y) and width (x)


def finger_path(step):
    """CENTERLINE resampled every step metres along its length. Returns the points and the length."""
    seg = [math.dist(a, b) for a, b in zip(CENTERLINE, CENTERLINE[1:])]
    total = sum(seg)
    n = max(1, round(total / step))
    points, k, done = [], 0, 0.0
    for i in range(n + 1):
        s = total * i / n
        while k < len(seg) - 1 and done + seg[k] < s:
            done += seg[k]
            k += 1
        t = min(1.0, (s - done) / seg[k])
        (ax, ay), (bx, by) = CENTERLINE[k], CENTERLINE[k + 1]
        points.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return points, total
