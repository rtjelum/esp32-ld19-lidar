"""
Two-piece snap-fit enclosure for LD19 lidar + LilyGo T-Display S3, side by side.

T-Display is mounted upright against the front wall (screen faces -Y, visible
through a window in the front of the box). Lidar dome pokes up through the lid.

Component reference dimensions:
  LD19 lidar body  : 38.6 x 38.6 x 22 mm (square base) + 38 mm dome, 13 mm tall
  T-Display S3     : 77.45 x 25.04 x ~8 mm; USB-C centered on one short edge

Output: lidar_tdisplay_box.stl (base + lid laid out side by side on Z=0).
"""

import numpy as np
import trimesh
from shapely.geometry import Polygon as ShapelyPolygon

# -------- Parameters (mm) --------
WALL = 2.0
FLOOR = 2.0
LID_TOP = 1.0

# Alignment tabs on the lid underside, sitting against the interior cavity walls.
LID_TAB_W = 6.0       # along wall
LID_TAB_T = 2.0       # perpendicular to wall (into cavity)
LID_TAB_H = 4.0       # how far the tab hangs down below the lid (4 mm deep)
LID_TAB_INSET = 8.0   # default corner offset along the wall
LID_TAB_FL_X_NUDGE = -3.0  # FL moves 3 mm left to clear brace h
LID_TAB_TIGHTEN = 0.15 # press each tab 0.15 mm into the wall for tight fit

# LD19: footprint from STL (bbox 53.95 × 46.29), vertical dims from ld19.png spec sheet
#   Total height : 34.80 mm
#   Dome         : 35.29 wide × 12.60 tall (top portion)
#   Body         : 22.20 tall (= 34.80 - 12.60); walls slope ~92° (≈2° flare to base)
#   3 round feet : Ø 7.15 mm, at local coords (0, 21.75), (±23.40, -10.17)
LIDAR_X = 53.95
LIDAR_Y = 46.29
LIDAR_BODY_Z = 22.20
LIDAR_DOME_D = 35.29
LIDAR_DOME_H = 12.60
LIDAR_TOTAL_Z = 34.80
LIDAR_FOOT_D = 7.15
LIDAR_FOOT_POS = [(0.0, 21.75), (-23.40, -10.17), (23.40, -10.17)]  # local to LD19 origin
# LD19 origin is centered in X within bbox, but offset in Y (bbox y-center is +3.725).
LIDAR_BBOX_Y_OFFSET = 3.725
LIDAR_BASE_H = 3.3           # tuned so body top (FLOOR+H+22.2) equals display top (FLOOR+25.5)

# T-Display module: user-given physical dims 57 × 10 × 25.5 mm
TDISP_LEN = 57.0             # wide  (X in box)
TDISP_THK = 10.0             # thick (Y, depth when upright)
TDISP_HEIGHT = 25.5          # tall  (Z, vertical when upright)

ITEM_CLEAR = 1.0             # clearance around each board
GAP = 4.0                    # cable routing space between boards
EXTRA_Y = 21.4               # added depth behind the boards (tuned so internal_y = 62 mm)
TRIM_USB_END = 18.95         # tuned so external X = 102 mm with the new TDISP_LEN
SCREEN_X_SHIFT = -0.25       # tuned so screen window LEFT edge is at X = 47
BACK_BRACE_BACK_OFFSET = 2.0 # tuned so i's top edge is at Y = 16

# USB-C anti-push brace (small interior wall at the board's inboard end)
BRACE_THICK = 2.0   # along X
BRACE_HEIGHT = 24.0 # along Z
BRACE_Y_LEN = 13.0  # Y extent of front and mid braces (anchored at front wall)
FRONT_BRACE_CX = 43.0            # tuned so g's left edge is at X = 42
MID_BRACE_THICK = 2.0            # mid brace X thickness
MID_BRACE_FROM_LEFT_WALL = 13.0  # mid brace distance from the left interior wall
USB_CUT_Y_SHIFT = 0.0            # shift USB cutout +Y (toward the back brace)

# Internal cavity
internal_x = LIDAR_X + ITEM_CLEAR + GAP + TDISP_LEN + ITEM_CLEAR - TRIM_USB_END  # ≈ 98
internal_y = max(LIDAR_Y, TDISP_THK) + 2 * ITEM_CLEAR + EXTRA_Y       # 62
internal_z = max(LIDAR_BODY_Z + LIDAR_BASE_H, TDISP_HEIGHT) + 1.0

ext_x = internal_x + 2 * WALL
ext_y = internal_y + 2 * WALL
ext_z = FLOOR + internal_z

# Component placement (centers, in box frame)
# Lidar: rotated 180° around Z. -Y bbox edge would touch braces (Y = WALL + BRACE_Y_LEN),
# but offset 6 mm toward the front so the lidar overlaps the brace tops by 6 mm.
LIDAR_ROT_DEG = 180.0
LIDAR_DOWN_OFFSET = 7.69        # tuned so body's -Y edge just touches brace tops (Y = 15)
LIDAR_LEFT_OFFSET = 1.0
lidar_cx = WALL + ITEM_CLEAR + LIDAR_X / 2 - LIDAR_LEFT_OFFSET
lidar_cy = WALL + BRACE_Y_LEN + LIDAR_Y / 2 - LIDAR_DOWN_OFFSET

# T-Display: upright, screen facing -Y, PCB pressed against front interior wall.
# PCB occupies Y = WALL .. WALL + TDISP_THK; X = right side of cavity; Z = floor up.
TDISP_X_SHIFT = -18.0        # shift T-Display toward -X (inward, +2 mm right)
tdisp_cx = WALL + ITEM_CLEAR + LIDAR_X + GAP + TDISP_LEN / 2 + TDISP_X_SHIFT
tdisp_cy = WALL + TDISP_THK / 2
tdisp_cz = FLOOR + TDISP_HEIGHT / 2

# USB-C cutout — board is upright, so connector long axis is vertical
USB_W = 6.0      # along Y (narrow)
USB_H = 11.0     # along Z (tall)

# Screen window in front wall (-Y); open at the top edge of the wall (slide-in slot)
SCREEN_W = 44.5       # along X (active area of 1.91" AMOLED at ~0.083 mm pitch)
SCREEN_BOTTOM_Z = 3.5  # bottom of cutout (just above floor)


def box(size, center):
    m = trimesh.creation.box(extents=size)
    m.apply_translation(center)
    return m


def cyl(radius, height, center):
    m = trimesh.creation.cylinder(radius=radius, height=height, sections=64)
    m.apply_translation(center)
    return m


def tri_prism(cx, cy, side, point_dir, height, z_base):
    """Equilateral triangle prism, apex pointing along normalized (dx, dy)."""
    dx, dy = point_dir
    h_tri = side * np.sqrt(3) / 2
    apex = (cx + dx * h_tri * 2 / 3, cy + dy * h_tri * 2 / 3)
    base_c = (cx - dx * h_tri / 3, cy - dy * h_tri / 3)
    px, py = -dy, dx
    base_l = (base_c[0] + px * side / 2, base_c[1] + py * side / 2)
    base_r = (base_c[0] - px * side / 2, base_c[1] - py * side / 2)
    poly = ShapelyPolygon([apex, base_l, base_r])
    prism = trimesh.creation.extrude_polygon(poly, height=height)
    prism.apply_translation((0, 0, z_base))
    return prism


# -------- Base (bottom shell) --------
outer = box((ext_x, ext_y, ext_z), (ext_x / 2, ext_y / 2, ext_z / 2))
cavity = box(
    (internal_x, internal_y, internal_z + 0.01),
    (ext_x / 2, ext_y / 2, FLOOR + (internal_z + 0.01) / 2),
)
# USB-C: through right wall (+X), aligned with the upright board's center face,
# shifted +Y toward the back brace to match the actual connector position.
usb_cut = box(
    (WALL + 2, USB_W, USB_H),
    (ext_x - WALL / 2 + 0.5, tdisp_cy + USB_CUT_Y_SHIFT, tdisp_cz),
)
# Screen slot: through front wall (-Y), notched open at the top edge of the wall
screen_top_z = ext_z + 1  # extend past top to guarantee clean cut
screen_h = screen_top_z - SCREEN_BOTTOM_Z
screen_cx = tdisp_cx + SCREEN_X_SHIFT
screen_win = box(
    (SCREEN_W, WALL + 2, screen_h),
    (screen_cx, WALL / 2 - 0.5, (SCREEN_BOTTOM_Z + screen_top_z) / 2),
)
base = outer.difference(cavity).difference(usb_cut).difference(screen_win)

# Lidar pedestal: raises the LD19 by LIDAR_BASE_H. Y spans from the top of
# braces g & h to the back wall; +X edge ends at the +X face of brace g.
pedestal_y0 = WALL                 # front wall
pedestal_y1 = ext_y - WALL         # back wall
pedestal_x0 = lidar_cx - LIDAR_X / 2
pedestal_x1 = FRONT_BRACE_CX + BRACE_THICK / 2
pedestal_x_len = pedestal_x1 - pedestal_x0
pedestal_y_len = pedestal_y1 - pedestal_y0
pedestal = box(
    (pedestal_x_len, pedestal_y_len, LIDAR_BASE_H),
    ((pedestal_x0 + pedestal_x1) / 2,
     (pedestal_y0 + pedestal_y1) / 2,
     FLOOR + LIDAR_BASE_H / 2),
)
base = base.union(pedestal)

# Front brace (g): fixed at FRONT_BRACE_CX, spans BRACE_Y_LEN from the front wall.
tdisp_inboard_x = tdisp_cx - TDISP_LEN / 2
brace_cx = FRONT_BRACE_CX
brace_cy = WALL + BRACE_Y_LEN / 2
brace = box(
    (BRACE_THICK, BRACE_Y_LEN, BRACE_HEIGHT),
    (brace_cx, brace_cy, FLOOR + BRACE_HEIGHT / 2),
)
base = base.union(brace)

# USB-end back brace: small wall on the +Y side of the T-Display, next to the USB
# cutout. Holds the board flat against the front wall when a cable is plugged in.
BACK_BRACE_LEN = 18.0     # along X (+3 mm)
BACK_BRACE_THICK = 2.0    # along Y
back_brace_cx = (ext_x - WALL) - BACK_BRACE_LEN / 2
back_brace_cy = WALL + TDISP_THK + BACK_BRACE_BACK_OFFSET + BACK_BRACE_THICK / 2
back_brace = box(
    (BACK_BRACE_LEN, BACK_BRACE_THICK, BRACE_HEIGHT),
    (back_brace_cx, back_brace_cy, FLOOR + BRACE_HEIGHT / 2),
)
base = base.union(back_brace)

# Mid brace (h): parallel to the front brace, positioned MID_BRACE_FROM_LEFT_WALL
# in from the left interior wall.
mid_brace_cx = WALL + MID_BRACE_FROM_LEFT_WALL + MID_BRACE_THICK / 2
mid_brace = box(
    (MID_BRACE_THICK, BRACE_Y_LEN, BRACE_HEIGHT),
    (mid_brace_cx, brace_cy, FLOOR + BRACE_HEIGHT / 2),
)
base = base.union(mid_brace)

# (no fixation on the base — alignment tabs are added to the lid below.)

# -------- Lid: plain 1 mm cap, no snap lip --------
lid = box((ext_x, ext_y, LID_TOP), (ext_x / 2, ext_y / 2, LID_TOP / 2))

# Lidar dome hole: dome and body are both centered on the LD19's rotor axis
# (the STL native origin). After bbox-center placement and 180° rotation,
# that lands at (lidar_cx, lidar_cy + LIDAR_BBOX_Y_OFFSET). The asymmetric bbox
# came from the feet protruding more on one side, not the body itself being offset.
dome_cy = lidar_cy + LIDAR_BBOX_Y_OFFSET
dome_hole = cyl(
    radius=LIDAR_DOME_D / 2 + 0.5,   # +0.5 mm clearance
    height=LID_TOP + 1,
    center=(lidar_cx, dome_cy, LID_TOP / 2),
)
lid = lid.difference(dome_hole)

# Alignment tabs on the lid underside, sitting against interior cavity walls.
# Tabs hang DOWN from the lid (in the lid's local frame, Z < 0).
# Format: (cx, cy, size_x, size_y)
#   long-wall tabs: size_x = LID_TAB_W (along wall), size_y = LID_TAB_T (into cavity)
#   short-wall tabs: size_x = LID_TAB_T (into cavity), size_y = LID_TAB_W (along wall)
# Keep the endpoints from the original 7-tab layout: each merged tab spans only
# from the outermost edge of the tabs it replaces.
# BACK: from BL's left edge to BR's right edge
back_x_lo = WALL + LID_TAB_INSET                              # 10  (BL left edge)
back_x_hi = ext_x - WALL - LID_TAB_INSET                      # 92  (BR right edge)
# LEFT: L2 was Y 20-26, L1 was Y 55-61
left_y_lo = 23.0 - LID_TAB_W / 2                              # 20  (L2 start)
left_y_hi = 58.0 + LID_TAB_W / 2                              # 61  (L1 end)
# RIGHT: R1 was Y 30-36, R2 was Y 50-56
right_y_lo = 33.0 - LID_TAB_W / 2                             # 30
right_y_hi = 53.0 + LID_TAB_W / 2                             # 56

tab_specs = [
    # FL: unchanged small tab on front wall
    (WALL + LID_TAB_INSET + LID_TAB_W / 2 + LID_TAB_FL_X_NUDGE,
     WALL + LID_TAB_T / 2 - LID_TAB_TIGHTEN,
     LID_TAB_W, LID_TAB_T),
    # BACK long tab spanning BL → BR
    ((back_x_lo + back_x_hi) / 2,
     ext_y - WALL - LID_TAB_T / 2 + LID_TAB_TIGHTEN,
     back_x_hi - back_x_lo, LID_TAB_T),
    # LEFT long tab spanning L2 → L1
    (WALL + LID_TAB_T / 2 - LID_TAB_TIGHTEN,
     (left_y_lo + left_y_hi) / 2,
     LID_TAB_T, left_y_hi - left_y_lo),
    # RIGHT long tab spanning R1 → R2
    (ext_x - WALL - LID_TAB_T / 2 + LID_TAB_TIGHTEN,
     (right_y_lo + right_y_hi) / 2,
     LID_TAB_T, right_y_hi - right_y_lo),
    # USB-side tab parallel to brace i: 15 mm long along X, 2 mm thick in Y.
    #   +X face presses 0.15 into the right wall (interior wall at X = ext_x-WALL).
    #   -Y face presses 0.15 into brace i (whose +Y face is at bb_top_y).
    ((ext_x - WALL) + LID_TAB_TIGHTEN - 15.0 / 2,        # cx so +X face = 100.15
     (WALL + TDISP_THK + BACK_BRACE_BACK_OFFSET + BACK_BRACE_THICK)
      - LID_TAB_TIGHTEN + LID_TAB_T / 2,                 # cy so -Y face = 15.85
     15.0, LID_TAB_T),
]
for tcx, tcy, sx, sy in tab_specs:
    tab = box((sx, sy, LID_TAB_H), (tcx, tcy, -LID_TAB_H / 2))
    lid = lid.union(tab)

# Export base and lid as separate STL files.
bottom_path = "lidar_tdisplay_box_bottom.stl"
top_path = "lidar_tdisplay_box_top.stl"
base.export(bottom_path)
lid.export(top_path)

# -------- Test-fit version: just the bottom, sliced to 10 mm tall --------
TEST_FIT_HEIGHT = 10.0
slab = box(
    (ext_x + 2, ext_y + 2, TEST_FIT_HEIGHT),
    (ext_x / 2, ext_y / 2, TEST_FIT_HEIGHT / 2),
)
test_fit = base.intersection(slab)
test_fit_path = "lidar_tdisplay_box_testfit.stl"
test_fit.export(test_fit_path)
print(f"Wrote {test_fit_path}  ({TEST_FIT_HEIGHT:.0f} mm tall test fit of base)")

print(f"Wrote {bottom_path}")
print(f"Wrote {top_path}")
print(f"  External (base): {ext_x:.1f} x {ext_y:.1f} x {ext_z:.1f} mm")
print(f"  Internal cavity: {internal_x:.1f} x {internal_y:.1f} x {internal_z:.1f} mm")
print(f"  Lidar center   : ({lidar_cx:.1f}, {lidar_cy:.1f})")
print(f"  T-Display ctr  : ({tdisp_cx:.1f}, {tdisp_cy:.1f}, {tdisp_cz:.1f})")
print(f"  Base watertight: {base.is_watertight}")
print(f"  Lid  watertight: {lid.is_watertight}")
print(f"  Triangles      : base={len(base.faces)}, lid={len(lid.faces)}")
