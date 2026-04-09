"""
CAUSTICFORGE v2.0 — Caustic Surface G-code Exporter for Blender 4.x
  v2.0 — Spiral finishing pass (Archimedean, outside-in CW climb)
         Square cut module (profile perimeter with tabs)
         Separate .nc file exports (roughing / finishing / square cut)
  v1.3 — Axis-swap auto-correct (Y/Z swapped OBJs from pipeline fixed in-memory)
         Auto-populate target object (physical_lens_8x8 auto-detected on panel open)
         Smart default output path (project directory, not relative //)
         CNC-READY status line in analysis panel
         (v1.2) Numpy heightfield (no BVH — instant on 500k+ face grids)
         (v1.2) Auto-detect caustic relief (ignores flat base slab)
         (v1.2) Non-uniform scale guard
         (v1.2) F code on every G01 line (NK105 requirement)

MESH STRUCTURE NOTE:
  Caustic OBJs from the Julia solver contain TWO vertex layers:
    Layer 1: flat base slab at constant Z (slab bottom — ignore for CNC)
    Layer 2: caustic surface (small relief — the actual machining target)
  This addon detects both layers, uses only the caustic surface for toolpaths,
  and reports the TRUE relief depth (not the slab thickness).

DISTORTION WARNING:
  The dZ/dXY ratio of the caustic surface encodes refraction angles.
  NEVER apply non-uniform scale (XY != Z) — it destroys the optical geometry.
  Always scale uniformly. This addon warns and aborts export if scale is non-uniform.
"""

bl_info = {
    "name": "CAUSTICFORGE",
    "author": "Bland Design",
    "version": (2, 0, 0),
    "blender": (4, 0, 0),
    "location": "View3D > N-Panel > CAUSTICFORGE",
    "description": "G-code exporter for caustic surface OBJ — spiral finishing + square cut + separate exports (v2.0)",
    "category": "Object",
}

import bpy
import math
import os
import datetime
import numpy as np
from mathutils import Vector
from bpy.props import (
    FloatProperty, IntProperty, BoolProperty,
    EnumProperty, StringProperty, PointerProperty
)
from bpy.types import PropertyGroup, Panel, Operator


# ─────────────────────────────────────────────
#  BIT PRESETS
# ─────────────────────────────────────────────

BIT_PRESETS = {
    '025_BALL': {
        'label':          '1/4" Ball Nose 2-Flute',
        'diameter':       0.25,
        'flutes':         2,
        'rpm':            18000,
        'feed_normal':    144.0,
        'plunge_normal':  20.0,
        'stepover_pct':   0.40,   # finish pass stepover — 40% dia
        'stepover_spiral_pct': 0.20,  # spiral default — 20% dia (0.050")
        'stepover_rough_pct': 0.60,   # roughing stepover — 60% dia
        'doc_rough':      0.100,  # aggressive DOC — 0.1" per level
        'stock_to_leave': 0.015,  # skin for finish pass
        'role':           'rough',
        'notes':          'Roughing. 60% stepover, 0.1" DOC — leave 0.015" skin.',
    },
    '0125_BALL': {
        'label':          '1/8" Ball Nose 2-Flute',
        'diameter':       0.125,
        'flutes':         2,
        'rpm':            18000,
        'feed_normal':    100.0,
        'plunge_normal':  14.0,
        'stepover_pct':   0.10,
        'stepover_spiral_pct': 0.20,
        'role':           'finish',
        'notes':          '1/8" finish. Good for medium-detail caustics.',
    },
    '0625_BALL': {
        'label':          '1/16" Ball Nose 2-Flute',
        'diameter':       0.0625,
        'flutes':         2,
        'rpm':            18000,
        'feed_normal':    72.0,
        'plunge_normal':  10.0,
        'stepover_pct':   0.08,
        'stepover_spiral_pct': 0.20,
        'role':           'finish',
        'notes':          '1/16" finish. High-detail caustics.',
    },
    '03125_BALL': {
        'label':          '1/32" Ball Nose 2-Flute',
        'diameter':       0.03125,
        'flutes':         2,
        'rpm':            18000,
        'feed_normal':    40.0,
        'plunge_normal':  8.0,
        'stepover_pct':   0.10,
        'stepover_spiral_pct': 0.20,
        'role':           'finish',
        'notes':          '1/32" finish — fragile. Ultra-fine detail only.',
    },
}

FINISH_BITS = ['025_BALL', '0125_BALL', '0625_BALL', '03125_BALL']

M2IN = 39.3701  # metres to inches


def stepover_for(bit_key):
    p = BIT_PRESETS[bit_key]
    return round(p['diameter'] * p['stepover_pct'], 6)


def stepover_spiral_for(bit_key):
    """Spiral-specific stepover: 20% of diameter by default (finer than raster)."""
    p = BIT_PRESETS[bit_key]
    return round(p['diameter'] * p.get('stepover_spiral_pct', 0.20), 6)


def _apply_speed_mode(props):
    mult = 1.4 if props.speed_mode == 'SUPERFAST' else 1.0
    rk = props.rough_bit
    if rk in BIT_PRESETS:
        rp = BIT_PRESETS[rk]
        props.rough_rpm       = rp['rpm']
        props.rough_feed      = round(rp['feed_normal']   * mult, 1)
        props.rough_plunge    = round(rp['plunge_normal'] * mult, 1)
        props.rough_doc       = rp['doc_rough']
        props.stock_to_leave  = rp['stock_to_leave']
    fk = props.finish_bit
    if fk in BIT_PRESETS:
        fp = BIT_PRESETS[fk]
        props.finish_rpm    = fp['rpm']
        props.finish_feed   = round(fp['feed_normal']   * mult, 1)
        props.finish_plunge = round(fp['plunge_normal'] * mult, 1)

def _upd_mode(self, ctx): _apply_speed_mode(self)
def _upd_bit(self, ctx):  _apply_speed_mode(self)


# ─────────────────────────────────────────────
#  SCALE GUARD
# ─────────────────────────────────────────────

def check_scale_uniform(obj, tol=0.001):
    """
    Returns (is_uniform, sx, sy, sz).
    Caustic lens geometry requires uniform XYZ scale — non-uniform scale
    changes dZ/dXY ratio, distorting surface normals and destroying the caustic.
    """
    s = obj.scale
    sx, sy, sz = s.x, s.y, s.z
    uniform = (abs(sx - sy) < tol and abs(sx - sz) < tol)
    return uniform, sx, sy, sz


# ─────────────────────────────────────────────
#  MESH UTILITIES — NUMPY HEIGHTFIELD
# ─────────────────────────────────────────────

def get_mesh_bounds(obj):
    """World-space bounding box."""
    import mathutils
    verts = [obj.matrix_world @ v.co for v in obj.data.vertices]
    xs = [v.x for v in verts]; ys = [v.y for v in verts]; zs = [v.z for v in verts]
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def build_heightfield(obj):
    """
    Build a numpy heightfield from the caustic surface of the mesh.

    Caustic OBJs have two vertex layers:
      - A flat base slab at constant Z (the bottom)
      - The caustic surface (small relief, the CNC target)

    This function detects both layers using Z-histogram bimodality,
    extracts only the caustic surface, and returns:
      - z_grid_in: 2D numpy array [ny, nx] of Z heights in INCHES
      - grid_meta: dict with x_min, x_max, y_min, y_max (inches),
                   gsp_x, gsp_y (inches), nx, ny, relief_in, grid_size

    For non-bimodal meshes (single continuous surface), uses all verts.
    Z values: 0.0 = stock top / dome peak, negative = cuts into stock.
    """
    mesh = obj.data
    vcount = len(mesh.vertices)
    vf = np.zeros(vcount * 3, dtype=np.float64)
    mesh.vertices.foreach_get('co', vf)
    v = vf.reshape(-1, 3)

    # Apply world matrix
    mw = np.array([[obj.matrix_world[r][c] for c in range(4)] for r in range(4)])
    ones = np.ones((vcount, 1))
    v_world = (mw @ np.hstack([v, ones]).T).T[:, :3]

    # ── Axis-swap auto-correct (Y/Z swapped pipeline output) ────────
    y_span = v_world[:,1].max() - v_world[:,1].min()
    z_span_raw = v_world[:,2].max() - v_world[:,2].min()
    if z_span_raw > y_span * 5.0 and y_span > 1e-6:
        old_y = v_world[:,1].copy()
        old_z = v_world[:,2].copy()
        v_world[:,1] = old_z
        v_world[:,2] = -old_y
        z_after = v_world[:,2]
        z_mid_after = (z_after.min() + z_after.max()) / 2
        upper_z_after = z_after[z_after >= z_mid_after]
        v_world[:,2] -= upper_z_after.max()

    z_all = v_world[:, 2]

    # ── Detect bimodal structure (base slab + caustic surface) ─────
    # Uses gap-based detection: sort Z values, find largest gap.
    # If gap is >10x the mean gap, the mesh has two distinct Z layers.
    z_min, z_max = z_all.min(), z_all.max()
    z_range = z_max - z_min

    if z_range < 1e-9:
        raise ValueError("Mesh has zero Z relief — not a valid caustic surface")

    bimodal = False
    z_sorted = np.sort(z_all)
    z_diffs = np.diff(z_sorted)
    max_gap_idx = int(np.argmax(z_diffs))
    max_gap = z_diffs[max_gap_idx]
    mean_gap = np.mean(z_diffs)

    if mean_gap > 0 and max_gap > mean_gap * 10:
        # Large gap found — likely bimodal
        split_z = (z_sorted[max_gap_idx] + z_sorted[max_gap_idx + 1]) / 2.0
        upper_mask = z_all >= split_z
        lower_mask = ~upper_mask
        n_upper = int(upper_mask.sum())
        n_lower = int(lower_mask.sum())

        # The caustic surface is the layer whose count is closest to a perfect square
        # (the slab may have slightly different count due to OBJ import artifacts)
        gs_upper = int(round(math.sqrt(n_upper)))
        gs_lower = int(round(math.sqrt(n_lower)))
        err_upper = abs(gs_upper * gs_upper - n_upper)
        err_lower = abs(gs_lower * gs_lower - n_lower)

        # Pick whichever layer is closer to a square grid — that's the structured one
        # Caustic surface is typically the upper layer (higher Z)
        if err_upper <= gs_upper:
            caustic_verts = v_world[upper_mask]
            bimodal = True
        elif err_lower <= gs_lower:
            # Upper layer is the caustic but has bad count — try lower as slab check
            # If lower is the flat slab, use upper regardless
            lower_z = z_all[lower_mask]
            if lower_z.std() < z_range * 0.1:
                caustic_verts = v_world[upper_mask]
                bimodal = True
            else:
                caustic_verts = v_world
        else:
            caustic_verts = v_world
    else:
        caustic_verts = v_world

    # ── Build grid ────────────────────────────────────────────────
    n_caustic = len(caustic_verts)
    grid_size = int(round(math.sqrt(n_caustic)))
    expected = grid_size * grid_size

    if abs(expected - n_caustic) > grid_size:
        raise ValueError(f"Caustic surface ({n_caustic} verts) is not a square grid "
                         f"(nearest: {grid_size}x{grid_size}={expected})")

    # Sort verts into grid order: ascending Y, then ascending X within each row
    # This handles OBJ imports that may not preserve vertex order
    cx = caustic_verts[:, 0]
    cy = caustic_verts[:, 1]
    cz = caustic_verts[:, 2]

    # Sort by Y first (row), then X (column) — lexicographic on (Y, X)
    sort_idx = np.lexsort((cx, cy))
    cx = cx[sort_idx]
    cy = cy[sort_idx]
    cz = cz[sort_idx]

    # Handle slight vert count mismatch (±few from OBJ import)
    if n_caustic < expected:
        # Pad by duplicating the last vertex
        pad_n = expected - n_caustic
        cx = np.concatenate([cx, np.full(pad_n, cx[-1])])
        cy = np.concatenate([cy, np.full(pad_n, cy[-1])])
        cz = np.concatenate([cz, np.full(pad_n, cz[-1])])
    elif n_caustic > expected:
        # Trim extras (likely duplicate verts at boundaries)
        cx = cx[:expected]
        cy = cy[:expected]
        cz = cz[:expected]

    x_min_m, x_max_m = cx.min(), cx.max()
    y_min_m, y_max_m = cy.min(), cy.max()
    z_peak_m         = cz.max()
    relief_m         = cz.max() - cz.min()

    z_grid_m = cz.reshape(grid_size, grid_size)

    z_grid_in = (z_grid_m - z_peak_m) * M2IN

    gsp_x = (x_max_m - x_min_m) / (grid_size - 1) * M2IN
    gsp_y = (y_max_m - y_min_m) / (grid_size - 1) * M2IN

    meta = {
        'x_min':      x_min_m * M2IN,
        'x_max':      x_max_m * M2IN,
        'y_min':      y_min_m * M2IN,
        'y_max':      y_max_m * M2IN,
        'gsp_x':      gsp_x,
        'gsp_y':      gsp_y,
        'nx':         grid_size,
        'ny':         grid_size,
        'relief_in':  relief_m * M2IN,
        'relief_mm':  relief_m * 1000,
        'grid_size':  grid_size,
        'n_verts':      n_caustic,
        'bimodal':      bimodal,
        'axis_swapped': (z_span_raw > y_span * 5.0 and y_span > 1e-6),
    }
    return z_grid_in, meta


def interp_z_in(z_grid, meta, x_in, y_in):
    """
    Bilinear interpolation of Z (inches) at world position (x_in, y_in) in inches.
    Returns Z in inches (<= 0), or None if outside grid.
    """
    nx = meta['nx']; ny = meta['ny']
    gsp_x = meta['gsp_x']; gsp_y = meta['gsp_y']
    x_min = meta['x_min']; y_min = meta['y_min']
    x_max = meta['x_max']; y_max = meta['y_max']

    if x_in < x_min or x_in > x_max or y_in < y_min or y_in > y_max:
        return None

    fx = (x_in - x_min) / gsp_x
    fy = (y_in - y_min) / gsp_y
    ix = int(math.floor(fx)); iy = int(math.floor(fy))
    ix = max(0, min(nx - 2, ix)); iy = max(0, min(ny - 2, iy))
    tx = fx - ix; ty = fy - iy

    z00 = z_grid[iy,   ix  ]
    z10 = z_grid[iy,   ix+1]
    z01 = z_grid[iy+1, ix  ]
    z11 = z_grid[iy+1, ix+1]

    return float(z00*(1-tx)*(1-ty) + z10*tx*(1-ty) +
                 z01*(1-tx)*ty     + z11*tx*ty)


# ─────────────────────────────────────────────
#  SURFACE ANALYSIS
# ─────────────────────────────────────────────

def analyse_surface(obj, props):
    """
    Analyse caustic surface geometry.
    Uses numpy heightfield — fast on large grids.
    Returns dict with bit recommendation, time estimates, geometry facts.
    """
    z_grid, meta = build_heightfield(obj)

    nx = meta['nx']; ny = meta['ny']
    gsp_x = meta['gsp_x']; gsp_y = meta['gsp_y']
    relief_in = meta['relief_in']

    # ── Curvature from finite differences on the Z grid ──────────
    step = max(1, nx // 100)

    radii_in  = []
    slopes_deg = []

    for iy in range(1, ny-1, step):
        for ix in range(1, nx-1, step):
            z_c  = z_grid[iy,   ix  ]
            z_l  = z_grid[iy,   ix-1]
            z_r  = z_grid[iy,   ix+1]
            z_d  = z_grid[iy-1, ix  ]
            z_u  = z_grid[iy+1, ix  ]

            d2x = (z_l - 2*z_c + z_r) / (gsp_x ** 2)
            d2y = (z_d - 2*z_c + z_u) / (gsp_y ** 2)
            kappa = math.sqrt(d2x**2 + d2y**2)
            if kappa > 1e-9:
                radii_in.append(1.0 / kappa)

            dzdx = (z_r - z_l) / (2 * gsp_x)
            dzdy = (z_u - z_d) / (2 * gsp_y)
            slope = math.degrees(math.atan(math.sqrt(dzdx**2 + dzdy**2)))
            slopes_deg.append(slope)

    min_r  = min(radii_in)   if radii_in   else 999.0
    mean_r = sum(radii_in)/len(radii_in) if radii_in else 999.0
    max_sl = max(slopes_deg) if slopes_deg else 0.0

    # ── Bit recommendation ────────────────────────────────────────
    safe_r = min_r * 0.7
    recommended = '03125_BALL'
    for bk in ['025_BALL', '0125_BALL', '0625_BALL', '03125_BALL']:
        bit_r = BIT_PRESETS[bk]['diameter'] / 2.0
        if bit_r <= safe_r:
            recommended = bk
            break

    # ── Machine time estimates ────────────────────────────────────
    sw = props.stock_width; sh = props.stock_height
    time_estimates = {}
    for bk in FINISH_BITS:
        so   = stepover_for(bk)
        mult = 1.4 if props.speed_mode == 'SUPERFAST' else 1.0
        feed = BIT_PRESETS[bk]['feed_normal'] * mult
        rows = sh / so if so > 0 else 1
        path_len = rows * (sw + 0.4)
        time_estimates[bk] = round(path_len / feed, 1) if feed > 0 else 0

    return {
        'relief_in':       round(relief_in, 5),
        'relief_mm':       round(meta['relief_mm'], 3),
        'min_radius_in':   round(min_r,  5),
        'mean_radius_in':  round(mean_r, 4),
        'max_slope_deg':   round(max_sl, 1),
        'safe_bit_radius': round(safe_r, 5),
        'recommended_bit': recommended,
        'time_estimates':  time_estimates,
        'n_verts':         meta['n_verts'],
        'grid_size':       meta['grid_size'],
        'bimodal':         meta['bimodal'],
        'axis_swapped':    meta.get('axis_swapped', False),
    }


# ─────────────────────────────────────────────
#  GCODE HELPERS
# ─────────────────────────────────────────────

def g1(x=None, y=None, z=None, f=None, comment=None):
    """
    G01 line with F on every move (NK105 requires F on every G01 block).
    """
    parts = ['G01']
    if x is not None: parts.append(f'X{x:.4f}')
    if y is not None: parts.append(f'Y{y:.4f}')
    if z is not None: parts.append(f'Z{z:.4f}')
    if f is not None: parts.append(f'F{f:.1f}')
    line = ' '.join(parts)
    if comment: line += f'  ({comment})'
    return line


def gcode_header(filename, pass_name, obj_name, props, meta, bit_key, rpm):
    """Standard G-code file header for any pass."""
    now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    bit_label = BIT_PRESETS[bit_key]['label'] if bit_key in BIT_PRESETS else bit_key
    swap_note = '  axis-corrected' if meta and meta.get('axis_swapped') else ''
    lines = [
        f"({filename})",
        f"(CAUSTICFORGE v2.0  caustic acrylic{swap_note})",
        f"(Generated: {now})",
        f"(Object: {obj_name})",
        f"(Pass: {pass_name})",
        f"(Stock: {props.stock_width:.4f}\" x {props.stock_height:.4f}\" x {props.stock_thickness:.4f}\")",
        f"(Tool: {bit_label})",
        f"(Machine: Blue Elephant 1325 / NK105  F on every G01  no G00)",
        "(Zero: front-left corner of stock, Z=0 at dome peak)",
        "G54", "G20", "G17 G90",
    ]
    return lines


def gcode_header_squarecut(filename, props):
    """Header for square cut file (no caustic mesh needed)."""
    now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    n_passes = math.ceil(props.sq_depth / props.sq_doc)
    tab_info = f"{props.sq_tab_count} x {props.sq_tab_width:.3f}\" wide x {props.sq_tab_height:.3f}\" tall" if props.sq_tabs_enabled else "none"
    lines = [
        f"({filename})",
        f"(CAUSTICFORGE v2.0  Square Cut)",
        f"(Generated: {now})",
        f"(Square Cut -- {props.sq_width:.3f}\" x {props.sq_height:.3f}\" x {props.sq_depth:.3f}\" deep)",
        f"(Tool: {props.sq_bit_dia:.4f}\" O-flute, {props.sq_rpm} RPM)",
        f"(Feed: {props.sq_feed:.1f} IPM  Plunge: {props.sq_plunge:.1f} IPM)",
        f"(DOC: {props.sq_doc:.3f}\"  Passes: {n_passes})",
        f"(Tabs: {tab_info})",
        f"(Machine: Blue Elephant 1325 / NK105  F on every G01  no G00)",
        "(Zero: front-left corner of square, Z=0 at stock top)",
        "G54", "G20", "G17 G90",
    ]
    return lines


# ─────────────────────────────────────────────
#  ROUGHING GENERATOR (unchanged from v1)
# ─────────────────────────────────────────────

def gen_roughing(props, obj, z_grid, meta):
    """
    Aggressive Y-raster roughing.
    - Large DOC (0.1") and large stepover (60% dia) to clear material fast
    - Surface-following floor: every sample clamps to (surface + skin)
      so the finish pass always removes a uniform skin depth
    - Stops at (cut_depth - stock_to_leave) — never touches final surface
    - Edge-entry only — no plunges into stock
    - Along-path at 4x finer than stepover for accurate floor following
    """
    sw = props.stock_width;  sh = props.stock_height
    ox = props.origin_offset_x; oy = props.origin_offset_y

    doc     = props.rough_doc
    _bp = BIT_PRESETS[props.rough_bit]
    rough_so_pct = _bp.get('stepover_rough_pct', _bp['stepover_pct'])
    so      = round(_bp['diameter'] * rough_so_pct, 6)
    stl     = props.stock_to_leave
    feed    = props.rough_feed
    plunge  = props.rough_plunge
    rpm     = props.rough_rpm
    rapid   = props.rapid_feed
    safe_z  = props.safe_height
    depth   = props.cut_depth
    bit_dia = BIT_PRESETS[props.rough_bit]['diameter']

    overshoot = max(0.30, bit_dia * 0.6 + 0.05)

    rough_floor = -(depth - stl)
    z_levels = []
    z = -doc
    while z > rough_floor + 1e-6:
        z_levels.append(round(z, 4))
        z -= doc
    if not z_levels or z_levels[-1] > rough_floor + 1e-6:
        z_levels.append(round(rough_floor, 4))
    z_levels = sorted(set(z_levels))

    y_rows = []
    y = 0.0
    while y <= sh + 1e-6:
        y_rows.append(round(y, 4))
        y += so

    sample_sp = min(so * 0.25, meta['gsp_x'])
    N_X = max(40, int((sw + 2*overshoot) / sample_sp))

    lines = [
        "",
        "(=== ROUGHING PASS ===)",
        f"(Bit: {BIT_PRESETS[props.rough_bit]['label']})",
        f"(Strategy: Y-raster  DOC: {doc:.4f}\"  Stepover: {so:.4f}\"  ({int(so/bit_dia*100)}% dia))",
        f"(Skin left for finish: {stl:.4f}\"  Rough floor: {rough_floor:.4f}\")",
        f"(Feed: {feed:.1f} IPM  Plunge: {plunge:.1f} IPM  RPM: {rpm})",
        f"(Speed mode: {props.speed_mode}  Z levels: {len(z_levels)}  Y rows/level: {len(y_rows)})",
        "(Edge-entry only — no plunges into stock)",
        f"M03 S{rpm}",
        g1(z=safe_z, f=rapid),
    ]

    direction = 1
    for z_lv in z_levels:
        lines += ["", f"(--- Z level {z_lv:.4f}\" ---)"]
        for y_pos in y_rows:
            x_start = -overshoot   if direction == 1 else sw + overshoot
            x_end   = sw+overshoot if direction == 1 else -overshoot

            lines.append(g1(x=x_start, y=y_pos, f=rapid, comment=f"row Y={y_pos:.3f}"))
            lines.append(g1(z=z_lv, f=plunge, comment="entry plunge (outside stock)"))

            for si in range(N_X + 1):
                t     = si / N_X
                x_pos = round(x_start + (x_end - x_start) * t, 4)

                sz = interp_z_in(z_grid, meta, x_pos + ox, y_pos + oy)
                if sz is not None:
                    floor_z = round(sz + stl, 4)
                    cut_z   = round(max(z_lv, floor_z), 4)
                    cut_z   = min(0.0, cut_z)
                else:
                    cut_z = z_lv

                dz    = cut_z - z_lv
                f_use = plunge if dz < -0.005 else feed
                lines.append(g1(x=x_pos, y=y_pos, z=cut_z, f=f_use))

            lines.append(g1(z=safe_z, f=rapid, comment="lift"))
            direction *= -1

    lines += ["", "(=== ROUGHING COMPLETE ===)", ""]
    return lines


# ─────────────────────────────────────────────
#  FINISHING GENERATOR — X-RASTER (unchanged from v1)
# ─────────────────────────────────────────────

def gen_finishing(props, obj, z_grid, meta):
    """
    X-raster finishing pass — 90 deg offset from Y-raster roughing.
    Along-path resolution matched to mesh gsp_y (Y grid spacing)
    since columns now traverse in Y. Eliminates directional scallop marks.
    Stay-low between columns, mini-lift on surface rises.
    """
    sw = props.stock_width;  sh = props.stock_height
    ox = props.origin_offset_x; oy = props.origin_offset_y

    so         = stepover_for(props.finish_bit)
    feed       = props.finish_feed
    plunge     = props.finish_plunge
    rpm        = props.finish_rpm
    rapid      = props.rapid_feed
    safe_z     = props.safe_height
    rth        = props.finish_retract_threshold
    steep      = props.steep_angle_threshold
    approach_h = 0.050

    bit_r     = BIT_PRESETS[props.finish_bit]['diameter'] / 2.0
    overshoot = max(0.10, bit_r + 0.02)

    x_cols = []
    x = 0.0
    while x <= sw + 1e-6:
        x_cols.append(round(x, 4))
        x += so

    sample_sp = min(so * 0.5, meta['gsp_y'])
    N_Y = max(20, int((sh + 2*overshoot) / sample_sp))

    lines = [
        "",
        "(=== FINISHING PASS — X-RASTER ===)",
        f"(Bit: {BIT_PRESETS[props.finish_bit]['label']})",
        f"(Strategy: X-raster — 90 deg offset from roughing)",
        f"(Stepover: {so:.5f}\"  locked to {BIT_PRESETS[props.finish_bit]['stepover_pct']*100:.0f}% of dia)",
        f"(Feed: {feed:.1f} IPM  Plunge: {plunge:.1f} IPM  RPM: {rpm})",
        f"(Retract threshold: {rth:.4f}\"  Stay-low: yes)",
        f"(Speed mode: {props.speed_mode}  X cols: {len(x_cols)}  Y samples/col: {N_Y})",
        f"M03 S{rpm}",
        g1(z=safe_z, f=rapid),
    ]

    direction  = 1
    prev_end_z = safe_z

    for col_idx, x_pos in enumerate(x_cols):
        if direction == 1:
            ys = [round(-overshoot + (sh+2*overshoot)*i/N_Y, 4) for i in range(N_Y+1)]
        else:
            ys = [round((sh+overshoot) - (sh+2*overshoot)*i/N_Y, 4) for i in range(N_Y+1)]

        ey_cl = max(meta['y_min'], min(meta['y_max'], ys[0] + oy))
        ex_cl = max(meta['x_min'], min(meta['x_max'], x_pos + ox))
        ez = interp_z_in(z_grid, meta, ex_cl, ey_cl)
        ez = min(0.0, round(ez, 4)) if ez is not None else 0.0

        stay_low = (col_idx > 0 and abs(ez - prev_end_z) <= rth)

        lines.append("")
        lines.append(f"(Col {col_idx}  X={x_pos:.4f}\"  {'up' if direction==1 else 'down'})")

        if stay_low:
            lines.append(g1(x=x_pos, y=ys[0], f=rapid, comment="stay-low"))
        else:
            lines.append(g1(z=safe_z, f=rapid, comment="safe height"))
            lines.append(g1(x=x_pos, y=ys[0], f=rapid))
            lines.append(g1(z=round(ez + approach_h, 4), f=rapid))

        lines.append(g1(x=x_pos, y=ys[0], z=ez, f=plunge, comment="engage"))
        z_cur = ez

        for y_pos in ys[1:]:
            wx = max(meta['x_min'], min(meta['x_max'], x_pos + ox))
            wy = max(meta['y_min'], min(meta['y_max'], y_pos + oy))
            sz = interp_z_in(z_grid, meta, wx, wy)
            tgt = min(0.0, round(sz, 4)) if sz is not None else ez

            dz = tgt - z_cur
            if dz > rth:
                lines.append(g1(z=round(tgt + approach_h, 4), f=rapid, comment="mini-lift"))
                lines.append(g1(x=x_pos, y=y_pos, f=rapid))
                lines.append(g1(z=tgt, f=plunge))
            else:
                f_use = plunge if dz < -0.005 else feed
                lines.append(g1(x=x_pos, y=y_pos, z=tgt, f=f_use))

            z_cur = tgt

        lines.append(g1(z=safe_z, f=rapid, comment="lift"))
        prev_end_z = z_cur
        direction *= -1

    lines.append(g1(z=safe_z, f=rapid, comment="program end lift"))
    lines += ["", "(=== FINISHING COMPLETE ===)", ""]
    return lines


# ─────────────────────────────────────────────
#  FINISHING GENERATOR — SPIRAL (NEW in v2)
# ─────────────────────────────────────────────

def gen_spiral_finishing(props, obj, z_grid, meta):
    """
    Archimedean spiral finishing pass.
    Outside-in, CW (climb milling), adaptive angular step.
    Z sampled from heightfield via interp_z_in().
    """
    # Parameters
    stepover = stepover_spiral_for(props.finish_bit)
    if props.spiral_stepover_override > 0:
        stepover = props.spiral_stepover_override

    feed = props.finish_feed
    plunge = props.finish_plunge
    rpm = props.finish_rpm
    rapid = props.rapid_feed
    safe_z = props.safe_height

    # Stock geometry
    sw = props.stock_width
    sh = props.stock_height
    ox = props.origin_offset_x
    oy = props.origin_offset_y
    cx_stock = sw / 2.0
    cy_stock = sh / 2.0

    # Spiral parameters
    R_start = math.sqrt(sw**2 + sh**2) / 2.0 + stepover
    R_end = stepover / 2.0
    total_revolutions = (R_start - R_end) / stepover

    # Adaptive angular step: ~0.020" linear step between consecutive points
    target_linear_step = min(stepover * 0.4, 0.020)

    lines = [
        "",
        "(=== SPIRAL FINISHING PASS ===)",
        f"(Strategy: Archimedean spiral, outside-in, CW climb)",
        f"(Bit: {BIT_PRESETS[props.finish_bit]['label']})",
        f"(Stepover: {stepover:.5f}\")",
        f"(Feed: {feed:.1f} IPM  Plunge: {plunge:.1f} IPM  RPM: {rpm})",
        f"(Revolutions: ~{total_revolutions:.0f}  R_start: {R_start:.3f}\"  R_end: {R_end:.3f}\")",
        f"(Speed mode: {props.speed_mode})",
        f"(Scallop guide: 40%=~10mil  25%=~4mil  20%=~2.6mil  15%=~1.3mil)",
        f"M03 S{rpm}",
    ]

    # Move to start position (3 o'clock)
    theta = 0.0
    r = R_start
    x_start = cx_stock + r
    y_start = cy_stock

    lines.append(g1(z=safe_z, f=rapid))
    lines.append(g1(x=round(x_start, 4), y=round(y_start, 4), f=rapid))

    # Plunge to surface at start point
    sz = interp_z_in(z_grid, meta, x_start + ox, y_start + oy)
    start_z = min(0.0, round(sz, 4)) if sz is not None else 0.0
    lines.append(g1(z=round(start_z + 0.050, 4), f=rapid))
    lines.append(g1(z=start_z, f=plunge, comment="spiral engage"))

    z_cur = start_z
    point_count = 0

    # Generate spiral points
    while r > R_end:
        # Adaptive angular step
        dtheta = target_linear_step / max(r, 0.01)
        theta -= dtheta  # Negative = CW when viewed from above
        r = R_start - (stepover / (2.0 * math.pi)) * abs(theta)

        if r < R_end:
            r = R_end

        # Spiral XY position
        x_pos = round(cx_stock + r * math.cos(theta), 4)
        y_pos = round(cy_stock + r * math.sin(theta), 4)

        # Skip points outside stock bounds
        if x_pos < 0 or x_pos > sw or y_pos < 0 or y_pos > sh:
            continue

        # Sample surface height
        wx = max(meta['x_min'], min(meta['x_max'], x_pos + ox))
        wy = max(meta['y_min'], min(meta['y_max'], y_pos + oy))
        sz = interp_z_in(z_grid, meta, wx, wy)
        tgt = min(0.0, round(sz, 4)) if sz is not None else 0.0

        dz = tgt - z_cur
        f_use = plunge if dz < -0.005 else feed
        lines.append(g1(x=x_pos, y=y_pos, z=tgt, f=f_use))
        z_cur = tgt
        point_count += 1

    # Final center point
    wx = max(meta['x_min'], min(meta['x_max'], cx_stock + ox))
    wy = max(meta['y_min'], min(meta['y_max'], cy_stock + oy))
    sz = interp_z_in(z_grid, meta, wx, wy)
    tgt = min(0.0, round(sz, 4)) if sz is not None else 0.0
    lines.append(g1(x=round(cx_stock, 4), y=round(cy_stock, 4), z=tgt, f=feed))

    lines.append(g1(z=safe_z, f=rapid, comment="spiral complete lift"))
    lines.append("")
    lines.append(f"(Spiral: {point_count} points)")
    lines.append("(=== SPIRAL FINISHING COMPLETE ===)")

    return lines


# ─────────────────────────────────────────────
#  SQUARE CUT GENERATOR (NEW in v2)
# ─────────────────────────────────────────────

def gen_squarecut(props):
    """
    Profile perimeter cut with optional tabs.
    CW direction (climb milling on external profile).
    Multi-pass Z levels from 0 to -sq_depth in steps of sq_doc.
    Tool center offset outward by half bit diameter.
    """
    w = props.sq_width
    h = props.sq_height
    depth = props.sq_depth
    doc = props.sq_doc
    bit_r = props.sq_bit_dia / 2.0
    feed = props.sq_feed
    plunge = props.sq_plunge
    rpm = props.sq_rpm
    rapid = props.sq_rapid
    safe_z = props.sq_safe_height

    # Offset rectangle (tool center path — outside the square by bit_r)
    x_min = -bit_r
    x_max = w + bit_r
    y_min = -bit_r
    y_max = h + bit_r

    # Z levels
    z_levels = []
    z = -doc
    while z > -depth + 1e-6:
        z_levels.append(round(z, 4))
        z -= doc
    if not z_levels or z_levels[-1] > -depth + 1e-6:
        z_levels.append(round(-depth, 4))
    z_levels = sorted(set(z_levels))

    # Tab positions (midpoint of each side, in tool-center coordinates)
    tabs = []
    if props.sq_tabs_enabled:
        tab_hw = props.sq_tab_width / 2.0
        tab_z_floor = round(-depth + props.sq_tab_height, 4)  # Z to lift to at tab

        # Bottom side: travels +X at y_min
        tabs.append({'side': 'bottom', 'axis': 'x', 'center': w / 2.0,
                      'y': y_min, 'hw': tab_hw, 'tab_z': tab_z_floor})
        # Right side: travels +Y at x_max
        tabs.append({'side': 'right', 'axis': 'y', 'center': h / 2.0,
                      'x': x_max, 'hw': tab_hw, 'tab_z': tab_z_floor})
        # Top side: travels -X at y_max
        tabs.append({'side': 'top', 'axis': 'x', 'center': w / 2.0,
                      'y': y_max, 'hw': tab_hw, 'tab_z': tab_z_floor})
        # Left side: travels -Y at x_min
        tabs.append({'side': 'left', 'axis': 'y', 'center': h / 2.0,
                      'x': x_min, 'hw': tab_hw, 'tab_z': tab_z_floor})

    lines = [
        "",
        f"M03 S{rpm}",
        g1(z=safe_z, f=rapid),
    ]

    def _in_tab(side, pos, z_lv):
        """Check if position is within a tab zone on the given side, at sufficient depth."""
        for t in tabs:
            if t['side'] != side:
                continue
            if abs(pos - t['center']) <= t['hw']:
                # Only activate tab on the deepest passes (within tab_height of floor)
                if z_lv <= t['tab_z']:
                    return t['tab_z']
        return None

    # Start at bottom-left corner
    for z_idx, z_lv in enumerate(z_levels):
        lines.append("")
        lines.append(f"(--- Z level {z_lv:.4f}\" ---)")

        # Move to start (bottom-left)
        lines.append(g1(x=round(x_min, 4), y=round(y_min, 4), f=rapid))

        if z_idx == 0:
            lines.append(g1(z=round(z_lv + 0.050, 4), f=rapid))
        lines.append(g1(z=z_lv, f=plunge, comment="plunge"))

        # CW rectangle: bottom(+X) -> right(+Y) -> top(-X) -> left(-Y)
        # Number of sample points per side for tab detection
        n_seg = max(20, int(max(w, h) / 0.050))

        # Bottom edge: x_min -> x_max at y_min
        for i in range(1, n_seg + 1):
            xp = round(x_min + (x_max - x_min) * i / n_seg, 4)
            # Map tool center back to stock coordinate for tab check
            stock_x = xp + bit_r  # approximate stock X
            tz = _in_tab('bottom', stock_x - bit_r, z_lv)  # use midpoint of square
            # Actually tab center is at w/2 in stock coords, tool is at x along offset path
            tz = _in_tab('bottom', xp, z_lv)
            z_use = tz if tz is not None else z_lv
            lines.append(g1(x=xp, y=round(y_min, 4), z=z_use, f=feed))

        # Right edge: y_min -> y_max at x_max
        for i in range(1, n_seg + 1):
            yp = round(y_min + (y_max - y_min) * i / n_seg, 4)
            tz = _in_tab('right', yp, z_lv)
            z_use = tz if tz is not None else z_lv
            lines.append(g1(x=round(x_max, 4), y=yp, z=z_use, f=feed))

        # Top edge: x_max -> x_min at y_max
        for i in range(1, n_seg + 1):
            xp = round(x_max - (x_max - x_min) * i / n_seg, 4)
            tz = _in_tab('top', xp, z_lv)
            z_use = tz if tz is not None else z_lv
            lines.append(g1(x=xp, y=round(y_max, 4), z=z_use, f=feed))

        # Left edge: y_max -> y_min at x_min
        for i in range(1, n_seg + 1):
            yp = round(y_max - (y_max - y_min) * i / n_seg, 4)
            tz = _in_tab('left', yp, z_lv)
            z_use = tz if tz is not None else z_lv
            lines.append(g1(x=round(x_min, 4), y=yp, z=z_use, f=feed))

        lines.append(g1(z=safe_z, f=rapid, comment="lift"))

    lines.append("")
    lines.append(f"(Square cut complete: {len(z_levels)} Z levels)")
    lines.append("(=== SQUARE CUT COMPLETE ===)")

    return lines


# ─────────────────────────────────────────────
#  MACHINE TIME ESTIMATE
# ─────────────────────────────────────────────

def estimate_time_min(bit_key, stock_w, stock_h, speed_mode):
    so   = stepover_for(bit_key)
    mult = 1.4 if speed_mode == 'SUPERFAST' else 1.0
    feed = BIT_PRESETS[bit_key]['feed_normal'] * mult
    rows = stock_h / so if so > 0 else 1
    path_len = rows * (stock_w + 0.4)
    return round(path_len / feed, 1) if feed > 0 else 0


# ─────────────────────────────────────────────
#  PROPERTY GROUP
# ─────────────────────────────────────────────

class CF_Props(PropertyGroup):

    target_object: PointerProperty(
        name="Caustic Surface", type=bpy.types.Object,
        description="Imported caustic OBJ mesh — must have uniform XYZ scale"
    )

    # Stock
    stock_width:     FloatProperty(name="Stock Width X (in)",     default=24.0, min=0.1, max=48.0, precision=4)
    stock_height:    FloatProperty(name="Stock Height Y (in)",    default=24.0, min=0.1, max=48.0, precision=4)
    stock_thickness: FloatProperty(name="Stock Thickness Z (in)", default=1.0,  min=0.1, max=6.0,  precision=4)
    cut_depth:       FloatProperty(name="Max Cut Depth (in)",     default=0.10, min=0.001, max=2.0, precision=5,
        description="Set to just past the caustic relief depth (auto-filled by Analyse Surface)")

    # Origin
    origin_offset_x: FloatProperty(name="Origin X (in)", default=0.0, min=-48.0, max=48.0, precision=4)
    origin_offset_y: FloatProperty(name="Origin Y (in)", default=0.0, min=-48.0, max=48.0, precision=4)
    origin_offset_z: FloatProperty(name="Origin Z (in)", default=0.0, min=-12.0, max=12.0, precision=4)

    # Speed
    speed_mode: EnumProperty(
        name="Speed Mode",
        items=[('NORMAL','Normal',''), ('SUPERFAST','Superfast','140% feed')],
        default='NORMAL', update=_upd_mode
    )

    # Roughing
    rough_bit: EnumProperty(
        name="Roughing Bit",
        items=[('025_BALL','1/4" Ball Nose 2-Flute','')],
        default='025_BALL', update=_upd_bit
    )
    rough_rpm:      IntProperty(  name="RPM",            default=18000)
    rough_feed:     FloatProperty(name="Feed (IPM)",     default=144.0, min=1.0, max=600.0, precision=1)
    rough_plunge:   FloatProperty(name="Plunge (IPM)",   default=20.0,  min=1.0, max=100.0, precision=1)
    rough_doc:      FloatProperty(name="DOC (in)",       default=0.050, min=0.001, max=0.5, precision=4)
    stock_to_leave: FloatProperty(name="Stock to Leave", default=0.010, min=0.0, max=0.1, precision=4)

    # Finishing
    finish_bit: EnumProperty(
        name="Finishing Bit",
        items=[
            ('025_BALL',   '1/4" Ball Nose 2-Flute',  ''),
            ('0125_BALL',  '1/8" Ball Nose 2-Flute',  ''),
            ('0625_BALL',  '1/16" Ball Nose 2-Flute', ''),
            ('03125_BALL', '1/32" Ball Nose 2-Flute', ''),
        ],
        default='025_BALL', update=_upd_bit
    )
    finish_rpm:    IntProperty(  name="RPM",          default=18000)
    finish_feed:   FloatProperty(name="Feed (IPM)",   default=72.0,  min=1.0, max=300.0, precision=1)
    finish_plunge: FloatProperty(name="Plunge (IPM)", default=10.0,  min=1.0, max=60.0,  precision=1)

    # Finishing strategy (NEW v2)
    finish_strategy: EnumProperty(
        name="Finishing Strategy",
        items=[
            ('RASTER', 'X-Raster (90 offset)', 'Standard parallel passes'),
            ('SPIRAL', 'Spiral (outside-in)', 'Archimedean spiral — no directional artifacts'),
        ],
        default='SPIRAL'
    )

    spiral_stepover_override: FloatProperty(
        name="Spiral Stepover Override (in)",
        default=0.0, min=0.0, max=0.5, precision=5,
        description="Override bit stepover for spiral pass. 0 = use bit default (20% dia). Finer = smoother but slower."
    )

    # Quality
    steep_angle_threshold: FloatProperty(
        name="Steep Threshold (deg)", default=30.0, min=5.0, max=80.0, precision=1,
        description="Z drops steeper than this use plunge feed"
    )
    finish_retract_threshold: FloatProperty(
        name="Retract Threshold (in)", default=0.010, min=0.001, max=0.5, precision=4,
        description="Mini-lift only when surface rises by more than this"
    )

    # Common
    rapid_feed:  FloatProperty(name="Rapid Feed (IPM)", default=180.0, min=10.0, max=600.0, precision=1)
    safe_height: FloatProperty(name="Safe Height (in)", default=0.200, min=0.05, max=5.0,  precision=3)

    # Pass toggles
    export_roughing:  BoolProperty(name="Roughing",  default=True)
    export_finishing: BoolProperty(name="Finishing", default=True)

    # ── Square Cut (NEW v2) ────────────────────────────────────
    show_sec_squarecut: BoolProperty(default=True)
    export_squarecut: BoolProperty(name="Square Cut", default=False)

    sq_width:  FloatProperty(name="Lens Width (in)",  default=8.0, min=0.5, max=48.0, precision=3,
        description="Width of the square lens block to cut")
    sq_height: FloatProperty(name="Lens Height (in)", default=8.0, min=0.5, max=48.0, precision=3,
        description="Height of the square lens block to cut")
    sq_depth:  FloatProperty(name="Cut Depth (in)",   default=1.01, min=0.1, max=3.0, precision=3,
        description="Total through-cut depth. Set slightly deeper than stock thickness.")
    sq_doc:    FloatProperty(name="DOC per pass (in)", default=0.15, min=0.01, max=0.5, precision=3,
        description="Depth of cut per Z level. 0.15\" conservative for acrylic.")
    sq_bit_dia: FloatProperty(name="Bit Diameter (in)", default=0.5, min=0.0625, max=1.0, precision=4,
        description="O-flute end mill diameter")
    sq_rpm:    IntProperty(name="RPM", default=20000, min=5000, max=30000,
        description="Spindle speed. 20k typical for acrylic O-flute.")
    sq_feed:   FloatProperty(name="Feed (IPM)",   default=120.0, min=10.0, max=400.0, precision=1,
        description="Profile cut feed rate. 120 IPM conservative for 0.5\" O-flute at 20k RPM.")
    sq_plunge: FloatProperty(name="Plunge (IPM)", default=36.0, min=5.0, max=100.0, precision=1,
        description="Plunge feed rate. ~30% of feed rate.")
    sq_rapid:  FloatProperty(name="Rapid (IPM)",  default=220.0, min=50.0, max=400.0, precision=1)
    sq_safe_height: FloatProperty(name="Safe Height (in)", default=0.5, min=0.1, max=2.0, precision=2,
        description="Safe Z height for square cut rapids")

    # Tabs
    sq_tabs_enabled: BoolProperty(name="Use Tabs", default=True)
    sq_tab_count: IntProperty(name="Number of Tabs", default=4, min=2, max=8,
        description="Tabs to hold the block. 4 = one per side.")
    sq_tab_width: FloatProperty(name="Tab Width (in)", default=0.5, min=0.1, max=2.0, precision=2)
    sq_tab_height: FloatProperty(name="Tab Height (in)", default=0.1, min=0.02, max=0.5, precision=3,
        description="Tab thickness remaining. Cut with flush-cut saw or hand file after.")

    # ── Output paths (v2 — separate files) ─────────────────────
    sq_output_filepath: StringProperty(name="Square Cut Output", subtype='FILE_PATH',
        default="/Users/admin/causticsEngineering/squarecut_8x8.nc")
    rough_output_filepath: StringProperty(name="Roughing Output", subtype='FILE_PATH',
        default="/Users/admin/causticsEngineering/roughing.nc")
    finish_output_filepath: StringProperty(name="Finishing Output", subtype='FILE_PATH',
        default="/Users/admin/causticsEngineering/finishing.nc")

    # Legacy single output (kept for backward compat reference)
    output_filepath: StringProperty(name="Output File", subtype='FILE_PATH',
        default="/Users/admin/causticsEngineering/inkbrush_24in_finish.nc")

    # Analysis cache
    analysis_result: StringProperty(default="")

    # Section toggles
    show_sec_stock:    BoolProperty(default=True)
    show_sec_speed:    BoolProperty(default=True)
    show_sec_analysis: BoolProperty(default=True)
    show_sec_roughing: BoolProperty(default=True)
    show_sec_finish:   BoolProperty(default=True)
    show_sec_quality:  BoolProperty(default=True)
    show_sec_output:   BoolProperty(default=True)


# ─────────────────────────────────────────────
#  OPERATORS
# ─────────────────────────────────────────────

class CF_OT_Analyse(Operator):
    bl_idname      = "cf.analyse_surface"
    bl_label       = "Analyse Surface"
    bl_description = "Detect true caustic relief, recommend bit, estimate machine time"

    def execute(self, context):
        props = context.scene.cf_props
        obj   = props.target_object
        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, "Select a caustic mesh first."); return {'CANCELLED'}

        uniform, sx, sy, sz = check_scale_uniform(obj)
        if not uniform:
            self.report({'ERROR'},
                f"Non-uniform scale detected: X={sx:.4f} Y={sy:.4f} Z={sz:.4f}. "
                f"Apply uniform scale only — non-uniform scale destroys caustic geometry.")
            return {'CANCELLED'}

        try:
            result = analyse_surface(obj, props)
        except Exception as e:
            self.report({'ERROR'}, f"Analysis failed: {e}"); return {'CANCELLED'}

        r = result
        rec_label = BIT_PRESETS[r['recommended_bit']]['label']

        props.cut_depth = round(r['relief_in'] * 1.05, 5)
        props.finish_bit = r['recommended_bit']
        _apply_speed_mode(props)

        lines = [
            f"Grid: {r['grid_size']}x{r['grid_size']}  Verts: {r['n_verts']:,}",
            f"{'Bimodal mesh (base slab detected + ignored)' if r['bimodal'] else 'Single-layer mesh'}",
            f"",
            f"Caustic relief: {r['relief_in']:.5f}\"  ({r['relief_mm']:.3f} mm)",
            f"Min feature radius: {r['min_radius_in']:.5f}\"  ({r['min_radius_in']*25.4:.3f} mm)",
            f"Mean curvature radius: {r['mean_radius_in']:.4f}\"",
            f"Max surface slope: {r['max_slope_deg']:.1f} deg",
            f"Safe bit radius: {r['safe_bit_radius']:.5f}\"",
            f"Cut depth auto-set: {props.cut_depth:.5f}\" (relief + 5%)",
            f"",
            f"RECOMMENDED: {rec_label}",
            f"",
            f"-- Machine time ({props.stock_width:.1f}\"x{props.stock_height:.1f}\" stock) --",
        ]
        for bk in FINISH_BITS:
            t_n  = estimate_time_min(bk, props.stock_width, props.stock_height, 'NORMAL')
            t_sf = estimate_time_min(bk, props.stock_width, props.stock_height, 'SUPERFAST')
            so   = stepover_for(bk)
            marker = " <- REC" if bk == r['recommended_bit'] else ""
            lines.append(f"{BIT_PRESETS[bk]['label']:24s} "
                         f"{t_n:>5.0f}min N / {t_sf:>4.0f}min SF  "
                         f"so={so:.5f}\"{marker}")

        # Auto-set output paths from object name
        obj_stem = obj.name.replace(' ', '_').lower()
        base_dir = "/Users/admin/causticsEngineering"
        props.rough_output_filepath  = f"{base_dir}/{obj_stem}_roughing.nc"
        props.finish_output_filepath = f"{base_dir}/{obj_stem}_finishing.nc"
        props.sq_output_filepath     = f"{base_dir}/{obj_stem}_squarecut.nc"

        t_rec_n  = estimate_time_min(r['recommended_bit'], props.stock_width, props.stock_height, 'NORMAL')
        t_rec_sf = estimate_time_min(r['recommended_bit'], props.stock_width, props.stock_height, 'SUPERFAST')
        swap_note = "  (axis-swap corrected)" if r.get('axis_swapped') else ""
        header = [
            f"CNC READY{swap_note}",
            f"  Relief {r['relief_mm']:.3f}mm = {r['relief_in']:.5f}\"   Depth {props.cut_depth:.5f}\"",
            f"  {rec_label}   ~{t_rec_n:.0f}min N / ~{t_rec_sf:.0f}min SF",
            f"",
        ]
        props.analysis_result = "\n".join(header + lines)
        self.report({'INFO'}, f"Analysis done. Relief: {r['relief_mm']:.3f}mm. Recommended: {rec_label}")
        return {'FINISHED'}


class CF_OT_ApplyMode(Operator):
    bl_idname = "cf.apply_speed_mode"; bl_label = "Refresh Feeds"
    def execute(self, context):
        _apply_speed_mode(context.scene.cf_props)
        self.report({'INFO'}, "Feeds updated."); return {'FINISHED'}


class CF_OT_Inspect(Operator):
    bl_idname = "cf.inspect_object"; bl_label = "Inspect Bounds"
    def execute(self, context):
        props = context.scene.cf_props
        obj   = props.target_object
        if obj is None or obj.type != 'MESH':
            self.report({'WARNING'}, "Select a mesh first."); return {'CANCELLED'}
        min_x,max_x,min_y,max_y,min_z,max_z = get_mesh_bounds(obj)
        uniform, sx, sy, sz = check_scale_uniform(obj)
        nv = len(obj.data.vertices)
        msg = (f"{obj.name}  {nv:,}v  "
               f"W={max_x-min_x:.4f}\" H={max_y-min_y:.4f}\" "
               f"Z=[{min_z:.4f}\", {max_z:.4f}\"]  "
               f"Scale:{'uniform' if uniform else f'NON-UNIFORM X={sx:.3f} Y={sy:.3f} Z={sz:.3f}'}")
        self.report({'INFO'}, msg)
        def _draw(s, c): s.layout.label(text=msg)
        context.window_manager.popup_menu(_draw, title="Object Bounds", icon='INFO')
        return {'FINISHED'}


# ── Separate export operators (NEW v2) ────────────────────

def _resolve_path(raw_path):
    """Resolve output path, handling Blender relative paths."""
    raw = raw_path.strip()
    if raw.startswith('//') and not bpy.data.is_saved:
        import pathlib
        return str(pathlib.Path.home() / 'Desktop' / 'caustic_cut.nc'), True
    else:
        return bpy.path.abspath(raw), False


def _write_nc(filepath, lines, report_fn):
    """Write G-code lines to file, creating directories as needed."""
    try:
        d = os.path.dirname(filepath)
        if d: os.makedirs(d, exist_ok=True)
        with open(filepath, 'w') as f:
            f.write('\n'.join(lines))
        report_fn({'INFO'}, f"Exported {len(lines)} lines -> {os.path.basename(filepath)}")
        return True
    except Exception as e:
        report_fn({'ERROR'}, f"Write failed: {e}")
        return False


class CF_OT_ExportSquareCut(Operator):
    bl_idname = "cf.export_squarecut"
    bl_label = "Export Square Cut"
    bl_description = "Generate square cut perimeter G-code"

    def execute(self, context):
        props = context.scene.cf_props
        if not props.export_squarecut:
            self.report({'ERROR'}, "Square cut is disabled."); return {'CANCELLED'}

        fp, fallback = _resolve_path(props.sq_output_filepath)
        if fallback:
            self.report({'WARNING'}, "Blend unsaved — writing to Desktop")
        if not fp:
            self.report({'ERROR'}, "Set square cut output file path."); return {'CANCELLED'}

        filename = os.path.basename(fp)
        lines = gcode_header_squarecut(filename, props)
        lines.extend(gen_squarecut(props))
        lines += ["", "M05", "M30"]

        if _write_nc(fp, lines, self.report):
            return {'FINISHED'}
        return {'CANCELLED'}


class CF_OT_ExportRoughing(Operator):
    bl_idname = "cf.export_roughing"
    bl_label = "Export Roughing"
    bl_description = "Generate roughing pass G-code"

    def execute(self, context):
        props = context.scene.cf_props
        obj = props.target_object
        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, "No caustic mesh selected."); return {'CANCELLED'}
        if not props.export_roughing:
            self.report({'ERROR'}, "Roughing is disabled."); return {'CANCELLED'}

        uniform, sx, sy, sz = check_scale_uniform(obj)
        if not uniform:
            self.report({'ERROR'},
                f"Non-uniform scale X={sx:.4f} Y={sy:.4f} Z={sz:.4f}. "
                f"Apply scale (Ctrl+A) uniformly first.")
            return {'CANCELLED'}

        try:
            z_grid, meta = build_heightfield(obj)
        except Exception as e:
            self.report({'ERROR'}, f"Heightfield build failed: {e}"); return {'CANCELLED'}

        fp, fallback = _resolve_path(props.rough_output_filepath)
        if fallback:
            self.report({'WARNING'}, "Blend unsaved — writing to Desktop")
        if not fp:
            self.report({'ERROR'}, "Set roughing output file path."); return {'CANCELLED'}

        filename = os.path.basename(fp)
        lines = gcode_header(filename, "ROUGHING", obj.name, props, meta,
                             props.rough_bit, props.rough_rpm)

        try:
            lines.extend(gen_roughing(props, obj, z_grid, meta))
        except Exception as e:
            self.report({'ERROR'}, f"Roughing: {e}"); return {'CANCELLED'}

        lines += ["", "M05", "M30"]

        if _write_nc(fp, lines, self.report):
            return {'FINISHED'}
        return {'CANCELLED'}


class CF_OT_ExportFinishing(Operator):
    bl_idname = "cf.export_finishing"
    bl_label = "Export Finishing"
    bl_description = "Generate finishing pass G-code (raster or spiral)"

    def execute(self, context):
        props = context.scene.cf_props
        obj = props.target_object
        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, "No caustic mesh selected."); return {'CANCELLED'}
        if not props.export_finishing:
            self.report({'ERROR'}, "Finishing is disabled."); return {'CANCELLED'}

        uniform, sx, sy, sz = check_scale_uniform(obj)
        if not uniform:
            self.report({'ERROR'},
                f"Non-uniform scale X={sx:.4f} Y={sy:.4f} Z={sz:.4f}. "
                f"Apply scale (Ctrl+A) uniformly first.")
            return {'CANCELLED'}

        try:
            z_grid, meta = build_heightfield(obj)
        except Exception as e:
            self.report({'ERROR'}, f"Heightfield build failed: {e}"); return {'CANCELLED'}

        fp, fallback = _resolve_path(props.finish_output_filepath)
        if fallback:
            self.report({'WARNING'}, "Blend unsaved — writing to Desktop")
        if not fp:
            self.report({'ERROR'}, "Set finishing output file path."); return {'CANCELLED'}

        strategy = props.finish_strategy
        pass_name = f"FINISHING-{strategy}"
        filename = os.path.basename(fp)
        lines = gcode_header(filename, pass_name, obj.name, props, meta,
                             props.finish_bit, props.finish_rpm)

        try:
            if strategy == 'SPIRAL':
                lines.extend(gen_spiral_finishing(props, obj, z_grid, meta))
            else:
                lines.extend(gen_finishing(props, obj, z_grid, meta))
        except Exception as e:
            self.report({'ERROR'}, f"Finishing: {e}"); return {'CANCELLED'}

        lines += ["", "M05", "M30"]

        if _write_nc(fp, lines, self.report):
            return {'FINISHED'}
        return {'CANCELLED'}


class CF_OT_ExportAll(Operator):
    bl_idname = "cf.export_all"
    bl_label = "Export All Enabled"
    bl_description = "Export all enabled passes to separate .nc files"

    def execute(self, context):
        props = context.scene.cf_props
        exported = 0

        if props.export_squarecut:
            bpy.ops.cf.export_squarecut()
            exported += 1

        if props.export_roughing:
            bpy.ops.cf.export_roughing()
            exported += 1

        if props.export_finishing:
            bpy.ops.cf.export_finishing()
            exported += 1

        if exported == 0:
            self.report({'ERROR'}, "Enable at least one pass."); return {'CANCELLED'}

        self.report({'INFO'}, f"Exported {exported} file(s).")
        return {'FINISHED'}


# ─────────────────────────────────────────────
#  PANEL
# ─────────────────────────────────────────────

class CF_PT_Main(Panel):
    bl_label       = "CAUSTICFORGE v2"
    bl_idname      = "CF_PT_main"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'CAUSTICFORGE'

    def draw(self, context):
        layout = self.layout
        props  = context.scene.cf_props

        def sec(prop, label, icon='NONE'):
            box = layout.box()
            row = box.row(align=True)
            exp = getattr(props, prop)
            row.prop(props, prop, icon='TRIA_DOWN' if exp else 'TRIA_RIGHT',
                     icon_only=True, emboss=False)
            row.label(text=label, icon=icon)
            return box, exp

        def cl_label(box, feed, rpm, flutes):
            cl = feed/(rpm*flutes) if rpm>0 else 0
            r = box.column(align=True); r.scale_y = 0.7
            r.label(text=f"  Chipload: {cl:.5f}\" per tooth")

        # ── 1. Stock + Object ──────────────────────────────────────
        box, exp = sec('show_sec_stock', "Stock + Object", 'CUBE')
        if exp:
            box.prop(props, 'target_object')
            if props.target_object:
                uniform, sx, sy, sz = check_scale_uniform(props.target_object)
                if not uniform:
                    w = box.column(align=True); w.alert = True
                    w.label(text=f"  NON-UNIFORM SCALE: X={sx:.3f} Y={sy:.3f} Z={sz:.3f}", icon='ERROR')
                    w.label(text="  Ctrl+A > Apply Scale (uniform only)")
                    w.label(text="  Non-uniform scale destroys caustic optics")
                row = box.row(align=True)
                row.operator("cf.inspect_object",  icon='INFO',     text="Inspect")
                row.operator("cf.analyse_surface", icon='VIEWZOOM', text="Analyse")

            sub = box.box(); sub.label(text="Stock")
            c = sub.column(align=True)
            c.prop(props, 'stock_width'); c.prop(props, 'stock_height')
            c.prop(props, 'stock_thickness'); c.prop(props, 'cut_depth')
            r = box.column(align=True); r.scale_y = 0.75
            r.label(text=f"  {props.stock_width:.3f}\" x {props.stock_height:.3f}\" x {props.stock_thickness:.3f}\"")
            r.label(text=f"  Cut depth: {props.cut_depth:.5f}\"  ({props.cut_depth*25.4:.4f} mm)")

            sub2 = box.box(); sub2.label(text="Origin Offset")
            c2 = sub2.column(align=True)
            c2.prop(props, 'origin_offset_x')
            c2.prop(props, 'origin_offset_y')
            c2.prop(props, 'origin_offset_z')

        # ── 2. Square Cut (NEW v2) ────────────────────────────────
        box, exp = sec('show_sec_squarecut', "Square Cut", 'MESH_PLANE')
        if exp:
            box.prop(props, 'export_squarecut', toggle=True,
                     icon='CHECKMARK' if props.export_squarecut else 'PANEL_CLOSE',
                     text="Enable Square Cut")
            if props.export_squarecut:
                sub = box.box(); sub.label(text="Lens Block Size")
                c = sub.column(align=True)
                c.prop(props, 'sq_width'); c.prop(props, 'sq_height')
                c.prop(props, 'sq_depth'); c.prop(props, 'sq_doc')

                sub2 = box.box(); sub2.label(text="Tool + Feeds")
                c2 = sub2.column(align=True)
                c2.prop(props, 'sq_bit_dia'); c2.prop(props, 'sq_rpm')
                c2.prop(props, 'sq_feed'); c2.prop(props, 'sq_plunge')
                c2.prop(props, 'sq_rapid'); c2.prop(props, 'sq_safe_height')
                # Chipload info
                cl = props.sq_feed / (props.sq_rpm * 1) if props.sq_rpm > 0 else 0  # 1 flute O-flute
                r = sub2.column(align=True); r.scale_y = 0.7
                r.label(text=f"  Chipload: {cl:.5f}\" (1-flute O-flute)")

                sub3 = box.box(); sub3.label(text="Tabs")
                sub3.prop(props, 'sq_tabs_enabled')
                if props.sq_tabs_enabled:
                    c3 = sub3.column(align=True)
                    c3.prop(props, 'sq_tab_count')
                    c3.prop(props, 'sq_tab_width'); c3.prop(props, 'sq_tab_height')

                r2 = box.column(align=True); r2.scale_y = 0.7
                n_passes = math.ceil(props.sq_depth / props.sq_doc)
                r2.label(text=f"  {n_passes} Z passes at {props.sq_doc:.3f}\" DOC")
                r2.label(text=f"  Zero: front-left corner of square, Z=0 at stock top")

        # ── 3. Surface Analysis ────────────────────────────────────
        box, exp = sec('show_sec_analysis', "Surface Analysis", 'VIEWZOOM')
        if exp:
            box.operator("cf.analyse_surface", icon='VIEWZOOM',
                         text="Run Analysis  (auto-sets cut depth + bit)")
            if props.analysis_result:
                sub = box.box()
                for line in props.analysis_result.split("\n"):
                    r = sub.row(); r.scale_y = 0.72
                    if "RECOMMENDED" in line or "<- REC" in line:
                        r.alert = True
                    elif "NON-UNIFORM" in line:
                        r.alert = True
                    r.label(text=line)
            else:
                r = box.column(align=True); r.scale_y = 0.75
                r.label(text="  Run analysis to detect true relief depth,")
                r.label(text="  get bit recommendation + machine time estimates.")

        # ── 4. Speed Mode ──────────────────────────────────────────
        box, exp = sec('show_sec_speed', "Speed Mode", 'DRIVER')
        if exp:
            box.prop(props, 'speed_mode', expand=True)
            box.operator("cf.apply_speed_mode", icon='FILE_REFRESH', text="Refresh Feeds")
            r = box.column(align=True); r.scale_y = 0.7
            r.label(text="  Normal: conservative chipload — start here")
            r.label(text="  Superfast: 140% feed — confirm workholding first")
            box.prop(props, 'rapid_feed')
            box.prop(props, 'safe_height')

        # ── 5. Roughing ────────────────────────────────────────────
        box, exp = sec('show_sec_roughing', "Roughing Pass", 'MOD_BEVEL')
        if exp:
            box.prop(props, 'export_roughing', toggle=True,
                     icon='CHECKMARK' if props.export_roughing else 'PANEL_CLOSE',
                     text="Include Roughing Pass")
            if props.export_roughing:
                box.prop(props, 'rough_bit')
                so_r = stepover_for(props.rough_bit)
                r = box.column(align=True); r.scale_y = 0.7
                r.label(text=f"  {BIT_PRESETS[props.rough_bit]['notes']}")
                r.label(text=f"  Stepover: {so_r:.4f}\" ({BIT_PRESETS[props.rough_bit]['stepover_pct']*100:.0f}% of dia)")
                sub = box.box(); sub.label(text="Feeds + Speeds")
                c = sub.column(align=True)
                c.prop(props, 'rough_rpm'); c.prop(props, 'rough_feed'); c.prop(props, 'rough_plunge')
                cl_label(sub, props.rough_feed, props.rough_rpm,
                         BIT_PRESETS[props.rough_bit].get('flutes', 2))
                sub2 = box.box(); sub2.label(text="Toolpath")
                c2 = sub2.column(align=True)
                c2.prop(props, 'rough_doc'); c2.prop(props, 'stock_to_leave')
                if props.cut_depth > 0 and props.rough_doc > 0:
                    np_ = math.ceil(props.cut_depth / props.rough_doc)
                    nr_ = math.ceil(props.stock_height / so_r) if so_r > 0 else 0
                    r2 = box.column(align=True); r2.scale_y = 0.75
                    r2.label(text=f"  ~{np_} Z levels x ~{nr_} rows")

        # ── 6. Finishing ───────────────────────────────────────────
        box, exp = sec('show_sec_finish', "Finishing Pass", 'SMOOTHCURVE')
        if exp:
            box.prop(props, 'export_finishing', toggle=True,
                     icon='CHECKMARK' if props.export_finishing else 'PANEL_CLOSE',
                     text="Include Finishing Pass")
            if props.export_finishing:
                box.prop(props, 'finish_strategy', expand=True)
                box.prop(props, 'finish_bit')
                dia = BIT_PRESETS[props.finish_bit]['diameter']

                if props.finish_strategy == 'SPIRAL':
                    so_eff = stepover_spiral_for(props.finish_bit)
                    if props.spiral_stepover_override > 0:
                        so_eff = props.spiral_stepover_override
                    so_pct = (so_eff / dia * 100) if dia > 0 else 0
                    r = box.column(align=True); r.scale_y = 0.7
                    r.label(text=f"  {BIT_PRESETS[props.finish_bit]['notes']}")
                    r.label(text=f"  Dia {dia:.5f}\"  ({dia*25.4:.4f} mm)")
                    r.label(text=f"  Spiral stepover: {so_eff:.5f}\" ({so_pct:.0f}% of dia)")
                    r.label(text="  Scallop guide:")
                    r.label(text="    40% (0.100\") ~10mil | 25% (0.063\") ~4mil")
                    r.label(text="    20% (0.050\") ~2.6mil | 15% (0.038\") ~1.3mil")
                    box.prop(props, 'spiral_stepover_override')
                else:
                    so_f = stepover_for(props.finish_bit)
                    r = box.column(align=True); r.scale_y = 0.7
                    r.label(text=f"  {BIT_PRESETS[props.finish_bit]['notes']}")
                    r.label(text=f"  Dia {dia:.5f}\"  ({dia*25.4:.4f} mm)")
                    r.label(text=f"  Stepover: {so_f:.5f}\" ({BIT_PRESETS[props.finish_bit]['stepover_pct']*100:.0f}% of dia)")
                    t_n = estimate_time_min(props.finish_bit, props.stock_width,
                                            props.stock_height, props.speed_mode)
                    r.label(text=f"  Est. time: ~{t_n:.0f} min ({props.speed_mode})")

                sub = box.box(); sub.label(text="Feeds + Speeds")
                c = sub.column(align=True)
                c.prop(props, 'finish_rpm'); c.prop(props, 'finish_feed'); c.prop(props, 'finish_plunge')
                cl_label(sub, props.finish_feed, props.finish_rpm,
                         BIT_PRESETS[props.finish_bit].get('flutes', 2))
                sub2 = box.box(); sub2.label(text="Toolpath")
                c2 = sub2.column(align=True)
                c2.prop(props, 'finish_retract_threshold')

        # ── 7. Quality ─────────────────────────────────────────────
        box, exp = sec('show_sec_quality', "Toolpath Quality", 'CURVE_PATH')
        if exp:
            box.prop(props, 'steep_angle_threshold')
            r = box.column(align=True); r.scale_y = 0.7
            r.label(text="  Z drops steeper than threshold use plunge feed")
            r.label(text="  F on every G01 line — NK105 requirement")
            r.label(text="  Stay-low between rows — minimises retracts")

        # ── 8. Output ──────────────────────────────────────────────
        box, exp = sec('show_sec_output', "Output", 'FILE_TEXT')
        if exp:
            r = box.column(align=True); r.scale_y = 0.7
            r.label(text="  Each pass exports as a separate .nc file")
            r.label(text="  Each file is a complete standalone program")

            if props.export_squarecut:
                sub = box.box()
                sub.label(text="Square Cut", icon='MESH_PLANE')
                sub.prop(props, 'sq_output_filepath')
                sub.operator("cf.export_squarecut", icon='FILE_TEXT', text="Export Square Cut")

            if props.export_roughing:
                sub = box.box()
                sub.label(text="Roughing", icon='MOD_BEVEL')
                sub.prop(props, 'rough_output_filepath')
                sub.operator("cf.export_roughing", icon='FILE_TEXT', text="Export Roughing")

            if props.export_finishing:
                sub = box.box()
                strat = "Spiral" if props.finish_strategy == 'SPIRAL' else "X-Raster"
                sub.label(text=f"Finishing ({strat})", icon='SMOOTHCURVE')
                sub.prop(props, 'finish_output_filepath')
                sub.operator("cf.export_finishing", icon='FILE_TEXT', text="Export Finishing")

            box.separator()
            row = box.row(align=True)
            row.prop(props, 'export_squarecut', toggle=True, icon='MESH_PLANE', text="SqCut")
            row.prop(props, 'export_roughing',  toggle=True, icon='MOD_BEVEL',  text="Rough")
            row.prop(props, 'export_finishing', toggle=True, icon='SMOOTHCURVE', text="Finish")
            box.operator("cf.export_all", icon='EXPORT', text="Export All Enabled")


# ─────────────────────────────────────────────
#  REGISTRATION
# ─────────────────────────────────────────────

classes = [
    CF_Props,
    CF_OT_Analyse, CF_OT_ApplyMode, CF_OT_Inspect,
    CF_OT_ExportSquareCut, CF_OT_ExportRoughing, CF_OT_ExportFinishing, CF_OT_ExportAll,
    CF_PT_Main,
]

def register():
    for cls in classes: bpy.utils.register_class(cls)
    bpy.types.Scene.cf_props = PointerProperty(type=CF_Props)

def unregister():
    for cls in reversed(classes): bpy.utils.unregister_class(cls)
    del bpy.types.Scene.cf_props

if __name__ == "__main__": register()
