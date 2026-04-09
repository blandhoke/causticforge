# HANDOFF — CausticForge v2.0: Spiral Finishing + Square Cut + Separate Exports
**Date:** 2026-04-08  
**Author:** Tier 1 (Claude Chat)  
**Target:** Claude Code  
**Addon file:** `/Users/admin/Documents/Claude/BlenderAddons/causticforge/causticforge_v1.py`  
**Output:** `causticforge_v2.py` (new version, do not overwrite v1)

---

## Overview

Three major additions to CausticForge:

1. **Spiral finishing pass** — Archimedean spiral, outside-in, CW (climb), replaces or supplements the existing X-raster finishing
2. **Square cut module** — Profile cut to size acrylic blocks before caustic milling
3. **Separate file exports** — Three separate .nc files instead of one combined file

Everything else (analysis, roughing, heightfield, scale guard, NK105 conventions) stays unchanged.

---

## Source code reference

Read the existing `causticforge_v1.py` at `/Users/admin/Documents/Claude/BlenderAddons/causticforge/causticforge_v1.py` before making any changes. The new version should be `causticforge_v2.py` in the same directory. Copy v1 as the starting point, then modify.

Read the sample G-code at `/mnt/user-data/uploads/circleprofile_plunge18_travel220_F150.nc` for NK105 convention reference (G54, G20, G17 G90, M03 S18000, G01 with F codes).

---

## FEATURE 1: Spiral Finishing Pass

### New function: `gen_spiral_finishing()`

Generates an Archimedean spiral toolpath that follows the caustic surface height field. Sits alongside the existing `gen_finishing()` (X-raster), selectable via UI toggle.

**Spiral geometry:**

```
r(θ) = R_start - (stepover / (2π)) × θ
```

Where `R_start` is half the stock diagonal: `sqrt(stock_width² + stock_height²) / 2`

The spiral starts at the outer edge of the stock and winds inward toward the center. At each point along the spiral, the Z height is sampled from the heightfield via `interp_z_in()`.

**Direction:** Clockwise (CW) when viewed from above. This is climb milling for the outer engagement — confirmed best practice for acrylic surface finish.

**Algorithm:**

```python
def gen_spiral_finishing(props, obj, z_grid, meta):
    # Parameters
    stepover = stepover_for(props.finish_bit)  # Use finish bit stepover
    if props.spiral_stepover_override > 0:
        stepover = props.spiral_stepover_override  # Allow manual override
    
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
    cx_stock = sw / 2.0  # Center of stock
    cy_stock = sh / 2.0
    
    # Spiral parameters
    R_start = math.sqrt(sw**2 + sh**2) / 2.0 + stepover  # Start just outside corners
    R_end = stepover / 2.0  # End near center
    total_revolutions = (R_start - R_end) / stepover
    
    # Angular step: controls point density along spiral
    # Target ~0.020" linear step between consecutive points
    # At radius r, arc length per radian = r
    # So dθ = target_step / r (adaptive — tighter near center)
    target_linear_step = min(stepover * 0.4, 0.020)
    
    lines = [
        "",
        "(=== SPIRAL FINISHING PASS ===)",
        f"(Strategy: Archimedean spiral, outside-in, CW climb)",
        f"(Stepover: {stepover:.5f}\")",
        f"(Feed: {feed:.1f} IPM  Plunge: {plunge:.1f} IPM  RPM: {rpm})",
        f"(Revolutions: ~{total_revolutions:.0f}  R_start: {R_start:.3f}\"  R_end: {R_end:.3f}\")",
        f"M03 S{rpm}",
    ]
    
    # Move to start position
    theta = 0.0
    r = R_start
    x_start = cx_stock + r  # Start at 3 o'clock position
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
        
        # Clamp to stock bounds (spiral overshoots on square stock corners)
        if x_pos < 0 or x_pos > sw or y_pos < 0 or y_pos > sh:
            continue  # Skip points outside stock
        
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
```

**Key details:**
- Points outside the stock rectangle are skipped (spiral inscribes a circle, stock is square — corners are handled by roughing)
- Adaptive angular step: tighter near center (smaller radius = smaller arc length per degree), constant linear step ~0.020" between G1 points
- Z sampling uses existing `interp_z_in()` — no new heightfield code needed
- Plunge feed triggered when Z drops more than 0.005" between consecutive points
- F code on every G01 line (NK105 requirement — already established in `g1()` helper)

### UI additions for spiral finishing

Add to `CF_Props`:

```python
finish_strategy: EnumProperty(
    name="Finishing Strategy",
    items=[
        ('RASTER', 'X-Raster (90° offset)', 'Standard parallel passes'),
        ('SPIRAL', 'Spiral (outside-in)', 'Archimedean spiral — no directional artifacts'),
    ],
    default='SPIRAL'
)

spiral_stepover_override: FloatProperty(
    name="Spiral Stepover Override (in)",
    default=0.0, min=0.0, max=0.5, precision=5,
    description="Override bit stepover for spiral pass. 0 = use bit default. Finer = smoother but slower."
)
```

**Recommended stepover presets (add as info text in the panel):**
- 40% (0.100") — fast, visible scallops (~10 mil) — roughing only
- 25% (0.063") — moderate, ~4 mil scallops
- 20% (0.050") — recommended for caustics, ~2.6 mil scallops (DEFAULT)
- 15% (0.038") — optical quality, ~1.3 mil scallops, flame polish only

**Change the default stepover for 1/4" ball nose in finish mode** to 20% (0.050") instead of 40% (0.100"). This is specifically for the spiral pass — raster keeps 40% for backward compatibility.

Update the finishing section in the panel to show the strategy toggle and stepover info.

---

## FEATURE 2: Square Cut Module

### New section in the panel: "Square Cut"

A profile/perimeter cut to cut acrylic sheet into square blocks before caustic milling. Separate from the caustic operations.

**New properties:**

```python
# Square Cut section
show_sec_squarecut: BoolProperty(default=True)
export_squarecut: BoolProperty(name="Square Cut", default=False)

sq_width: FloatProperty(name="Lens Width (in)", default=8.0, min=0.5, max=48.0, precision=3,
    description="Width of the square lens block to cut")
sq_height: FloatProperty(name="Lens Height (in)", default=8.0, min=0.5, max=48.0, precision=3,
    description="Height of the square lens block to cut")
sq_depth: FloatProperty(name="Cut Depth (in)", default=1.01, min=0.1, max=3.0, precision=3,
    description="Total through-cut depth. Set slightly deeper than stock thickness.")
sq_doc: FloatProperty(name="DOC per pass (in)", default=0.15, min=0.01, max=0.5, precision=3,
    description="Depth of cut per Z level. 0.15\" conservative for acrylic.")
sq_bit_dia: FloatProperty(name="Bit Diameter (in)", default=0.5, min=0.0625, max=1.0, precision=4,
    description="O-flute end mill diameter")
sq_rpm: IntProperty(name="RPM", default=20000, min=5000, max=30000,
    description="Spindle speed. 20k typical for acrylic O-flute.")
sq_feed: FloatProperty(name="Feed (IPM)", default=120.0, min=10.0, max=400.0, precision=1,
    description="Profile cut feed rate. 120 IPM conservative for 0.5\" O-flute at 20k RPM.")
sq_plunge: FloatProperty(name="Plunge (IPM)", default=36.0, min=5.0, max=100.0, precision=1,
    description="Plunge feed rate. ~30% of feed rate.")
sq_rapid: FloatProperty(name="Rapid (IPM)", default=220.0, min=50.0, max=400.0, precision=1)
sq_safe_height: FloatProperty(name="Safe Height (in)", default=0.5, min=0.1, max=2.0, precision=2,
    description="Safe Z height for square cut rapids")

# Tabs
sq_tabs_enabled: BoolProperty(name="Use Tabs", default=True)
sq_tab_count: IntProperty(name="Number of Tabs", default=4, min=2, max=8,
    description="Tabs to hold the block. 4 = one per side.")
sq_tab_width: FloatProperty(name="Tab Width (in)", default=0.5, min=0.1, max=2.0, precision=2)
sq_tab_height: FloatProperty(name="Tab Height (in)", default=0.1, min=0.02, max=0.5, precision=3,
    description="Tab thickness remaining. Cut with flush-cut saw or hand file after.")

sq_output_filepath: StringProperty(name="Square Cut Output", subtype='FILE_PATH',
    default="/Users/admin/causticsEngineering/squarecut_8x8.nc")
```

### New function: `gen_squarecut()`

**Algorithm:**

1. Compute the rectangle perimeter path, offset outward by half the bit diameter (tool center follows this path, cutting edge cuts the exact square)
2. At each Z level (0 to -sq_depth in steps of sq_doc):
   - Move to start position (lower-left corner of the offset rectangle, at safe Z)
   - Plunge to current Z level
   - CW rectangle path (climb milling): bottom → right → top → left
   - At tab locations: raise Z to -(sq_depth - sq_tab_height) for the width of the tab
3. Retract to safe Z between Z levels
4. Final pass at full depth with tabs

**Tab placement:**
- 4 tabs: one at the midpoint of each side
- Bottom tab: X = sq_width/2, Y = 0 (offset)
- Right tab: X = sq_width (offset), Y = sq_height/2
- Top tab: X = sq_width/2, Y = sq_height (offset)
- Left tab: X = 0 (offset), Y = sq_height/2

**At tab locations:** When the tool is within ±(tab_width/2) of a tab center AND at the final Z level (or within tab_height of final depth), the tool lifts to tab_height above final depth. This leaves the tab material in place.

**CW direction for a rectangle (climb milling on external profile):**
Starting from bottom-left corner, moving right:
- Bottom edge: +X direction
- Right edge: +Y direction  
- Top edge: -X direction
- Left edge: -Y direction

This matches the CW convention in the sample G-code file.

**G-code structure:**
```gcode
(Square Cut — 8.000" x 8.000" x 1.010" deep)
(Tool: 0.500" O-flute, 20000 RPM)
(Feed: 120 IPM  Plunge: 36 IPM)
(DOC: 0.150"  Passes: 7)
(Tabs: 4 x 0.500" wide x 0.100" tall)
G54
G20
G17 G90
M03 S20000
G01 Z0.5000 F220.0
... perimeter passes ...
M05
M30
```

**WCS zero:** Top of stock, centered on the square to be cut. Or front-left corner of the square — match whatever the user zeros to. Add a note in the panel: "Zero: front-left corner of square, Z=0 at stock top."

Actually, to be consistent with CausticForge's existing convention (zero at front-left corner of stock, Z=0 at top), keep the same convention.

---

## FEATURE 3: Separate File Exports

### Current behavior
One export button, one .nc file containing both roughing and finishing (if both enabled).

### New behavior
Three separate export buttons and file paths:

```python
sq_output_filepath: StringProperty(...)      # Square cut
rough_output_filepath: StringProperty(...)   # Roughing
finish_output_filepath: StringProperty(...)  # Finishing (raster or spiral)
```

Each produces an independent .nc file with its own header, spindle start, and M30 end. Each file is a complete, self-contained program the NK105 can run independently.

**Default naming convention:**
- `{object_name}_squarecut.nc`
- `{object_name}_roughing.nc`
- `{object_name}_finishing.nc`

**Update the Output section of the panel:**
- Show three file path fields (only for enabled passes)
- Three export buttons: "Export Square Cut", "Export Roughing", "Export Finishing"
- Or keep one "Export All" button that writes all enabled passes to their respective files

### Each exported file gets a full header:
```gcode
({filename})
(CAUSTICFORGE v2.0)
(Generated: {date time})
(Object: {obj.name})
(Pass: ROUGHING / FINISHING-SPIRAL / FINISHING-RASTER / SQUARE-CUT)
(Stock: {w}" x {h}" x {t}")
(Tool: {bit label})
(Machine: Blue Elephant 1325 / NK105  F on every G01  no G00)
G54
G20
G17 G90
M03 S{rpm}
... toolpath ...
M05
M30
```

---

## Panel layout update

The panel sections should be ordered:

1. **Stock + Object** (existing)
2. **Square Cut** (NEW — collapsible, disabled by default)
3. **Surface Analysis** (existing)
4. **Speed Mode** (existing)
5. **Roughing Pass** (existing)
6. **Finishing Pass** (existing — add strategy toggle: Raster / Spiral)
7. **Toolpath Quality** (existing — add spiral stepover info)
8. **Output** (existing — updated for separate files)

---

## Parameters with suggested defaults (all editable)

### Square cut defaults:
| Parameter | Default | Notes |
|-----------|---------|-------|
| Width/Height | 8.0" | Lens block size |
| Cut depth | 1.01" | Through 1" stock with 0.01" clearance |
| DOC | 0.15" | Conservative for acrylic |
| Bit diameter | 0.500" | O-flute end mill |
| RPM | 20000 | Standard for acrylic |
| Feed | 120 IPM | Chipload 0.006" (conservative) |
| Plunge | 36 IPM | 30% of feed |
| Rapid | 220 IPM | Matches sample G-code |
| Safe height | 0.500" | Higher than caustic safe height — fresh stock may have warpage |
| Tabs | 4 × 0.5" wide × 0.1" tall | One per side, midpoint |

### Spiral finishing defaults:
| Parameter | Default | Notes |
|-----------|---------|-------|
| Stepover | 20% of bit dia (0.050" for 1/4") | Optical quality, ~2.6 mil scallop |
| Angular step | Adaptive, ~0.020" linear | Smooth enough for G1 interpolation |
| Direction | CW outside-in | Climb milling on acrylic |

---

## Hard constraints

- **F code on every G01 line.** NK105 requirement. Already enforced by `g1()` helper.
- **No G00.** Use G01 at rapid feed instead. Existing convention.
- **Separate .nc files.** Each pass is a standalone program.
- **Do not modify `gen_roughing()`.** It works. Leave it.
- **Do not modify `build_heightfield()`, `interp_z_in()`, or `analyse_surface()`.** They work.
- **Keep X-raster finishing as an option.** Don't remove `gen_finishing()`.
- **All fields editable in the UI.** Default values are suggestions, not locks.
- **Version the file as `causticforge_v2.py`.** Do not overwrite v1.
- **bl_info version bumps to (2, 0, 0).**

---

## Testing plan

After implementation, test with:

1. **Square cut:** Generate a square cut .nc file. Open in a G-code viewer (or just inspect the text) — verify CW perimeter, correct Z levels, tabs present at 4 midpoints.
2. **Spiral finishing:** Load the `causticforge_v1.blend` file (or any caustic OBJ), run analysis, export spiral finishing. Verify the spiral starts outside the stock, winds inward, Z follows the heightfield.
3. **Separate files:** Export all three passes. Verify each has its own header and M30.
4. **Raster fallback:** Confirm X-raster finishing still works when selected.

---

## Git

```bash
cd /Users/admin/Documents/Claude/BlenderAddons/causticforge
git add -A && git commit -m "session start: CausticForge v2.0 — spiral finishing + square cut + separate exports"
# ... implement ...
git add -A && git commit -m "CausticForge v2.0 complete: spiral finishing, square cut, separate file exports"
```
