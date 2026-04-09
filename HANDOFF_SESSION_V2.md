# HANDOFF SESSION — CausticForge v2.0
**Date:** 2026-04-08
**Agent:** Claude Code (Opus 4.6)
**File:** `causticforge_v2.py`

---

## What was built

### 1. Spiral Finishing Pass (`gen_spiral_finishing()`)
- Archimedean spiral: `r(theta) = R_start - (stepover / 2pi) * |theta|`
- Outside-in, CW (climb milling) — best surface finish on acrylic
- `R_start` = half stock diagonal + stepover (starts just outside corners)
- Adaptive angular step: targets ~0.020" linear step between G1 points (tighter near center)
- Z sampled from heightfield via existing `interp_z_in()` — no new heightfield code
- Points outside stock rectangle are skipped (spiral inscribes circle, stock is rectangular)
- Plunge feed triggered when Z drops > 0.005" between consecutive points
- Default stepover: 20% of bit diameter (0.050" for 1/4" ball nose) — optical quality ~2.6 mil scallop
- Override via `spiral_stepover_override` property (0 = use bit default)
- Added `stepover_spiral_pct` to BIT_PRESETS (all bits get 20% spiral default)
- Scallop guide shown in panel: 40%/25%/20%/15% with scallop heights

### 2. Square Cut Module (`gen_squarecut()`)
- Profile perimeter cut for sizing acrylic blocks before caustic milling
- Tool center offset outward by half bit diameter (cuts exact square)
- CW rectangle path (climb milling): bottom(+X) -> right(+Y) -> top(-X) -> left(-Y)
- Multi-pass Z levels from 0 to -sq_depth in steps of sq_doc
- Tab support: 4 tabs at side midpoints, lifts to tab_z on deepest passes
- Tab activation: only when within tab_height of final depth
- 20+ sample points per side for smooth tab transitions
- Defaults: 8x8" block, 1.01" depth, 0.15" DOC, 0.5" O-flute, 20k RPM, 120 IPM

### 3. Separate File Exports
- Three independent export operators: `CF_OT_ExportSquareCut`, `CF_OT_ExportRoughing`, `CF_OT_ExportFinishing`
- Each produces a standalone .nc file with full header (G54, G20, G17 G90, M03, ... M05, M30)
- `gcode_header()` for roughing/finishing (includes caustic metadata)
- `gcode_header_squarecut()` for square cut (no mesh needed)
- Three file path properties with auto-naming from object: `{obj}_roughing.nc`, `{obj}_finishing.nc`, `{obj}_squarecut.nc`
- "Export All Enabled" button calls all active operators
- Output section only shows file paths for enabled passes

### 4. UI Panel Updates
- Panel sections reordered: Stock > Square Cut > Analysis > Speed > Roughing > Finishing > Quality > Output
- Finishing section: strategy toggle (X-Raster / Spiral) with `expand=True` radio buttons
- Spiral mode shows: stepover override field + scallop guide reference
- Raster mode shows: locked stepover + time estimate (as before)
- Square Cut section: collapsible, all fields exposed, tab sub-section, chipload + pass count info
- Output section: separate file paths + buttons per pass, pass toggles, Export All button
- `bl_info` version bumped to (2, 0, 0)

## Geometry approach

**Spiral:** Parametric theta loop, decrementing theta (CW). Radius decreases linearly with |theta|. Each point is checked against stock bounds before emitting G1. The spiral naturally covers the rectangular stock area minus the corners that fall outside the inscribed circle — roughing handles those corners.

**Square cut:** Simple offset rectangle, no arc corners. Tool center walks the perimeter at each Z level. Tabs implemented as Z-lift zones checked per point along each side. Tab detection uses position tolerance of tab_width/2 around side midpoints.

## What was NOT changed
- `build_heightfield()` — untouched
- `interp_z_in()` — untouched
- `analyse_surface()` — untouched
- `gen_roughing()` — untouched
- `gen_finishing()` (X-raster) — untouched
- All scale guard logic — untouched

## Known issues / limitations
1. **Square cut tabs are 4 fixed (one per side midpoint)** — the `sq_tab_count` property exists but the generator always places exactly 4 at midpoints. Supporting 2-8 arbitrary tabs would need a distribution algorithm.
2. **Spiral time estimate not shown** — the existing `estimate_time_min()` calculates raster time only. Spiral time would need path length integration. Not critical since the point count is reported in the G-code comments.
3. **Square cut corner radius** — the perimeter path has sharp 90-degree corners (no arc interpolation). The CNC will decelerate at corners naturally. Could add G2/G3 corner arcs later if needed.
4. **Export All uses bpy.ops calls** — calls operators sequentially. If one fails, the rest still run. Error reporting may be lost for earlier failures.

## Next steps
- Test with actual caustic OBJ in Blender
- Verify spiral Z-following against known heightfield
- Verify square cut tab placement in G-code viewer
- Consider adding spiral time estimate to panel
- Consider G2/G3 corner arcs for square cut

## Test checklist
- [ ] Addon registers in Blender without errors
- [ ] Square Cut: generates .nc with CW perimeter, correct Z levels, 4 tabs at midpoints
- [ ] Spiral Finishing: loads caustic OBJ, runs analysis, exports spiral — verify outside-in CW direction
- [ ] Spiral: Z values follow heightfield (not flat), all points within stock bounds
- [ ] Raster Finishing: still works when strategy set to X-Raster
- [ ] Separate Files: each .nc has its own header (G54/G20/G17 G90) and M05/M30 footer
- [ ] Export All: writes all enabled passes to separate files
- [ ] Panel: strategy toggle switches between spiral and raster UI
- [ ] Panel: square cut section shows/hides with toggle
- [ ] Auto-naming: output paths update after analysis with object name
