---
addon: CausticForge
version: 2.0.0
prefix: cf_
type: single-file
filename: causticforge_v2.py
category: Object
status: active
---

# CausticForge

G-code exporter for caustic surface OBJ — analysis + roughing + finishing + square cut.

- **Location:** View3D > N-Panel > CAUSTICFORGE
- **Blender:** 4.0+
- **Author:** Bland Design
- **Props:** `scene.cf_props`

## v2.0 features (over v1.3)
- Spiral finishing pass (Archimedean, outside-in CW climb)
- Square cut module (perimeter profile cut with tabs)
- Separate .nc file exports (roughing, finishing, square cut)
- No-lift roughing raster (edge-entry overshoot, no retract between rows)
