# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running

Requires a local HTTP server (ES module `importmap` won't work over `file://`):

```bash
python3 -m http.server 8080
# then open http://localhost:8080
```

If port 8080 is taken: `lsof -ti :8080 | xargs kill -9`

## Architecture

Two files only — no build step, no npm, CDN-loaded Three.js r0.158.0.

| File | Role |
|---|---|
| `index.html` | HUD markup + CSS (including mobile styles), CDN importmap |
| `simulation.js` | Everything else: physics, Three.js scene, animation loop, paint system, UI binding |

### Physics (`simulation.js` top section)

- **Integrator**: Runge-Kutta 4 with `SUBSTEPS=20` sub-steps per frame
- **Key constants**: `DT_BASE=0.005`, `SOFTENING=0.08` (small value is critical for choreographies — do not increase), `G_BASE=1.0`
- State packed into `Float64Array [x,y,z,vx,vy,vz]` per body; `computeDerivatives()` + `rk4Step()` operate on this flat array

### Choreographies

`CHOREOGRAPHIES` map of named functions → initial conditions (`{ masses, positions, velocities }`). Two IC generators:
- `suvakov(vx, vy)` — Šuvakov & Dmitrašinović 2013 initial conditions, scaled by `choreoScale`
- `equilateralIC(vt, vr)` — equilateral triangle configs for 3-fold symmetric mandalas

Kepler scaling: positions × `s`, velocities / `√s`, period ∝ `s^1.5`. `choreoScale` (default 3) is the scale factor.

### Trail System

`Trail` class uses a `Float32Array` ring buffer (`MAX_HISTORY=20000` pts × 3 floats). `addPoint()` is O(1). `updateVisible(n)` copies last `n` points contiguously (handling wrap-around) into a GPU buffer every frame.

### Canvas / Paint Mode

Two sub-modes controlled by `paintMode` state variable:

- **`'world'`**: 3D trails remain visible as Three.js `Line` geometry. Nothing is drawn on the 2D overlay. Looks like space mode but with a white background. Save PNG captures the 3D renderer.
- **`'screen'`**: 3D trails are hidden (`updateVisible(0)`). Brush strokes are painted each frame onto a 2D `<canvas id="paintCanvas">` overlay using `drawBrushStroke()`. Save PNG captures the 2D overlay with a white background composite.

`toggleCanvasMode()` switches backgrounds (dark space ↔ white canvas), toggles starfield/grid style, and dims/restores body point lights. On entry in screen mode, `replayHistoryToCanvas()` replays the full trail ring buffer as 2D strokes.

`worldToScreen(pos)` projects a `THREE.Vector3` through the live camera to canvas pixel coordinates — used in screen mode and during history replay.

### Saving

`saveImage()` branches on `canvasMode && paintMode === 'screen'`:
- Screen mode → composite white bg + paintCanvas → PNG
- Everything else → `renderer.domElement.toDataURL()` (renderer has `preserveDrawingBuffer: true`)

### UI

- All sliders/buttons bound in `bindUI()`
- User presets stored in `localStorage` under key `'3bp-presets'`; exposed as `window._3bp_load` / `window._3bp_del` for inline `onclick` handlers
- Mobile: HUD slides up from bottom via CSS `transform: translateY(calc(100% - 44px))`; tap header to expand/collapse (`@media (max-width: 700px)`)
- Body dragging: Three.js `Raycaster` intersects body spheres, projects onto a horizontal `THREE.Plane` at the body's y-height
