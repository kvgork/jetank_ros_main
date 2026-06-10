# Plan — Separate Mapping from Navigation (saved-map-only nav)

**Date:** 2026-06-08
**Rigor:** agentic · **Mode:** implementation
**Request:** "Mapping UX doesn't work; localization on saved maps is bad. Make mapping mode completely separate from navigating (it sometimes shifts the map). When navigating, only use a saved map."

## Root cause

The web control's **Mapping Mode** (`web_control_node.start_mapping`) launches **`slam_nav2.launch.py`** = online `slam_toolbox` (mapping) **+ nav2 navigation** running concurrently. Online SLAM continuously runs scan-matching, loop-closure and pose-graph optimization, so `/map` and the `map→odom` transform **shift while you drive/navigate**. Sending NavigateToPose goals against a live-shifting map is the "mapping doesn't work / shifts the map" complaint.

Navigation Mode (`start_navigation` → `nav2_bringup.launch.py`, `map_server` + AMCL) is **already saved-map-only**. "Localization is bad" stems from (a) the saved map being poisoned by the shifting online SLAM, and (b) loose AMCL params. (a) is fixed by separation; (b) is a tuning pass — staged below.

Process teardown between modes already exists: `_launch_nav` calls `stop_nav` + pkills `async_slam_toolbox_node`/`amcl`/`map_server`/all nav2 servers, so the two modes never co-run at the process level. The bug is purely *which launch* mapping starts.

## Decisions (user-confirmed)

- **Mapping = teleop-only.** SLAM-only; drive with the web joystick to build the map, then Save Map. Click-to-navigate disabled while mapping.
- **AMCL tuning = staged, NOT applied now.** Implement separation first; keep the tuning as a ready-to-apply Phase 2 so we can see whether a clean saved map alone fixes localization.
- **Scope guard (pre-flight WARN):** touch ONLY the mapping + navigation paths. Do not rename/remove unrelated launch files, worlds, RViz configs, or web routes. `slam_nav2.launch.py` stays on disk (legacy / advanced manual use) but is no longer used by the web UI.

---

## Phase 1 — Mode separation (APPLY NOW)

All in `src/jetank_web_control/jetank_web_control/web_control_node.py` (+ a docstring note in `jetank_navigation`).

1. **`start_mapping()`** (~L2623): launch **`slam.launch.py`** instead of `slam_nav2.launch.py`. Return message → `"mapping started (slam_toolbox only — drive with the joystick, then Save Map)"`.
2. **Expose running mode to the frontend:** in `refreshNavStatus()` JS, store `navRunning = s.running` (already returned by `nav_status()` as `'mapping' | 'navigation' | null`) in a module-level var.
3. **`mapClick(ev)`** (~L1290): when `navRunning === 'mapping'`, do NOT send navigate/fetch/deposit; show `navMsg('Driving mode — use the joystick to build the map, then Save Map.', true)` and return. Click modes work only when navigating.
4. **`toggleMappingMode()`** (~L998): button label while active → `"⏹ Stop Mapping"` (currently "Stop Nav"); idle → "Start Mapping".
5. **UI copy:** mapping panel hint (~L555, L1214-1217) → `"Mapping active · drive with the joystick to build the map, then Save Map."`; idle hint unchanged ("Start Mapping, drive around, then Save Map.").
6. **Top docstring** (~L15) `/start_mapping` line → "start slam_toolbox (mapping only)".
7. **`slam_nav2.launch.py` docstring:** add one line — "LEGACY: combined SLAM+nav2 shifts the live map under loop-closure; the web UI now uses slam.launch.py (mapping) and nav2_bringup.launch.py (saved-map nav) separately."

**No functional change to `start_navigation`** — it is already `nav2_bringup` + saved map + AMCL + `_seed_initial_pose`. Confirm only.

### Phase 1 acceptance criteria (sim)
- Mapping mode: `async_slam_toolbox_node` running; **zero** `amcl` / `map_server` / nav2-server processes. `/map` + `map→odom` published. Teleop drives the robot. Save Map writes `<map_dir>/<name>.yaml` + `.pgm`.
- Map click while mapping → "Driving mode…" message, NO NavigateToPose goal sent.
- Navigation mode (after Save Map): `amcl` + `map_server` running; **zero** `slam_toolbox` processes. AMCL publishes `map→odom`; map click sends a goal that is accepted.
- Switching mapping↔navigation leaves no lingering nodes from the other mode.

---

## Phase 2 — AMCL localization tuning (STAGED — apply only if localization still bad after Phase 1)

Per localization-specialist. Apply as a **separate commit** so it can be A/B'd against Phase 1's clean-map result and reverted independently.

`src/jetank_navigation/config/nav2/nav2_params.yaml` (amcl block):
- `laser_likelihood_max_dist: 2.0 → 0.4`  (biggest win — 2 m is ~half a 5 m room, blurs the likelihood field)
- `z_hit: 0.5 → 0.85`, `z_rand: 0.5 → 0.05`  (let map-matching dominate the uniform model)
- `sigma_hit: 0.2 → 0.08`  (C1M1 is low-noise indoors; sharper hit scoring)
- `max_beams: 60 → 120`  (richer wall/corner constraints; cheap on Orin)
- `update_min_d: 0.1 → 0.05`, `update_min_a: 0.2 → 0.1`  (more frequent updates in a small room)
- `laser_max_range: 12.0 → 9.0`  (nothing real beyond ~7 m diagonal; avoid cross-wall artifacts)
- `recovery_alpha_slow/fast`: **leave 0.0** (kidnap recovery adds jitter risk; start pose is seeded)

`web_control_node._seed_initial_pose` (~L2647): tighten seeded covariance `cov[0]=cov[7] 0.25 → 0.04`, `cov[35] 0.0685 → 0.02` (start pose is well-known).

### If still bad after Phase 2, check (specialist):
map↔scan overlay alignment in RViz; `odom→base_link` TF freshness (`tf2_monitor`); sim-time consistency on AMCL; `/particlecloud` collapse-then-jump = weights saturating (loosen sigma_hit or clean map).

---

## Out of scope
Frontier/autonomous exploration during mapping; mission ("Fetch sock") flow; removing `slam_nav2.launch.py`; costmap retuning.
