# Plan — Web map-click sock-fetch mission

**Status:** M1–M6 IMPLEMENTED (component-validated); full-mission e2e = interactive · **Created:** 2026-06-08 · **Mode:** sim-first, sim + real
**Goal:** From the web control page, click a point on the map → the robot drives there, searches for a
sock, picks it up, and deposits it in a predefined area. Then reports back to the UI and is ready for
the next click.

**Design principle (standing): keep it modular.** A thin `mission_coordinator` orchestrates *existing*
capabilities through their action interfaces; it owns no low-level logic. The web layer is a thin
addition to the existing `web_control_node`.

---

## Mission flow

```
[web map click] --pixel→world--> /mission/goal (PoseStamped, map frame)
        │
        ▼
  mission_coordinator  (NEW, thin state machine; action ~/run_mission, cancellable)
   NAVIGATE_TO_SITE → SEARCH → PICK → NAVIGATE_TO_DEPOSIT → DEPOSIT → REPORT
        │                │        │             │              │
   nav2 NavigateToPose   rotate-  mobile_grasp  nav2 Navigate  open gripper
   (to clicked point)    scan +   _coordinator  ToPose (to     (GripperCommand)
                         /detect  /execute_     deposit pose)  + small place
                         ions     sock_grasp
        └──── status/feedback ────────────────────────────────► web UI (SSE/WS)
```

## Reuse (do NOT rebuild) — verified present
- **nav2** (`jetank_navigation/navigation_full.launch.py mode:=nav2 map:=…`): provides `NavigateToPose`
  action + AMCL localization + costmaps. The mission just sends `NavigateToPose` goals.
- **`web_control_node`** (`jetank_web_control`): already serves the UI, `/map.png` (Nav2 occupancy grid
  as PNG), `/map_meta` (width/height/resolution), `/ws` teleop; already imports `PoseStamped`/
  `PoseWithCovarianceStamped`. The map panel already renders the map → add a click handler + a goal route.
- **`mobile_grasp_coordinator`** (`jetank_manipulation`, Trigger `~/execute_sock_grasp`): the whole
  detect→approach→grasp pick (built in Phase 7). The mission calls it for PICK.
- **`grasp_server`** gripper control (`GripperCommand` on `/gripper_controller/gripper_cmd`) → reused for
  DEPOSIT (open to release).
- **detection** (`/detections/socks`) → used by SEARCH to know a sock is in view before PICK.

## New components (modular)
1. **Web map-click → goal** (UI + `web_control_node`):
   - UI: on map-canvas click, convert pixel→world using `/map_meta` (`resolution`, map `origin`); draw a
     marker; POST `/mission/goal {x, y, [theta]}` (world, map frame). Add a "Fetch sock here" button +
     a mission-status line. (The map panel + meta already exist — this is an overlay + one fetch call.)
   - `web_control_node`: add `POST /mission/goal` → send a goal to `mission_coordinator` (action client
     or republish `PoseStamped` on `/mission/goal`); add `GET /mission/status` (SSE or poll) fed by a
     mission-status subscription. Keep all new code behind the same aiohttp app; pure pixel↔world helper
     is unit-testable (mirror the existing module-level-helpers pattern).
2. **`mission_coordinator`** (NEW node, `jetank_manipulation` or a new `jetank_mission` pkg):
   - Action `RunMission` (goal: `PoseStamped site`, optional `search_timeout`; result: success + outcome
     string + final state; feedback: current state) — OR a simpler `PoseStamped` topic trigger if an
     action is overkill. **Recommend an action** (cancellable, feedback to UI).
   - State machine: `NAVIGATE_TO_SITE` (`NavigateToPose` → clicked point) → `SEARCH` (rotate in place via
     `/cmd_vel`, watch `/detections/socks`; found → stop; timeout → expanding pattern or fail) →
     `PICK` (call `~/execute_sock_grasp`) → `NAVIGATE_TO_DEPOSIT` (`NavigateToPose` → predefined deposit
     pose param) → `DEPOSIT` (open gripper; optional lower/place via grasp_server) → `REPORT`.
     Any step fails → graceful abort + status. Pure transition logic factored for unit tests.
   - Publishes `/mission/status` (state + outcome) for the UI.
3. **Deposit area**: a param/config `deposit_pose` (x,y,theta in map frame), default set for `sock_arena`.

---

## Phased implementation (sim-first, each phase independently testable)

**M1 — Map-click → world goal (web), two modes.** UI click→world (pixel·resolution + map origin), marker
overlay. **Pick-site** click → `POST /mission/goal {x,y}`. A **"Set deposit area"** toggle → click →
`POST /mission/deposit {x,y}` (distinct marker; `web_control_node` persists it to a param/file so it
survives restarts). *Accept:* both clicks yield correct world coords (verify against a known map feature);
pick-site publishes the mission goal; deposit click stores + redraws the deposit marker.

**M2 — `mission_coordinator` + NAVIGATE.** Skeleton action `~/run_mission`; implement
`NAVIGATE_TO_SITE` via `NavigateToPose`. Run with nav2 (mode:=nav2 + a saved `sock_arena` map). *Accept:*
clicking the map drives the base to that point in sim (within nav2 tolerance).

**M3 — SEARCH.** At the site, rotate-scan (`/cmd_vel`) while monitoring `/detections/socks`; stop facing
the sock when found; bounded timeout → report "no sock". *Accept:* base rotates, stops when a sock is
detected; clean "not found" when none.

**M4 — PICK.** Call `~/execute_sock_grasp` (the Phase-7 pipeline). *Accept:* on a found sock, the pick
sequence runs and reports success/fail (subject to the Phase-7 grasp validation still in progress).

**M5 — DEPOSIT.** `NavigateToPose` → the **stored deposit pose** (set via the M1 deposit click; fail
clearly if none set yet); open gripper to release (+ optional place motion). *Accept:* base drives to the
deposit area and the gripper opens.

**M6 — End-to-end + UI feedback.** Wire `/mission/status` → UI status line; full click→deposit run;
`mission_coordinator` returns to idle for the next click. Add a `web_mission.launch.py` (nav2 + the
mobile-grasp stack + mission_coordinator + web_control) — one command. *Accept:* one map click yields a
full fetch-and-deposit cycle in sim, status shown in the UI.

---

## Decisions (locked 2026-06-08)
1. **Localization:** **AMCL + a saved `sock_arena` map** (`navigation_full mode:=nav2 map:=…`). Build the
   map once via SLAM, save it; the web map-click + deposit are in that stable `map` frame.
2. **Mission home:** a **new `jetank_mission` package** (ament_cmake+python or ament_python) — top-level
   orchestration separated from manipulation. Depends on the nav2 / `jetank_manipulation` / detection
   interfaces only.
3. **Mission interface:** **`RunMission` action** (`jetank_mission/action/RunMission`) —
   goal `{PoseStamped site, float32 search_timeout}`, result `{bool success, string outcome}`,
   feedback `{string state}`. `web_control_node` is the action client (cancellable; streams state to UI).
4. **Deposit area = a SECOND clicked point (two-click UI).** The web map supports two modes: a normal
   click = **pick site** (runs the mission), and a **"Set deposit area"** toggle whose click stores the
   **deposit pose** (persisted to a param/file, drawn as a distinct marker on the map). The mission's
   DEPOSIT step navigates to the stored deposit pose. (Adds a UI mode + a `POST /mission/deposit` route +
   persistence; folded into M1/M5.)
5. **Search pattern:** rotate-in-place at the site + timeout (v1); expanding sweep deferred.
6. **Pick coupling:** reuse `mobile_grasp_coordinator` (`~/execute_sock_grasp`) as-is; SEARCH only ensures
   a sock is in view before delegating PICK.

## Risks
- **Localization accuracy** drives both navigation-to-site and the grasp's open-loop pose (odom drift over
  the approach). Good AMCL + a decent map are prerequisites.
- **The Phase-7 grasp** is built but its final move_group pose-execution is still being validated
  (`sock-pointcloud-plan.md` §8); a floor pick may need the interactive tuning noted there. The mission
  inherits that status — M4 is gated on it.
- **Search coverage:** rotate-in-place only finds socks visible from the clicked point; off-view socks
  need a sweep/patrol (out of v1 scope).
- **nav2 footprint/recovery** on a small tracked base in a cluttered arena → tune costmap inflation.
- **Deposit "place"** is open-gripper-release v1 (drop), not a gentle placement.

## Out of scope (v1)
Multi-sock collection, dynamic obstacle replanning beyond nav2 defaults, gentle placement, search patrol
across the whole map, real-hardware bring-up (mirrors `sock-pointcloud-plan.md` Phase 5).

---

## Implementation status (2026-06-08)
- **M1** ✅ web map-click → `/mission/goal` + two-click deposit (persisted), live-verified.
- **M2–M5** ✅ new **`jetank_mission`** pkg + `RunMission` action + `mission_coordinator` FSM (NAVIGATE→SEARCH→PICK→DEPOSIT→DONE), cancellable, latched `/mission/status`; 31 FSM pytests; graceful-fail verified.
- **M6** ✅ `web_control_node` RunMission client + `/mission/status` poll + `/mission/cancel` + UI status/cancel; `web_mission.launch.py` (one-command full stack; nav2 `use_sim_time` gates hardware off to avoid double-start). 91 web pytests.
- **PICK** uses `grasp_mode:=preset` (Cartesian floor grasp infeasible on the 4-DOF arm — see sock-pointcloud-plan §8).
- **Remaining = interactive full-mission validation:** needs a **saved `sock_arena` map** (build via SLAM + save) for nav2 mode, then `ros2 launch jetank_mission web_mission.launch.py map:=<map.yaml>`, set a deposit point, click a pick site, watch the FSM. The headless 10+-node stack is flaky here; every component is individually validated.
- **`jetank_mission` is a new LOCAL git repo** (branch feature/sim-disparity-source) — set an `origin` (kvgork/jetank_mission) + add to `jetank.repos` to spin it out like the other siblings.
