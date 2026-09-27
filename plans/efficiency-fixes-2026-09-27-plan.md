# Efficiency Fixes — Apply Plan (2026-09-27)

Session `20260927-093513-apply-audit-high-medium` · rigor agentic · grill mode on · Workflow shape (user-confirmed)

## Outcome
The 7 high and 46 medium findings of `plans/efficiency-audit-2026-09-26.md` applied as one conventional commit per finding on branch `chore/efficiency-fixes` in each touched repo (branched from each repo's current HEAD; nothing pushed).

## Why
Cut idle CPU/bandwidth on the Jetson (perception ~190 % CPU, disparity at 0.6 Hz, web control decoding for nobody, Nav2 voxel grid to nobody) and remove silent-config bugs (motor_params keys, DWB `vth_samples`, gripper action name) before further hardware work.

## Constraints
- Preserve modular architecture; no package merges; keep perception strategy/factory abstractions.
- Each fix follows its verified fix_sketch and verifier caveats; no behaviour change beyond the finding.
- Deferred: motor_control-16/17/18 (serial bus timing) until the servo bus answers.
- Tagged `needs runtime validation`: detection-01/02/04/05 (no ultralytics/model), simulation-09/14/19 (no Gazebo run).
- pixi.toml findings (footprint-01/02/09/10, ros_main-31) are Phase C, applied only after a second user confirmation of the re-solve.
- Executors edit only their own repo; cross-* findings run in Phase B2 by a single executor after B1.
- Executors validate per package with `cmake --build build/<pkg>` (C++) or py_compile + flake8 (Python); the orchestrator runs the workspace `pixi run build` + `colcon test` between phases.

## Execution Shape
| phase | content | agents | gate |
|---|---|---|---|
| A | 6 high code findings: perception-01/02/05/22, ros_main-01, web_control-01 | 3 executors (Sonnet, parallel by repo) → 3 adversarial reviewers | build+test, 0 blockers |
| B1 | 34 medium findings by repo (perception 12, navigation 4, manipulation 3, moveit_config 1, detection 4, ros_main 4, web_control 3, simulation 3) | 8 executors → 8 reviewers | build+test, 0 blockers |
| B2 | cross-01/02/06/07/09 (multi-repo config/launch consolidation) | 1 executor → 1 reviewer | build+test, 0 blockers |
| C | footprint-01/02/09/10, ros_main-31 (pixi.toml + template) | 1 executor, prepared then user-confirmed re-solve | pixi install + build |
| final | rebuild install_sys (system-ROS camera), live re-measure camera + web control + mock arm goal | 1 measurement agent | high symptoms gone, numbers in report |

## Done
- `pixi run build` exit 0 after each phase; `colcon test` on touched packages: no new failures vs baseline.
- Grill review: zero blockers per phase (blockers fixed in-loop, max 2 rounds).
- Final measurements appended to `plans/efficiency-audit-2026-09-26.md` as "Post-fix addendum".
- Per-repo `git log chore/efficiency-fixes` lists one commit per applied finding id.

## Plan-critique / panel
Skipped: every finding was already adversarially verified (85 skeptics, 3 refuted) in the audit run; the plan is that verified list plus phase ordering. Adversarial review runs on the *diffs* in 7.5a instead.

## Run log
- 2026-09-27 10:00 branches chore/efficiency-fixes created in all 10 repos; baseline build exit 0.
- 2026-09-27 10:21 Phase A: 6/6 applied (perception c231b3e cd397e1 f699e86 6341282; ros_main 9f4361d; web_control 7d62dfb). Grill: 2 blockers (Q(3,2) sign; world->odom dual parent), 4 majors. Fix loop round 1 started.
- 2026-09-27 10:31 Phase A fix loop: 45f7a28 79cc5f3 (perception), bde572d (ros_main), 23fcfa4 (moveit_config), 6e98e5d (web_control). Gate: build exit 0; tests: only pre-existing offline xmllint failure; web_control pytest 108 passed. Round-2 review running.
- 2026-09-27 10:39 Phase A round-2: blockers resolved; majors fixed in af55986 (perception), a626ef2 (moveit_config), 670f273 (ros_main). Open minors documented (rect-consumer race on unsubscribe, capture timeout not tunable, SRDF virtual_joint fixed-type semantics, no Q regression test, sim camera_info P[3] may be 0). Phase B1 launched (8 repos).
- 2026-09-27 11:35 Phase B1: 35/35 applied across 8 repos (1.79M tokens). Grill: 4 blockers (perception-04 default still BGR8; manipulation-33 cold TF buffer; simulation-09 test count; simulation-19 gripper_controllers), 4 majors. simulation fixed in 98ea843; perception + manipulation fixers running.
- 2026-09-27 11:50 B1 fix loop r1: perception 93ee94a 87e75a0 fc70548; manipulation 2ba40ab 77ea6f6; simulation 98ea843. Gate: build 0; full colcon test 382 tests, only offline xmllint + 2 new pep257 (fixed in follow-up commit). Round-2 review running.
- 2026-09-27 13:48 B1 round-2: majors fixed in 422b46b (perception helper/PNG/single_camera), 198d408 + ead0457 (gripper default per-controller, hw override in launch), 9456aaf (web_control bgra8). Open minors: bgra8 raw is 33% more bytes (trade-off, documented), redundant double can_transform wait, no tests pinning BGR8 template / cold-buffer path, jetank_simulation missing exec_depend jetank_motor_control. Phase B2 launching.
- 2026-09-27 13:51 install_sys rebuilt under /opt/ros/humble (--packages-up-to jetank_perception: jetank_detection + jetank_perception), exit 0; stereo_camera_node binary fresh (13:50). Ready for final measurement.
- 2026-09-27 14:20 Phase B2: cross-01 (ae66f27 ros_main + 49e05aa navigation), cross-02 (6fa041d), cross-06 (be925a9 ros_main + 95fb043 motor_control), cross-09 (942a43c manipulation + 53c6217 moveit_config) applied; cross-07 DEFERRED (track width 0.14 sim vs 0.11 hardware needs a physical wheel-separation measurement before unifying; velocity caps already consistent). Grill: 3 blockers + 3 majors -> fixed in 1fd11be 35449c7 (ros_main), b21c287 (navigation), 4c23a32 (manipulation). Round-2 review + gate running.
- 2026-09-27 14:59 B2 round-2: pass. Minors folded in: e4830e1 (all enable_* gates .lower()), fa7c9c1 (SRDF guard TypeError + empty set). Gate build failed once on a dangling symlink-install link build/jetank_ros_main/config/motor_params.yaml (file deleted by cross-06) — removed, gate re-running. NOTE for anyone pulling the branch with an old build/: remove that link or clean build/jetank_ros_main.
- 2026-09-27 15:01 B2 gate re-run: build 0, tests only offline xmllint. Phase C started: pixi.toml replaced (backup pixi.toml.bak-2026-09-27, pixi.lock.bak-2026-09-27); pixi install running.
- 2026-09-27 15:03 Phase C: first solve failed (python=3.10.* pin vs rplidar-ros needing 3.11/3.12; env is 3.12) — pin removed; pixi install exit 0. Clean rebuild started.
- 2026-09-27 15:03 pixi env after Phase C: 5.2G (from 5.9G), 792 packages; removed desktop/gazebo/ign/servo/turtlesim/rqt; rviz2 remains via moveit metapackage; environments default/dev/detect.
- 2026-09-27 15:09 Phase C done: clean rebuild exit 0 (2m15s, BUILD_TESTING off); smoke test 52 checks 0 failed; ldd clean on all C++ binaries. Final stack live (camera BGRx 4-channel confirmed); measurement agent running. Lessons routed: system-improvements #28-32, memory project_efficiency_fixes_2026_09.
- 2026-09-27 15:17 Final measurement done; stack down; post-fix addendum + digest + memory + vault capture written; wiki ingest next.
- 2026-09-27 15:27 COMPLETE. Wiki ingested (sources/2026-09-27-jetank-efficiency-fixes.md); finalize emitted.
- 2026-09-27 15:52 MERGED: chore/efficiency-fixes -> base branch in 9 repos (merge commits: detection a706cec, manipulation 820943a, motor_control 3caa6d2, moveit_config e4d80f6, navigation 8d7f231, perception f36d9a1, ros_main 50da31b, simulation 0e002ff, web_control 22ff265); merged build exit 0; python tests 111/0 failures; full 'pixi run test' (BUILD_TESTING on) running. Not pushed.
- 2026-09-27 15:54 Merged-state full gate: pixi run test (BUILD_TESTING on) -> 382 tests, 0 errors, 10 failures = the pre-existing offline xmllint checks only. Merge complete; branches chore/efficiency-fixes kept for reference; nothing pushed.
- 2026-09-27 16:37 PUSHED to origin: merged base branches (6x main, 3x feature/sim2real-hardware-bringup) and chore/efficiency-fixes in all 10 repos.
- 2026-09-27 16:52 Disparity experiment: hypothesis (UDP fragment loss) FALSIFIED — real delivery 30 Hz disparity / 30 Hz points; 0.6 Hz was an ros2-mcp tool artifact. Camera CPU 40% idle, 120% serving. Correction appended to report + digest; probes saved as plans/disp_probe.py, plans/points_probe.py.
