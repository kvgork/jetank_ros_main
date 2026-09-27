"""
Shared /sock_detector lifecycle auto-activation.

The configure -> activate sequence for the jetank_detection sock_detector
lifecycle node used to be hand-rolled three different ways across this
package's integration launch files:

  - sim_demo.launch.py:       a bash poll loop (``ros2 node list | grep``,
                              up to 180 s) then configure + activate.
  - mobile_grasp.launch.py:   two fixed TimerActions at 40 s / 46 s.
  - mobile_grasp_hw.launch.py: two fixed TimerActions at 22 s / 28 s.

Fixed delays either fire before the node exists (the lifecycle transition
is rejected) or waste tens of seconds waiting past when the node was
actually ready. All three call sites now share this one poll-based
implementation instead.

Non-fatal by design: if ``on_configure`` fails (e.g. a missing model file),
the node just stays unconfigured and logs a warning -- this matches the
pre-existing sim_demo.launch.py convention, which every caller here
preserves.
"""

from launch.actions import ExecuteProcess, TimerAction


def sock_detector_autostart(
    condition=None, poll_timeout_s=180, node_name='/sock_detector',
    start_after_s=0.0,
):
    """
    Return an action that configures then activates node_name.

    Polls ``ros2 node list`` every 2 s, up to ``poll_timeout_s``, until
    ``node_name`` appears (it may not exist yet if e.g. Gazebo or the camera
    driver is still starting up), then runs the configure -> activate
    lifecycle transitions with a 3 s settle delay between them.

    :param condition: optional launch Condition gating the whole action
        (e.g. only auto-start when a `detect` launch argument is true).
    :param poll_timeout_s: maximum time to wait for the node to appear.
    :param node_name: fully-qualified lifecycle node name to transition.
    :param start_after_s: delay before polling starts. Callers that start the
        detector on a TimerAction should pass that same period so the poll
        does not spawn ``ros2 node list`` every 2 s while the detector is
        deliberately not running yet.
    """
    attempts = max(1, poll_timeout_s // 2)
    process = ExecuteProcess(
        condition=condition,
        cmd=['bash', '-c',
             f'for i in $(seq 1 {attempts}); do '
             f'ros2 node list 2>/dev/null | grep -q {node_name} && break; sleep 2; done; '
             f'ros2 lifecycle set {node_name} configure && sleep 3 && '
             f'ros2 lifecycle set {node_name} activate'],
        output='screen',
    )
    if start_after_s > 0:
        return TimerAction(period=float(start_after_s), actions=[process])
    return process
