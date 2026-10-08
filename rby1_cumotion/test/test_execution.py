"""Retiming, target validation, driver-facing helpers, and the launch modes."""

from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Duration
import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rby1_cumotion.executor.execution import MAX_STEP, retime, seconds
from rby1_cumotion.planning import check_transform

PACKAGE = Path(__file__).resolve().parent.parent
LIMITS = {f'right_arm_{i}': {'lower': -3.0, 'upper': 3.0, 'velocity': 2.0} for i in range(2)}


def planned(times, positions):
    """A cuMotion-like trajectory: few waypoints, its own timing."""
    trajectory = JointTrajectory(joint_names=list(LIMITS))
    for t, q in zip(times, positions):
        whole = int(t)
        trajectory.points.append(JointTrajectoryPoint(
            positions=list(q), time_from_start=Duration(sec=whole, nanosec=int((t - whole) * 1e9))))
    return trajectory


@pytest.fixture
def plan():
    times = np.linspace(0.0, 13.6, 32)  # 32 waypoints at MoveIt's 0.1 scaling
    s = times / times[-1]
    return planned(times, np.stack([0.3 * s, -0.2 * s], axis=1))


def test_retime_hits_the_requested_duration_in_even_short_steps(plan):
    result, _ = retime(plan, 4.0, 0.05, LIMITS)
    stamps = np.array([seconds(p.time_from_start) for p in result.points])
    assert stamps[0] == 0.0 and stamps[-1] == pytest.approx(4.0, abs=1e-9)
    assert np.diff(stamps).max() <= 0.05 + 1e-9
    assert np.all(np.diff(stamps) > 0)


def test_retime_keeps_the_path_and_starts_and_ends_at_rest(plan):
    result, _ = retime(plan, 4.0, 0.05, LIMITS)
    assert result.points[0].positions == pytest.approx(plan.points[0].positions)
    assert result.points[-1].positions == pytest.approx(plan.points[-1].positions)
    assert result.points[0].velocities == pytest.approx([0.0, 0.0])
    assert result.points[-1].velocities == pytest.approx([0.0, 0.0])
    assert result.joint_names == plan.joint_names


def test_a_too_short_duration_is_refused_with_the_shortest_safe_one(plan):
    with pytest.raises(ValueError, match=r'too fast.*at least (\d+\.\d+) s') as error:
        retime(plan, 0.05, 0.01, LIMITS)
    suggested = float(error.value.args[0].rsplit('at least ', 1)[1].split(' s')[0])
    # ...and that suggestion really is feasible.
    retime(plan, suggested * 1.001, 0.01, LIMITS)


def test_load_reports_how_close_to_the_velocity_limit(plan):
    _, slow = retime(plan, 20.0, 0.05, LIMITS)
    _, fast = retime(plan, 2.0, 0.05, LIMITS)
    assert 0 < slow < fast <= 1.0
    assert fast == pytest.approx(slow * 10, rel=1e-6)  # uniform time scaling


@pytest.mark.parametrize('step', [0.0, MAX_STEP + 0.01, 1.5])
def test_steps_must_stay_short(plan, step):
    with pytest.raises(ValueError, match='step'):
        retime(plan, 4.0, step, LIMITS)
    assert MAX_STEP < 1.0


def test_spline_overshoot_past_a_joint_limit_is_refused():
    limits = {name: dict(entry, upper=0.301) for name, entry in LIMITS.items()}
    # Every waypoint is inside the limit, but a fast rise into a plateau makes the
    # clamped spline swing to ~0.55 between them.
    times = [0.0, 0.2, 1.0, 2.0]
    positions = [[0.0, 0.0], [0.3, 0.0], [0.3, 0.0], [0.3, 0.0]]
    with pytest.raises(ValueError, match='leave its limits'):
        retime(planned(times, positions), 2.0, 0.01, limits)


@pytest.mark.parametrize('duration', [0.0, -1.0, float('nan')])
def test_duration_must_be_positive(plan, duration):
    with pytest.raises(ValueError, match='duration'):
        retime(plan, duration, 0.05, LIMITS)


def rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def pose(rotation, xyz):
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, xyz
    return matrix


def test_a_row_major_list_of_sixteen_is_a_valid_target():
    target = pose(rotation_z(0.4), [0.5, -0.2, 1.0])
    assert check_transform(target.reshape(-1).tolist()) == pytest.approx(target)


def test_a_transposed_matrix_is_caught_not_executed():
    """The likeliest mistake: sending column-major. It must not look plausible."""
    target = pose(rotation_z(0.4), [0.5, -0.2, 1.0])
    with pytest.raises(ValueError, match='transposed'):
        check_transform(target.T)


@pytest.mark.parametrize('bad,message', [
    (np.diag([2.0, 1.0, 1.0, 1.0]), 'orthonormal'),
    (np.diag([-1.0, 1.0, 1.0, 1.0]), 'reflection'),
    (np.full(16, np.nan), 'non-finite'),
    (np.eye(3), 'shape'),
])
def test_malformed_targets_are_rejected(bad, message):
    with pytest.raises(ValueError, match=message):
        check_transform(bad)


def test_executor_refuses_a_bundle_for_the_other_robot_kind():
    pytest.importorskip('rby1_msgs', reason='rby1_msgs is baked into the image; run in the container')
    from rby1_cumotion.bringup.driver import check_driver_model
    check_driver_model('m', 'm_1_2')
    check_driver_model('M', 'm_1_3')
    with pytest.raises(ValueError, match='started for a_1_2'):
        check_driver_model('m', 'a_1_2')


def test_executor_refuses_to_move_under_estop_or_major_fault():
    pytest.importorskip('rby1_msgs', reason='rby1_msgs is baked into the image; run in the container')
    from rby1_msgs.msg import RobotState
    from rby1_cumotion.bringup.driver import robot_problem
    assert robot_problem(RobotState(control_manager_state=RobotState.STATE_ENABLE)) is None
    assert robot_problem(RobotState(control_manager_state=RobotState.STATE_IDLE)) is None
    assert 'Emergency stop' in robot_problem(RobotState(emo_state=True))
    assert 'CMD_RESET' in robot_problem(RobotState(control_manager_state=RobotState.STATE_MAJOR_FAULT))


def test_launch_never_loads_the_driver_hardware_plugin():
    """rby1_hardware is not in the container, and would lock out the driver's action."""
    source = (PACKAGE / 'rby1_cumotion' / 'bringup' / 'description.py').read_text()
    code = source.split('"""', 2)[2]  # the module docstring may explain why
    assert 'rby1_hardware' not in code
    assert 'mock_components/GenericSystem' in code
    assert "choices=['driver', 'mock']" in code
    assert "DeclareLaunchArgument('hardware', default_value='driver'" in code
    assert 'joint_state_relay' in code
    # The robot's address belongs to the host driver now, not to this launch.
    assert "DeclareLaunchArgument('use_fake_hardware'" not in code
    assert "DeclareLaunchArgument('robot_ip'" not in code


def test_hold_repeats_the_final_pose_at_rest_after_the_motion(plan):
    """The driver reports done once the last waypoint is *sent*; switching the
    stream off then cut fast moves 0.03-0.055 rad short. The hold keeps the
    final target streaming while the arm settles, inside the 1 s stream timeout."""
    from rby1_cumotion.executor.execution import DEFAULT_HOLD, with_hold
    moved, _ = retime(plan, 2.0, 0.05, LIMITS)
    held = with_hold(moved, DEFAULT_HOLD, 0.05)
    tail = held.points[len(moved.points):]
    assert len(tail) == round(DEFAULT_HOLD / 0.05)
    assert seconds(held.points[-1].time_from_start) == pytest.approx(2.0 + DEFAULT_HOLD)
    for point in tail:
        assert point.positions == pytest.approx(moved.points[-1].positions)
        assert point.velocities == pytest.approx([0.0, 0.0])
    stamps = [seconds(p.time_from_start) for p in held.points]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    assert max(b - a for a, b in zip(stamps, stamps[1:])) <= 0.05 + 1e-9  # stream stays fed
    assert moved.points == held.points[:len(moved.points)]  # the motion itself is untouched


def test_zero_hold_sends_the_motion_unchanged(plan):
    from rby1_cumotion.executor.execution import with_hold
    moved, _ = retime(plan, 2.0, 0.05, LIMITS)
    assert with_hold(moved, 0.0, 0.05) is moved
    with pytest.raises(ValueError, match='hold'):
        with_hold(moved, -1.0, 0.05)


@pytest.mark.parametrize('kind,version,name', [
    ('m', 1.2, 'm_1_2'), ('M', 1.3, 'm_1_3'), ('a', 1.0, 'a_1_0'), (' a ', 1.1, 'a_1_1'),
])
def test_driver_report_maps_to_a_bundle_name(kind, version, name):
    pytest.importorskip('rby1_msgs', reason='rby1_msgs is baked into the image; run in the container')
    from rby1_cumotion.bringup.driver import bundle_name
    assert bundle_name(kind, version) == name


def test_an_unpatched_driver_version_is_explained():
    """Drivers before the fix publish 0.0: the SDK says "v1.2" and stod fails."""
    pytest.importorskip('rby1_msgs', reason='rby1_msgs is baked into the image; run in the container')
    from rby1_cumotion.bringup.driver import bundle_name
    with pytest.raises(ValueError, match='model:=m_1_2'):
        bundle_name('m', 0.0)
    with pytest.raises(ValueError, match='expected m or a'):
        bundle_name('x', 1.2)


def test_executor_is_the_only_node_that_moves_the_robot():
    """prepare_robot and move_arm were folded into target_executor and prepare;
    other posture moves are rby1_examples' job (06_zero_pose, 07_joint_command)."""
    for gone in ('prepare_robot.py', 'move_arm.py'):
        assert not (PACKAGE / 'rby1_cumotion' / gone).exists()
    movers = [path.name for path in (PACKAGE / 'rby1_cumotion').rglob('*.py')
              if 'DriverExecutor(' in path.read_text()]
    assert movers == ['target_executor.py']
    # The only other motion: prepare bends a straight arm, before the planner starts.
    posers = sorted(path.name for path in (PACKAGE / 'rby1_cumotion').rglob('*.py')
                    if 'ready_goal(' in path.read_text() and path.name != 'driver.py')
    assert posers == ['prepare.py']


def test_prepare_reads_model_and_posture_from_the_driver_before_the_stack_starts():
    """move_group and the planner load the model at start, so the bundle and the
    locked posture must be settled first -- and every arm move done before that."""
    code = (PACKAGE / 'rby1_cumotion' / 'bringup' / 'prepare.py').read_text().split('"""', 2)[2]
    assert 'read_robot(' in code and 'activate(' in code and 'straight_parts(' in code
    assert code.index('preparer.ready(') < code.index('activate(source, group, positions=positions')
    bringup = (PACKAGE / 'rby1_cumotion' / 'bringup' / 'description.py').read_text()
    assert "executable='prepare'" in bringup
    assert 'OnProcessExit(target_action=prepare, on_exit=after_prepare)' in bringup
    assert "executable='target_executor'" in bringup
    for name in ('cumotion.launch.py', 'demo.launch.py'):
        assert 'launch_description(rviz_default=' in (PACKAGE / 'launch' / name).read_text()


def test_prepare_readies_every_straight_part():
    """A straight arm is singular for hand-pose goals (arm rolls come back twisted), and
    a straight torso for a planner that moves it. Both arms and the torso are taken to
    the ready pose of the settings file, whichever group plans; a bent part stays."""
    pytest.importorskip('rby1_msgs', reason='rby1_msgs is baked into the image; run in the container')
    from rby1_cumotion import settings
    from rby1_cumotion.bringup.driver import ready_goal, straight_parts
    ready = settings.load(None, 'robot')['ready_pose']
    zero = {f'{part}_{i}': 0.0 for part, count in settings.PARTS.items() for i in range(count)}
    assert straight_parts(zero, ready, 0.3) == ['right_arm', 'left_arm', 'torso']
    assert straight_parts(dict(zero, right_arm_3=-1.57), ready, 0.3) == ['left_arm', 'torso']
    assert straight_parts(dict(zero, torso_2=-0.2), ready, 0.3) == ['right_arm', 'left_arm']
    assert straight_parts({'left_arm_3': 0.0}, ready, 0.3) == ['left_arm']  # what the driver does not report stays
    assert straight_parts(zero, dict(ready, torso=[0.0] * 6), 0.3) == ['right_arm', 'left_arm']  # a flat ready torso
    goal = ready_goal(ready, ['right_arm', 'torso'], 4.0)
    assert list(goal.right_arm.position) == ready['right_arm'] and goal.right_arm.minimum_time == 4.0
    assert list(goal.torso.position) == ready['torso'] and goal.torso.minimum_time == 4.0
    assert list(goal.left_arm.position) == []


def test_ready_pose_is_checked_when_the_settings_load(tmp_path):
    import yaml
    from rby1_cumotion import settings
    data = yaml.safe_load(settings.default_path().read_text())
    for change, message in (({'torso': [0.0] * 5}, 'ready_pose.torso needs 6'),
                            ({'head': [0.0, 0.0]}, "unknown parts \\['head'\\]"),
                            ({'left_arm': None}, 'ready_pose.left_arm needs 7')):
        broken = dict(data, robot=dict(data['robot'], ready_pose={**data['robot']['ready_pose'], **change}))
        path = tmp_path / 'cumotion.yaml'
        path.write_text(yaml.safe_dump(broken))
        with pytest.raises(ValueError, match=message):
            settings.load(path, 'robot')


@pytest.fixture
def arm():
    """The m_1_2 right arm from the shipped bundle: real kinematics and limits."""
    from rby1_cumotion.executor.execution import ToolChain
    from rby1_cumotion.model import load_model
    bundle = PACKAGE.parent / 'docker' / 'bundles' / 'm_1_2'
    if not (bundle / 'model.json').is_file():
        pytest.skip('docker/bundles is empty; run docker/make_bundles.sh')
    metadata, root = load_model(bundle, 'right_arm')
    fixed = dict(metadata['default_positions'])
    return metadata, ToolChain(root, metadata['tool_frame']), fixed


def arm_plan(metadata, start, end):
    names = metadata['active_joints']
    times = np.linspace(0.0, 10.0, 32)
    s = (1 - np.cos(np.pi * times / times[-1])) / 2
    trajectory = JointTrajectory(joint_names=list(names))
    for t, k in zip(times, s):
        q = np.asarray(start) + k * (np.asarray(end) - np.asarray(start))
        whole = int(t)
        trajectory.points.append(JointTrajectoryPoint(
            positions=q.tolist(), time_from_start=Duration(sec=whole, nanosec=int((t - whole) * 1e9))))
    return trajectory


def test_tool_chain_matches_the_full_model(arm):
    from rby1_cumotion.model import forward_kinematics, load_model
    metadata, chain, fixed = arm
    _, root = load_model(PACKAGE.parent / 'docker' / 'bundles' / 'm_1_2', 'right_arm')
    positions = dict(fixed, right_arm_1=-0.4, right_arm_3=-1.2, right_arm_5=0.3)
    assert np.allclose(chain.pose(positions), forward_kinematics(root, positions, metadata['tool_frame']),
                       atol=1e-12)


def test_short_moves_take_the_minimum_time(arm):
    from rby1_cumotion.executor.execution import motion_duration
    metadata, chain, fixed = arm
    start = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    end = [0.05, -0.5, 0.0, -1.5, 0.0, 0.0, 0.0]
    seconds_, needed = motion_duration(arm_plan(metadata, start, end), chain, fixed,
                                       metadata['joint_limits'], 1.5, 4.712388, 2.0)
    assert seconds_ == 2.0 and max(needed, key=needed.get) == 'minimum_time'


def test_a_long_move_is_timed_by_the_tool_speed_and_respects_it(arm):
    """The retimed path must peak at (not above) the linear limit when that limit decides."""
    from rby1_cumotion.executor.execution import motion_duration
    metadata, chain, fixed = arm
    start = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    end = [0.6, -0.9, 0.3, -0.4, 0.2, 0.3, 0.0]
    path = arm_plan(metadata, start, end)
    seconds_, needed = motion_duration(path, chain, fixed, metadata['joint_limits'], 0.2, 50.0, 0.1)
    assert max(needed, key=needed.get) == 'linear_velocity_limit'
    moved, _ = retime(path, seconds_, 0.01, metadata['joint_limits'])
    poses = [chain.pose({**fixed, **dict(zip(moved.joint_names, p.positions))}) for p in moved.points]
    speeds = [np.linalg.norm(b[:3, 3] - a[:3, 3]) / 0.01 for a, b in zip(poses, poses[1:])]
    assert 0.18 <= max(speeds) <= 0.2 * 1.02


def test_the_joint_limits_can_decide_and_the_result_passes_retime(arm):
    from rby1_cumotion.executor.execution import motion_duration
    metadata, chain, fixed = arm
    start = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    end = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 2.5]   # wrist roll only: tool barely moves
    path = arm_plan(metadata, start, end)
    seconds_, needed = motion_duration(path, chain, fixed, metadata['joint_limits'], 1.5, 50.0, 0.1)
    assert max(needed, key=needed.get) == 'joint_velocity_limits'
    _, load = retime(path, seconds_, 0.05, metadata['joint_limits'])
    assert load <= 1.0


def test_choose_source_insists_on_the_connected_robot(tmp_path):
    from rby1_cumotion.bringup.prepare import choose_source
    bundle = tmp_path / 'm_1_2'
    bundle.mkdir()
    (bundle / 'model.json').write_text('{"model": "m_1_2"}')
    assert choose_source('m', 1.2, '', '', tmp_path) == bundle
    assert choose_source('M', 1.2, 'm_1_2', '', tmp_path) == bundle
    with pytest.raises(ValueError, match="RB-Y1 'a'"):
        choose_source('a', 1.2, 'm_1_2', '', tmp_path)
    with pytest.raises(ValueError, match='reports m_1_3'):
        choose_source('m', 1.3, 'm_1_2', '', tmp_path)
    with pytest.raises(ValueError, match='No bundle'):
        choose_source('m', 1.0, '', '', tmp_path)


def test_prepare_refuses_to_start_next_to_another_cumotion_launch(tmp_path):
    from rby1_cumotion.bringup.prepare import check_alone
    (tmp_path / '42').mkdir()
    (tmp_path / '42' / 'cmdline').write_bytes(b'/usr/bin/python3\0/opt/ros/humble/lib/x/cumotion_planner_node\0')
    with pytest.raises(RuntimeError, match='Another cuMotion launch'):
        check_alone(str(tmp_path))
    (tmp_path / '42' / 'cmdline').write_bytes(b'bash\0-c\0ros2 launch rby1_cumotion demo.launch.py\0')
    check_alone(str(tmp_path))


# --- joint impedance and stream ownership ------------------------------------------

ROBOT_JOINTS = ([f'torso_{i}' for i in range(6)] + [f'right_arm_{i}' for i in range(7)]
                + [f'left_arm_{i}' for i in range(7)] + ['head_0', 'head_1'])
IMPEDANCE = {'enabled': True, 'stiffness': 80.0, 'damping_ratio': 0.8, 'torque_limit': 12.0}


def test_impedance_covers_exactly_the_group_parts():
    from rby1_cumotion.executor.execution import impedance_request
    group = [f'right_arm_{i}' for i in range(7)] + [f'torso_{i}' for i in range(6)]
    request = impedance_request(IMPEDANCE, group, ROBOT_JOINTS)
    assert list(request.state) == [True, True, False]  # torso, right_arm, left_arm
    assert list(request.torso_stiffness) == [80.0] * 6
    assert list(request.right_arm_stiffness) == [80.0] * 7
    assert list(request.left_arm_stiffness) == []
    assert list(request.damping_ratio) == [0.8] and list(request.torque_limit) == [12.0]


def test_impedance_off_asks_for_position_everywhere():
    from rby1_cumotion.executor.execution import impedance_request
    request = impedance_request({**IMPEDANCE, 'enabled': False}, [f'left_arm_{i}' for i in range(7)],
                                ROBOT_JOINTS)
    assert list(request.state) == [False, False, False]
    assert list(request.left_arm_stiffness) == []


class _FakeDriver:
    def __init__(self, open_already=()):
        self.calls = []
        self.open = set(open_already)  # stream channels

    def call(self, service, kind, request):
        self.calls.append((service, list(request.state)))

    def switch(self, service, state, parameters='all'):
        """stream_control as the driver answers it: which channels it opened."""
        self.calls.append((service, state, parameters))
        channels = set(parameters.split(','))
        if not state:
            self.open -= channels
            return SimpleNamespace(message='Stream channels closed: ' + parameters + '.')
        opened = channels - self.open
        self.open |= channels
        return SimpleNamespace(message=('Stream channels opened: ' + ', '.join(sorted(opened)) + '.') if opened
                               else 'Stream channels already open: ' + parameters + '.')


class _FakeNode:
    def get_logger(self):
        import logging
        return logging.getLogger('test')


def _runner(open_already, enabled, channels=('arm',)):
    from rby1_cumotion.executor.execution import DriverExecutor, impedance_request
    runner = DriverExecutor.__new__(DriverExecutor)
    runner.node, runner.driver = _FakeNode(), _FakeDriver(open_already)
    runner.goal_handle, runner.stream_closed, runner.client = None, 0.0, object()
    runner.applied_impedance, runner.owned, runner.channels = None, [], channels
    runner.impedance = lambda: impedance_request({**IMPEDANCE, 'enabled': enabled[0]},
                                                 [f'right_arm_{i}' for i in range(7)], ROBOT_JOINTS)
    runner.send = lambda trajectory: None
    return runner


def test_a_stream_channel_someone_else_opened_stays_open():
    runner = _runner(open_already=['arm', 'head'], enabled=[False])
    runner.begin(None)
    runner.stop()
    assert not [call for call in runner.driver.calls if call[:2] == ('stream_control', False)]
    assert runner.driver.open == {'arm', 'head'}


def test_the_stream_channel_this_executor_opened_is_closed():
    runner = _runner(open_already=['head'], enabled=[False])
    runner.begin(None)
    assert runner.driver.calls[-1] == ('stream_control', True, 'arm')  # only its own channel
    runner.stop()
    assert runner.driver.calls[-1] == ('stream_control', False, 'arm')
    assert runner.driver.open == {'head'}  # a head tracker's channel is left alone


def test_a_torso_group_opens_the_torso_channel_too_and_closes_what_it_opened():
    from rby1_cumotion.executor.execution import stream_channels
    joints = [f'torso_{i}' for i in range(6)] + [f'right_arm_{i}' for i in range(7)]
    assert stream_channels(joints) == ('arm', 'torso')
    assert stream_channels(joints[6:]) == ('arm',)
    runner = _runner(open_already=['arm'], enabled=[False], channels=stream_channels(joints))
    runner.begin(None)
    runner.stop()
    assert runner.driver.calls[-1] == ('stream_control', False, 'torso')
    assert runner.driver.open == {'arm'}


def test_impedance_is_sent_when_it_changes_and_not_while_never_enabled():
    enabled = [False]
    runner = _runner(open_already=[], enabled=enabled)
    runner.apply_impedance()
    assert runner.driver.calls == []  # never enabled: the driver's own setting stands
    enabled[0] = True
    runner.apply_impedance()
    runner.apply_impedance()
    enabled[0] = False
    runner.apply_impedance()
    assert runner.driver.calls == [('set_trajectory_impedance', [False, True, False]),
                                   ('set_trajectory_impedance', [False, False, False])]
