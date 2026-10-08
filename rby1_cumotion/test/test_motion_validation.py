from copy import deepcopy

import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rby1_cumotion.model import transform
from rby1_cumotion.planning import check_locked, PostureMismatch, quaternion, validate_trajectory


@pytest.fixture
def valid():
    names = [f'right_arm_{i}' for i in range(7)]
    metadata = {'active_joints': names, 'locked_joints': {'torso_0': 0.0},
                'joint_limits': {n: {'lower': -1.0, 'upper': 1.0, 'velocity': 2.0} for n in names}}
    trajectory = JointTrajectory(joint_names=names.copy())
    for second in (0, 1):
        point = JointTrajectoryPoint(positions=[0.1 * second] * 7)
        point.time_from_start.sec = second
        trajectory.points.append(point)
    return trajectory, metadata, dict.fromkeys(names, 0.0)


def test_valid_and_reordered_trajectory(valid):
    validate_trajectory(*valid)
    valid[0].joint_names.reverse()
    validate_trajectory(*valid)


@pytest.mark.parametrize('failure', ['empty', 'duplicate', 'other_arm', 'nan', 'time', 'position_limit', 'velocity_limit', 'derivatives', 'start'])
def test_bad_trajectory_rejected(valid, failure):
    trajectory, metadata, current = deepcopy(valid)
    if failure == 'empty':
        trajectory.points.clear()
    elif failure == 'duplicate':
        trajectory.joint_names[-1] = trajectory.joint_names[0]
    elif failure == 'other_arm':
        trajectory.joint_names[0] = 'left_arm_0'
    elif failure == 'nan':
        trajectory.points[-1].positions[0] = float('nan')
    elif failure == 'time':
        trajectory.points[-1].time_from_start.sec = 0
    elif failure == 'position_limit':
        trajectory.points[-1].positions[0] = 1.1
    elif failure == 'velocity_limit':
        trajectory.points[-1].velocities = [3.0] * 7
    elif failure == 'derivatives':
        trajectory.points[-1].accelerations = [0.0]
    elif failure == 'start':
        current['right_arm_0'] = 0.5
    with pytest.raises(ValueError):
        validate_trajectory(trajectory, metadata, current)


@pytest.mark.parametrize('positions', [{}, {'torso_0': float('nan')}, {'torso_0': 0.2}])
def test_locked_joint_must_be_observed_and_match(valid, positions):
    with pytest.raises(ValueError):
        check_locked(valid[1], positions, 0.01)


def test_wrong_posture_is_its_own_error_and_names_the_fix(valid):
    """A missing joint state may still arrive, so callers wait for it; a robot at
    another posture will not move by itself, so waiting or warmup retries only
    burn time (it cost ~80 s against the simulator). Keep the two apart. The fix is
    to relaunch, which reads the posture again."""
    with pytest.raises(PostureMismatch, match='Restart the launch'):
        check_locked(valid[1], {'torso_0': 0.2}, 0.01)
    with pytest.raises(ValueError) as missing:
        check_locked(valid[1], {}, 0.01)
    assert not isinstance(missing.value, PostureMismatch)


@pytest.mark.parametrize('angle', [0.0, 0.4, np.pi, -np.pi])
def test_quaternion_round_trip(angle):
    rotation = transform(rpy=(0, 0, angle))[:3, :3]
    q = quaternion(rotation)
    expected = np.array([0, 0, np.sin(angle / 2), np.cos(angle / 2)])
    assert abs(np.dot(q, expected)) == pytest.approx(1.0)


def test_planning_failure_names_the_code_and_where_to_look():
    """`status=6, MoveIt error=-1` was all a user saw when an obstacle blocked the target."""
    from moveit_msgs.msg import MoveItErrorCodes
    from rby1_cumotion.planning import planning_failure
    blocked = planning_failure(6, MoveItErrorCodes.PLANNING_FAILED)
    assert 'PLANNING_FAILED' in blocked and 'obstacle' in blocked and 'IK_FAIL' in blocked
    assert 'TIMED_OUT' in planning_failure(6, MoveItErrorCodes.TIMED_OUT)
    assert 'UNKNOWN' in planning_failure(6, 12345)


def test_warmup_tries_every_axis_when_one_direction_is_blocked():
    """An obstacle in the default warmup direction kept target_executor from starting."""
    from rby1_cumotion.planning import warmup_offsets
    offsets = warmup_offsets([0.03, 0.0, 0.0])
    assert offsets[0] == [0.03, 0.0, 0.0] and offsets[1] == [-0.03, 0.0, 0.0]
    assert len(offsets) == 6 and len({tuple(o) for o in offsets}) == 6
    assert [0.0, 0.0, 0.03] in offsets and [0.0, -0.03, 0.0] in offsets
    assert all(abs(np.linalg.norm(o) - 0.03) < 1e-12 for o in offsets)


def test_missing_joint_states_fail_once_and_name_the_driver():
    """With the driver stopped, check_plan retried 'Waiting for joint state: torso_0'
    six times (~80 s) without saying why."""
    from rby1_cumotion.planning import PlanningClient

    class Stub:
        def param(self, name):
            return '/joint_states'

        def fresh_snapshot(self, timeout):
            raise ValueError('Waiting for joint state: torso_0')

    with pytest.raises(RuntimeError, match='Is the RB-Y1 driver running'):
        PlanningClient.await_joint_states(Stub(), timeout=0.1)

    class Moved(Stub):
        def fresh_snapshot(self, timeout):
            raise PostureMismatch('Restart demo.launch.py')

    with pytest.raises(PostureMismatch):
        PlanningClient.await_joint_states(Moved(), timeout=0.1)
