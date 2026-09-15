from copy import deepcopy

import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rby1_cumotion.model import transform
from rby1_cumotion.move_arm import check_locked, quaternion, validate_trajectory


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


@pytest.mark.parametrize('angle', [0.0, 0.4, np.pi, -np.pi])
def test_quaternion_round_trip(angle):
    rotation = transform(rpy=(0, 0, angle))[:3, :3]
    q = quaternion(rotation)
    expected = np.array([0, 0, np.sin(angle / 2), np.cos(angle / 2)])
    assert abs(np.dot(q, expected)) == pytest.approx(1.0)
