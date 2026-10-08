"""Tracking mode: leading a moving target, the IK servo's joint steps, switching modes (no GPU, no ROS graph)."""

from types import SimpleNamespace

import numpy as np
import pytest

from rby1_cumotion.target_executor import TargetExecutor
from rby1_cumotion.tracking import joint_rates, REST_SPEED, SERVO_GAIN, servo_step, TargetMotion


def at(x, y, z):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix


def steady(motion, speed, samples=80, period=0.1):
    """`samples` of a target moving along x at `speed` (enough to forget how it started); the time of the last."""
    for k in range(samples):
        motion.update(at(speed * period * k, 0.0, 0.0), period * k)
    return period * (samples - 1)


def test_a_moving_target_is_aimed_ahead_but_not_too_far():
    motion = TargetMotion(smoothing=0.0)  # every sample as it is
    now = steady(motion, 0.1)  # 0.1 m/s along x
    assert np.allclose(motion.velocity, [0.1, 0.0, 0.0])
    assert np.allclose(motion.at(now, lead=0.5, limit=0.1)[:3, 3], [0.1 * now + 0.05, 0, 0])
    assert np.allclose(motion.at(now, lead=5.0, limit=0.1)[:3, 3], [0.1 * now + 0.1, 0, 0])  # capped at lead_max
    assert np.allclose(motion.at()[:3, 3], [0.1 * now, 0, 0])  # no longer coming: where it was last seen


def test_a_fresh_target_is_not_aimed_ahead_of_yet():
    """The first samples give a speed, but a guess: no aiming ahead until a few more agree."""
    motion = TargetMotion(smoothing=0.0)
    for k in range(3):
        motion.update(at(0.01 * k, 0.0, 0.0), 0.1 * k)
    assert np.allclose(motion.velocity, [0.1, 0.0, 0.0])
    assert np.allclose(motion.heading(), 0.0)
    assert motion.at(0.2, lead=0.5, limit=0.1)[0, 3] < 0.021


def test_an_old_sample_is_carried_on_only_so_far():
    motion = TargetMotion(smoothing=0.0)
    now = steady(motion, 0.1)
    here = 0.1 * now
    assert np.allclose(motion.at(now + 0.03, carry=0.1)[:3, 3], [here + 0.003, 0, 0])  # 30 ms old: 3 mm further
    assert np.allclose(motion.at(now + 0.8, carry=0.1)[:3, 3], [here + 0.01, 0, 0])    # not for 0.8 s


def test_a_target_after_a_pause_does_not_count_as_fast():
    motion = TargetMotion(smoothing=0.0, reset_after=0.5)
    motion.update(at(0.0, 0.0, 0.0), 0.0)
    motion.update(at(0.3, 0.0, 0.0), 2.0)  # 2 s later: a new start, not 0.15 m/s
    assert np.allclose(motion.velocity, 0.0) and np.allclose(motion.position, [0.3, 0, 0])


def test_a_jump_is_taken_at_once_and_at_rest():
    motion = TargetMotion(smoothing=0.7)
    for k in range(10):
        motion.update(at(0.5, 0.0, 1.0), k / 30)
    motion.update(at(0.5, 0.1, 1.0), 10 / 30)  # 10 cm in one frame
    assert np.allclose(motion.position, [0.5, 0.1, 1.0]) and np.allclose(motion.velocity, 0.0)


def test_a_target_at_rest_is_not_aimed_ahead_of():
    motion = TargetMotion(smoothing=0.0)
    now = steady(motion, 0.005)  # 5 mm/s: noise, not motion
    assert np.allclose(motion.heading(), 0.0)
    assert motion.at(now, lead=0.5, limit=0.1)[0, 3] <= 0.005 * now + 1e-9
    motion = TargetMotion(smoothing=0.0)
    now = steady(motion, 0.025)  # 25 mm/s: half way to being aimed ahead of in full
    assert np.allclose(motion.heading(), [0.0125, 0, 0])


def test_two_samples_at_once_do_not_start_it_afresh():
    motion = TargetMotion(smoothing=0.0)
    now = steady(motion, 0.1)
    motion.update(at(0.1 * now + 0.001, 0.0, 0.0), now + 1e-4)  # delivered in a burst with the last
    assert np.allclose(motion.velocity, [0.1, 0.0, 0.0]) and motion.samples > 5


def test_a_noisier_camera_needs_a_faster_target_to_count_as_moving():
    def rest_speed(noise):
        rng, motion = np.random.default_rng(3), TargetMotion()
        for k in range(200):
            motion.update(at(*(np.array([0.5, 0.0, 1.0]) + rng.normal(0.0, noise, 3))), k / 30)
        return motion.rest_speed()
    assert rest_speed(0.0) == pytest.approx(REST_SPEED)
    assert REST_SPEED < rest_speed(0.001) < 0.035 < rest_speed(0.003) < 0.10


def test_camera_noise_at_rest_does_not_reach_the_arm():
    """3 mm of noise on a target at rest: the aim stays within 2 mm (aimed ahead of, it was 10)."""
    rng, motion, aimed = np.random.default_rng(4), TargetMotion(), []
    for k in range(400):
        motion.update(at(*(np.array([0.5, 0.0, 1.0]) + rng.normal(0.0, 0.003, 3))), k / 30)
        aimed.append(motion.at(k / 30, lead=0.2, limit=0.1)[:3, 3])
    assert np.linalg.norm(np.std(np.array(aimed)[100:], axis=0)) < 0.002


def test_smoothing_rides_out_camera_noise_on_a_moving_target():
    """1 mm of noise on a target at 10 cm/s, aimed 0.17 s ahead: every sample as it is misses by several times more."""
    miss = {}
    for smoothing in (0.0, 0.75):
        rng, motion, errors = np.random.default_rng(1), TargetMotion(smoothing), []
        for k in range(300):
            t = k / 30
            motion.update(at(*(np.array([0.1 * t, 0.0, 1.0]) + rng.normal(0.0, 0.001, 3))), t)
            if k > 60:
                errors.append(np.linalg.norm(motion.at(t, lead=0.17, limit=0.1)[:3, 3] - [0.1 * (t + 0.17), 0.0, 1.0]))
        miss[smoothing] = float(np.mean(errors))
    assert miss[0.75] < 0.003 and miss[0.0] > 3 * miss[0.75]


def test_a_turning_target_is_aimed_ahead_along_its_turn():
    """On a 6 cm circle at 13 cm/s, 0.17 s ahead: within 7 mm, and heading the way it will go then."""
    def circle(t):
        a = 2 * np.pi * t / 3.0
        return np.array([0.6, 0.06 * np.sin(a), 1.0 + 0.06 * np.cos(a)])
    motion, misses = TargetMotion(smoothing=0.75), []
    for k in range(180):
        t = k / 30
        motion.update(at(*circle(t)), t)
        if k > 90:
            misses.append(np.linalg.norm(motion.at(t, lead=0.17, limit=0.1)[:3, 3] - circle(t + 0.17)))
    assert np.mean(misses) < 0.007
    will = (circle(t + 0.17 + 1e-4) - circle(t + 0.17)) / 1e-4
    heading = motion.heading(t, 0.17)
    assert heading @ will > 0.95 * np.linalg.norm(heading) * np.linalg.norm(will)  # within 18 deg
    assert np.linalg.norm(heading) == pytest.approx(np.linalg.norm(will), rel=0.4)


def test_smoothing_still_follows_a_steady_move_without_falling_behind():
    motion = TargetMotion(smoothing=0.75)
    for k in range(90):
        motion.update(at(0.1 * k / 30, 0.0, 1.0), k / 30)  # 10 cm/s
    assert motion.velocity[0] == pytest.approx(0.1, abs=1e-3)
    assert motion.position[0] == pytest.approx(0.1 * 89 / 30, abs=1e-4)


def executor(busy=False, tracking=False):
    return SimpleNamespace(busy=busy, tracking=tracking, tracking_cfg={'rate': 50.0},
                           param=lambda name: '/rby1/target_pose')


def test_switching_to_tracking_waits_for_a_move_to_end():
    node = executor(busy=True)
    response = TargetExecutor.on_set_tracking(node, SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and 'in progress' in response.message and not node.tracking


def test_switching_modes():
    node = executor()
    response = TargetExecutor.on_set_tracking(node, SimpleNamespace(data=True), SimpleNamespace())
    assert response.success and node.tracking and '50 Hz' in response.message
    response = TargetExecutor.on_set_tracking(node, SimpleNamespace(data=False), SimpleNamespace())
    assert response.success and not node.tracking


def run_servo(goal, steps, max_speed=2.0, max_acceleration=10.0, dt=0.02):
    position, velocity, trace = np.zeros(len(goal)), np.zeros(len(goal)), []
    for _ in range(steps):
        position, velocity = servo_step(position, velocity, goal, dt, max_speed, max_acceleration)
        trace.append((position.copy(), velocity.copy()))
    return trace


def test_servo_step_reaches_a_jumped_goal_without_passing_it():
    trace = run_servo([0.2, -0.5], 60)
    positions = np.array([p for p, _ in trace])
    assert positions[-1] == pytest.approx([0.2, -0.5], abs=1e-4)
    assert positions[:, 0].max() <= 0.2 + 1e-3 and positions[:, 1].min() >= -0.5 - 1e-3
    assert np.abs(trace[-1][1]).max() < 1e-3  # at rest on the goal


def test_servo_step_keeps_speed_and_acceleration_limits():
    trace = run_servo([3.0], 100, max_speed=1.0, max_acceleration=5.0)
    velocities = np.array([0.0] + [float(v[0]) for _, v in trace])
    assert np.abs(velocities).max() <= 1.0 + 1e-9
    assert np.abs(np.diff(velocities)).max() <= 5.0 * 0.02 + 1e-9
    assert velocities[1] == pytest.approx(0.1)  # a ramp from rest, not a jump to full speed


def test_servo_step_trails_a_goal_whose_speed_it_does_not_know():
    """A goal moving at 0.5 rad/s is trailed by v / gain, less the step it has just taken."""
    position, velocity = np.zeros(1), np.zeros(1)
    for k in range(200):
        goal = [0.5 * 0.02 * (k + 1)]
        position, velocity = servo_step(position, velocity, goal, 0.02, 2.0, 10.0)
    assert goal[0] - position[0] == pytest.approx(0.5 / SERVO_GAIN - 0.5 * 0.02, abs=1e-3)


def test_servo_step_does_not_chase_small_changes():
    """A goal that shakes by 1 mrad from step to step moves the joint a fraction of that."""
    position, velocity, moves = np.zeros(1), np.zeros(1), []
    for k in range(200):
        position, velocity = servo_step(position, velocity, [0.001 * (-1) ** k], 0.02, 2.0, 10.0)
        moves.append(position[0])
    assert np.ptp(moves[50:]) < 0.0004


def test_joint_rates_move_the_tool_as_asked_and_stay_small_when_singular():
    jacobian = np.zeros((6, 7))
    jacobian[0, 0], jacobian[1, 1], jacobian[2, 2] = 0.5, 0.5, 0.5   # three joints each move one axis
    jacobian[3, 3], jacobian[4, 4], jacobian[5, 5] = 1.0, 1.0, 1.0   # three turn the tool
    rates = joint_rates(jacobian, [0.1, 0.0, -0.05])
    assert jacobian @ rates == pytest.approx([0.1, 0.0, -0.05, 0.0, 0.0, 0.0], abs=2e-3)
    jacobian[0, 0] = 1e-4  # x can no longer be moved
    assert np.abs(joint_rates(jacobian, [0.1, 0.0, 0.0])).max() < 0.01


def test_servo_step_rides_along_with_a_goal_whose_speed_it_knows():
    """Told how fast the goal moves, the joint ends on it instead of v / 2a behind."""
    position, velocity = np.zeros(1), np.zeros(1)
    for k in range(100):
        goal = [0.5 * 0.02 * (k + 1)]
        position, velocity = servo_step(position, velocity, goal, 0.02, 2.0, 10.0, goal_velocity=[0.5])
    assert abs(goal[0] - position[0]) < 1e-3 + 0.5 * 0.02  # at most the goal's own step behind, not v / gain
    assert velocity[0] == pytest.approx(0.5, abs=1e-3)


def test_servo_step_goal_velocity_stays_inside_the_limits():
    position, velocity = servo_step(np.zeros(1), np.zeros(1), [1.0], 0.02, 1.0, 5.0, goal_velocity=[50.0])
    assert velocity[0] == pytest.approx(0.1)  # still a ramp from rest
    for _ in range(50):
        position, velocity = servo_step(position, velocity, [100.0], 0.02, 1.0, 5.0, goal_velocity=[50.0])
    assert velocity[0] <= 1.0 + 1e-9


def test_tracking_method_is_validated():
    from rby1_cumotion import settings
    check = settings.SECTIONS['tracking']['method'][1]
    assert check('mpc') and check('ik') and not check('servo')
    assert settings.load()['tracking']['method'] in settings.TRACKING_METHODS
