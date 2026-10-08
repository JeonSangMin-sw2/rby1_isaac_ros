"""Replacing a running trajectory without a jolt, and tracking obstacles that move."""

import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive

from rby1_cumotion.avoidance import MotionTracker, objects_from_scene, swept
from rby1_cumotion.execution import (ahead, back_off, resume_from, retime, seconds, slow_to_stop,
                                     splice, stamp, state_at, times_of)
from test_execution import LIMITS, planned

STEP = 0.05


@pytest.fixture
def running():
    """A retimed 3 s move of two joints, as the driver would be running it."""
    times = np.linspace(0.0, 10.0, 32)
    s = times / times[-1]
    moved, _ = retime(planned(times, np.stack([0.8 * s, -0.5 * s], axis=1)), 3.0, STEP, LIMITS)
    return moved


def detour(start, end, bulge=0.2, count=40):
    """A replanned path from `start` to `end` that swings out on joint 1."""
    s = np.linspace(0.0, 1.0, count)[:, None]
    path = start + s * (np.asarray(end) - start)
    path[:, 1] += bulge * np.sin(np.pi * s[:, 0])
    return path


def test_state_at_follows_the_trajectory(running):
    q, v = state_at(running, 1.5)
    ref = next(p for p in running.points if abs(seconds(p.time_from_start) - 1.5) < 1e-6)
    assert q == pytest.approx(ref.positions, abs=1e-9)
    assert v == pytest.approx(ref.velocities, abs=1e-6)


def test_ahead_takes_only_the_window(running):
    rows = ahead(running, 1.0, 0.5)
    assert len(rows) == 10  # waypoints in (1.0, 1.5] at 0.05 s


def test_splice_keeps_the_near_future_and_continues_the_velocity(running):
    t_now, t_join = 1.0, 1.4
    q_join, v_join = state_at(running, t_join)
    new = splice(running, t_now, t_join, detour(q_join, [0.8, -0.5]), LIMITS, STEP, floor=1.6)
    times = times_of(new)
    assert times[0] == pytest.approx(STEP) and np.all(np.diff(times) > 0)
    # Up to the join it is exactly what the driver was about to send.
    for point in new.points:
        t = seconds(point.time_from_start)
        if t < t_join - t_now - 1e-9:
            q, _ = state_at(running, t_now + t)
            assert point.positions == pytest.approx(q.tolist(), abs=1e-9)
    # At the join the velocity carries on instead of dropping to zero.
    _, v_new = state_at(new, t_join - t_now)
    assert v_new == pytest.approx(v_join, abs=1e-3)
    assert np.linalg.norm(v_join) > 0.1
    # It ends at the target, at rest, and no faster than asked (floor).
    assert new.points[-1].positions == pytest.approx([0.8, -0.5], abs=1e-9)
    assert new.points[-1].velocities == pytest.approx([0.0, 0.0])
    assert times[-1] >= (t_join - t_now) + 1.6 - 1e-9


def test_splice_respects_joint_velocity_limits(running):
    q_join, _ = state_at(running, 1.4)
    new = splice(running, 1.0, 1.4, detour(q_join, [0.8, -0.5], bulge=1.5), LIMITS, STEP)
    worst = max(abs(v) for p in new.points for v in p.velocities)
    assert worst <= 2.0 * 1.02


def test_splice_refuses_a_path_from_elsewhere(running):
    with pytest.raises(ValueError, match='does not start'):
        splice(running, 1.0, 1.4, detour(np.array([0.0, 0.0]), [0.8, -0.5]), LIMITS, STEP)


def box(name, x, y, z, size=0.1):
    obj = CollisionObject(id=name)
    obj.pose.orientation.w = 1.0
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size] * 3)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    obj.primitive_poses = [pose]
    return obj


def test_scene_objects_become_poses_in_base():
    objects = objects_from_scene([box('shelf', 0.2, -0.3, 1.1)])
    assert objects[0]['name'] == 'shelf_0' and objects[0]['kind'] == 'box'
    assert objects[0]['pose'][:3, 3] == pytest.approx([0.2, -0.3, 1.1])


def test_a_mesh_obstacle_is_read_with_its_bounding_box():
    """The collision checks get the mesh; where a distance is worked out here its box stands in."""
    from geometry_msgs.msg import Point, Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle
    from rby1_cumotion.avoidance import MESH, signature, surface_distance
    obj = CollisionObject(id='bracket')
    obj.pose.position.x, obj.pose.orientation.w = 0.5, 1.0
    mesh = Mesh()
    for x, y, z in ((0, 0, 0), (0.2, 0, 0), (0, 0.1, 0), (0, 0, 0.4)):  # a tetrahedron, corner at its origin
        mesh.vertices.append(Point(x=float(x), y=float(y), z=float(z)))
    for a, b, c in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
        mesh.triangles.append(MeshTriangle(vertex_indices=[a, b, c]))
    pose = Pose()
    pose.orientation.w = 1.0
    obj.meshes.append(mesh)
    obj.mesh_poses.append(pose)
    found, = objects_from_scene([obj])
    assert found['kind'] == MESH and found['dims'] == pytest.approx([0.2, 0.1, 0.4])
    assert found['pose'][:3, 3] == pytest.approx([0.6, 0.05, 0.2])           # the box's centre, in base
    assert (found['vertices'] + found['pose'][:3, 3]).min(axis=0) == pytest.approx([0.5, 0.0, 0.0])
    assert len(found['faces']) == 4
    assert surface_distance([[0.6, 0.05, 0.2], [0.9, 0.05, 0.2]], found) == pytest.approx([-0.05, 0.2])
    moved = dict(found, vertices=found['vertices'] * 1.01)
    assert signature([found]) != signature([moved])


def test_a_moving_obstacle_is_swept_ahead_a_standing_one_is_not():
    tracker = MotionTracker()
    tracker.update(objects_from_scene([box('ball', 0.5, 0.0, 1.0), box('wall', 0.3, 0.3, 1.0)]), 0.0)
    moving = tracker.update(objects_from_scene([box('ball', 0.5, 0.1, 1.0), box('wall', 0.3, 0.3, 1.0)]), 0.5)
    assert set(moving) == {'ball_0'} and moving['ball_0'] == pytest.approx([0.0, 0.2, 0.0])
    objects = objects_from_scene([box('ball', 0.5, 0.1, 1.0), box('wall', 0.3, 0.3, 1.0)])
    result = swept(objects, moving, 1.0, count=4)
    ahead_y = sorted(o['pose'][1, 3] for o in result if o['name'].startswith('ball_0'))
    assert ahead_y == pytest.approx([0.1, 0.15, 0.2, 0.25, 0.3])
    assert sum(o['name'].startswith('wall_0') for o in result) == 1


def test_a_path_planned_from_a_moving_start_is_trimmed_to_where_it_passes_the_start():
    """MotionGen puts its first points about v * 0.07 s behind a moving start."""
    from rby1_cumotion.avoidance import from_start
    q, v = np.array([0.3, -0.2]), np.array([0.15, -0.1])
    path = q + np.outer(np.linspace(-0.07, 1.0, 44), v)
    trimmed = from_start(path, q)
    assert trimmed[0] == pytest.approx(q) and np.all(np.diff(trimmed[:, 0]) > 0)
    with pytest.raises(ValueError, match='passes'):
        from_start(path + 0.5, q)


def test_surface_distance_for_each_shape():
    from rby1_cumotion.avoidance import surface_distance
    at = np.eye(4)
    at[:3, 3] = [1.0, 0.0, 0.0]
    points = np.array([[1.0, 0.0, 0.0], [1.3, 0.0, 0.0], [1.0, 0.0, 0.5]])
    assert surface_distance(points, {'kind': 'box', 'dims': [0.2, 0.2, 0.2], 'pose': at}) == \
        pytest.approx([-0.1, 0.2, 0.4])
    assert surface_distance(points, {'kind': 'sphere', 'dims': [0.1], 'pose': at}) == \
        pytest.approx([-0.1, 0.2, 0.4])
    assert surface_distance(points, {'kind': 'cylinder', 'dims': [0.4, 0.1], 'pose': at}) == \
        pytest.approx([-0.1, 0.2, 0.3])


def test_a_moving_obstacle_is_compared_with_the_arm_at_the_same_moment():
    """Swept over the whole window it would meet the arm; at matching times it does not."""
    from rby1_cumotion.avoidance import moving_collision
    ball = {'name': 'ball', 'kind': 'sphere', 'dims': [0.05], 'pose': np.eye(4)}
    ball['pose'][:3, 3] = [0.0, -0.5, 1.0]
    v = {'ball': np.array([0.0, 1.0, 0.0])}               # reaches y=0 at t=0.5 s
    times = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
    arm_leaving = np.array([[[0.0, 0.0 + 0.5 * t, 1.0, 0.05]] for t in times])   # moves away along +y
    assert moving_collision(arm_leaving, times, [ball], v) is None
    arm_waiting = np.array([[[0.0, 0.0, 1.0, 0.05]] for _ in times])            # stays at y=0
    assert moving_collision(arm_waiting, times, [ball], v) == 3   # touching at 0.4 s: 0.1 m = both radii


def test_only_copies_that_cover_the_arm_now_are_left_out_of_the_plan():
    from rby1_cumotion.avoidance import clear_of
    arm = np.array([[0.0, 0.0, 1.0, 0.05], [0.0, 0.0, 1.1, -10.0]])      # one real sphere, one placeholder
    def ball(y):
        pose = np.eye(4)
        pose[:3, 3] = [0.0, y, 1.0]
        return {'name': f'b{y}', 'kind': 'sphere', 'dims': [0.05], 'pose': pose}
    kept = clear_of([ball(-0.3), ball(-0.05), ball(0.3)], arm)
    assert [o['name'] for o in kept] == ['b-0.3', 'b0.3']


def path_distance(trajectory, reference):
    """Largest distance of `trajectory`'s positions from `reference`'s path (densely sampled)."""
    dense = np.array([state_at(reference, t)[0] for t in np.linspace(0.0, times_of(reference)[-1], 600)])
    return max(np.linalg.norm(dense - np.asarray(p.positions), axis=1).min() for p in trajectory.points)


def test_braking_stays_on_the_path_and_ends_at_rest_where_it_says(running):
    t_now = 1.0
    stop, s_stop = slow_to_stop(running, t_now, 0.6, 2.0, STEP)
    assert s_stop == pytest.approx(1.3)
    assert path_distance(stop, running) < 2e-3
    times = times_of(stop)
    assert times[0] == pytest.approx(STEP) and np.all(np.diff(times) > 0)
    assert times[-1] == pytest.approx(0.6 + 2.0, abs=STEP)
    # Starts at the speed it was going, slows monotonically, ends at rest at s_stop.
    speeds = [np.linalg.norm(p.velocities) for p in stop.points]
    assert speeds[0] == pytest.approx(np.linalg.norm(state_at(running, t_now)[1]), rel=0.15)
    assert np.all(np.diff(speeds[:12]) <= 1e-9)
    assert stop.points[-1].positions == pytest.approx(state_at(running, 1.3)[0].tolist(), abs=1e-9)
    assert speeds[-1] == 0.0


def test_resuming_carries_on_along_the_rest_of_the_path(running):
    end = times_of(running)[-1]
    rest = resume_from(running, 1.3, end, 0.6, STEP)
    assert path_distance(rest, running) < 2e-3
    assert rest.points[0].positions == pytest.approx(state_at(running, 1.3)[0].tolist(), abs=2e-3)
    speeds = [np.linalg.norm(p.velocities) for p in rest.points]
    assert speeds[0] < 0.1 * max(speeds) and speeds[-1] == 0.0
    assert rest.points[-1].positions == pytest.approx(running.points[-1].positions, abs=1e-9)
    # The ramp costs half its length: the rest takes (end - s_stop) + ramp / 2.
    assert times_of(rest)[-1] == pytest.approx(end - 1.3 + 0.3, abs=STEP)


def test_resuming_at_the_end_is_just_the_end_pose(running):
    end = times_of(running)[-1]
    rest = resume_from(running, end, end, 0.6, STEP)
    assert len(rest.points) == 1
    assert rest.points[0].positions == pytest.approx(running.points[-1].positions, abs=1e-9)


def test_backing_off_retraces_the_path_and_holds(running):
    braking, s_stop = slow_to_stop(running, 1.0, 0.6, 0.0, STEP)
    back = back_off(running, s_stop, s_stop - 0.5, 3.0, STEP, start=braking)
    assert path_distance(back, running) < 2e-3
    times = times_of(back)
    assert np.all(np.diff(times) > 0)
    assert back.points[-1].positions == pytest.approx(state_at(running, s_stop - 0.5)[0].tolist(), abs=1e-9)
    assert np.linalg.norm(back.points[-1].velocities) == 0.0
    # Never faster along the path than the plan itself.
    for point in back.points:
        assert np.all(np.abs(point.velocities) <= np.abs(np.array([p.velocities for p in running.points])).max(axis=0) + 1e-9)


def test_stamp_never_steps_back_in_time():
    # 0.05 added up 40 times is 1.9999999999999998: it once became 1.0 s.
    t, last = 0.0, 0.0
    for _ in range(400):
        t += 0.05
        now = seconds(stamp(t))
        assert now == pytest.approx(t, abs=1e-9) and now > last
        last = now


def test_resumed_trajectory_has_no_gaps(running):
    rest = resume_from(running, 1.3, times_of(running)[-1], 0.6, STEP)
    assert np.diff(times_of(rest)).max() <= STEP + 1e-9
