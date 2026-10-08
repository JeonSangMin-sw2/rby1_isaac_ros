"""Attached modules as covering spheres on the hand (attached.py)."""

from pathlib import Path

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive

from rby1_cumotion.executor.attached import attached_spheres, cover_box, cover_cylinder, is_free, MAX_SPHERES
from rby1_cumotion.model import load_model, prepare, SUPPORTED_MODELS


def covered(points, centres, radii):
    distance = np.linalg.norm(points[:, None] - centres[None], axis=2) - radii[None]
    return bool((distance.min(axis=1) <= 1e-9).all())


def box_points(size, n=7):
    axes = [np.linspace(-s / 2, s / 2, n) for s in size]
    return np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3)


@pytest.mark.parametrize('size,count', [((0.04, 0.04, 0.12), 12), ((0.3, 0.2, 0.01), 20), ((0.05, 0.05, 0.05), 1)])
def test_box_spheres_cover_every_corner(size, count):
    centres, radii = cover_box(size, count)
    assert len(radii) <= max(count, 1)
    assert covered(box_points(size), centres, radii)


def test_box_spheres_use_the_budget_to_stay_close_to_the_faces():
    # The gripper body: with 33 spheres they reach 1.4 cm past its faces, not the 3 cm of four big ones.
    size = np.array([0.126, 0.065, 0.073])
    centres, radii = cover_box(size, 33)
    assert 16 < len(radii) <= 33
    assert (np.abs(centres) + radii[:, None] - size / 2).max() < 0.015


def test_cylinder_spheres_cover_rim_and_ends():
    centres, radii = cover_cylinder(0.015, 0.12, 10)
    angle = np.linspace(0, 2 * np.pi, 16)
    rim = np.array([[0.015 * np.cos(a), 0.015 * np.sin(a), z] for a in angle for z in np.linspace(-0.06, 0.06, 9)])
    assert covered(rim, centres, radii)


@pytest.fixture(scope='module')
def bundle(tmp_path_factory):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        description = get_package_share_directory('rby1_description')
        moveit = get_package_share_directory(f'rby1_moveit_{SUPPORTED_MODELS[0]}')
    except PackageNotFoundError:
        pytest.skip('Source the RBY1 driver workspace to test installed models')
    output = tmp_path_factory.mktemp('attached')
    prepare(description, moveit, output, model=SUPPORTED_MODELS[0])
    return load_model(output, 'right_arm')


def module(name, link, size, z=0.0):
    attached = AttachedCollisionObject(link_name=link)
    attached.object = CollisionObject(id=name)
    attached.object.header.frame_id = link
    attached.object.pose.orientation.w = 1.0
    attached.object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
    pose = Pose()
    pose.position.z = z
    pose.orientation.w = 1.0
    attached.object.primitive_poses.append(pose)
    return attached


def test_modules_on_the_hand_ride_on_the_tool_frame(bundle):
    metadata, root = bundle
    spheres, notes = attached_spheres([module('tool', 'ee_right', (0.02, 0.02, 0.1), z=-0.05)], metadata, root)
    assert 0 < len(spheres) <= MAX_SPHERES and 'spheres' in notes['tool']
    centres, radii = np.array([s[:3] for s in spheres]), np.array([s[3] for s in spheres])
    corners = box_points((0.02, 0.02, 0.1), 3) + [0, 0, -0.05]  # ee_right is the tool frame
    assert covered(corners, centres, radii)


def test_modules_elsewhere_are_named_not_planned_with(bundle):
    metadata, root = bundle
    spheres, notes = attached_spheres([module('camera', 'link_head_2', (0.09, 0.025, 0.025)),
                                       module('left_tool', 'ee_left', (0.02, 0.02, 0.1))], metadata, root)
    assert spheres == []
    assert 'head envelope' in notes['camera']
    assert 'does not see it' in notes['left_tool']


def test_free_parts_are_left_out(bundle):
    """Named in free_objects (names, * allowed), a part gets no spheres: it may touch anything."""
    metadata, root = bundle
    parts = [module('gripper_right_body', 'ee_right', (0.12, 0.06, 0.07), z=-0.035),
             module('gripper_right_finger_1', 'ee_right', (0.016, 0.03, 0.06), z=-0.105),
             module('gripper_right_finger_2', 'ee_right', (0.016, 0.03, 0.06), z=-0.105)]
    everything, _ = attached_spheres(parts, metadata, root)
    spheres, notes = attached_spheres(parts, metadata, root, free=['gripper_*_finger_*'])
    assert 'spheres' in notes['gripper_right_body']
    assert 'left out' in notes['gripper_right_finger_1'] and 'left out' in notes['gripper_right_finger_2']
    lowest = min(s[2] - s[3] for s in spheres)
    assert lowest > -0.11 > -0.13 > min(s[2] - s[3] for s in everything)  # nothing where the fingers are
    assert attached_spheres(parts, metadata, root, free=['gripper_*'])[0] == []
    assert not is_free('gripper_right_body', ['']) and is_free('tool', ['tool'])


def test_free_names_reach_moveit_as_allowed_collisions(bundle):
    """MoveIt checks every path against its own model: the same parts, and the robot links, are allowed there."""
    from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
    from rby1_cumotion.executor.attached import allow_touching, free_names
    _, root = bundle
    parts = [module('gripper_right_body', 'ee_right', (0.12, 0.06, 0.07)),
             module('gripper_right_finger_1', 'ee_right', (0.016, 0.03, 0.06))]
    names = free_names(parts, root, ['gripper_*finger*'])
    assert 'gripper_right_finger_1' in names and 'gripper_finger_r1' in names and 'gripper_finger_l2' in names
    assert 'gripper_right_body' not in names and 'gripper_right' not in names
    assert free_names(parts, root, []) == []
    before = AllowedCollisionMatrix(entry_names=['a', 'b'],
                                    entry_values=[AllowedCollisionEntry(enabled=[False, True]),
                                                  AllowedCollisionEntry(enabled=[True, False])],
                                    default_entry_names=['gripper_finger_r1'], default_entry_values=[False])
    after = allow_touching(before, ['gripper_finger_r1', 'gripper_right_finger_1'])
    assert dict(zip(after.default_entry_names, after.default_entry_values)) == {
        'gripper_finger_r1': True, 'gripper_right_finger_1': True}
    assert after.entry_names == ['a', 'b'] and before.default_entry_values == [False]  # the original is kept


def test_too_many_spheres_is_refused(bundle):
    metadata, root = bundle
    many = [module(f'm{i}', 'ee_right', (0.02, 0.02, 0.2)) for i in range(30)]  # 4 spheres each
    with pytest.raises(ValueError, match='spheres'):
        attached_spheres(many, metadata, root)


def test_executor_mirrors_attached_modules():
    source = (Path(__file__).resolve().parents[1] / 'rby1_cumotion' / 'executor'
              / 'target_executor.py').read_text()
    assert 'ROBOT_STATE_ATTACHED_OBJECTS' in source and 'UpdateLinkSpheres' in source
