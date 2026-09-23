import copy
import itertools
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from rby1_cumotion.model import (BUNDLE_CONFIGS, bundle_groups, canonicalize_joint_axes,
                                CAPSULE_TOLERANCE, capsule_spheres, check_frames_agree,
                                controllers_for, DEFAULT_GROUPS, forward_kinematics,
                                load_model, MESH_DIRECTORY, prepare, prepare_group,
                                PLACEHOLDER_RADIUS, sdk_collision_model, sdk_urdf,
                                srdf_groups, SUPPORTED_MODELS,
                                terminal_links, use_ros_mesh_uris)


@pytest.mark.parametrize('radius,length', [(0.095, 0.16), (0.035, 0.0), (0.155, 0.9)])
def test_spheres_cover_the_whole_capsule(radius, length):
    """Balls on a segment leave gaps between them unless each one is widened."""
    frame = np.eye(4)
    spheres = capsule_spheres(radius, length, frame)
    centres = np.array([s['center'] for s in spheres])
    radii = np.array([s['radius'] for s in spheres])
    # Sample the capsule: points within `radius` of the segment, ends included.
    axis = np.linspace(-length / 2, length / 2, 40)
    ring = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    fraction = np.linspace(0, 1, 6)
    points = [[0, 0, -length / 2 - radius], [0, 0, length / 2 + radius]]
    for z in axis:
        for f in fraction:
            for a in ring:
                points.append([radius * f * np.cos(a), radius * f * np.sin(a), z])
    points = np.array(points)
    covered = np.linalg.norm(points[:, None] - centres[None], axis=2) <= radii[None] + 1e-12
    assert covered.any(axis=1).all()
    # ... and the cover must not be wasteful: no sphere exceeds the tolerance.
    assert radii.max() <= radius * (1 + CAPSULE_TOLERANCE) + 1e-12


@pytest.fixture(scope='module', params=SUPPORTED_MODELS)
def prepared(request, tmp_path_factory):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        description = get_package_share_directory('rby1_description')
        moveit = get_package_share_directory(f'rby1_moveit_{request.param}')
    except PackageNotFoundError:
        pytest.skip('Source the RBY1 driver workspace to test installed models')
    output = tmp_path_factory.mktemp(request.param)
    prepare(description, moveit, output, model=request.param)
    return output


@pytest.fixture(scope='module')
def sdk(prepared):
    return ET.parse(json.loads((prepared / 'model.json').read_text())['sdk_model']).getroot()


@pytest.fixture(params=[g.replace('+', '_') for g in DEFAULT_GROUPS])
def group(request):
    return request.param


def test_model_uses_driver_limits_and_sdk_capsule_links(prepared, group, sdk):
    metadata, root = load_model(prepared, group)
    spheres = yaml.safe_load((prepared / 'spheres.yaml').read_text())
    capsuled = {l.get('name') for l in sdk.findall('link')
                if l.find('collision/geometry/capsule') is not None}
    placeholders = set(metadata['placeholder_links'])
    # Exactly the SDK's capsule links, plus the placeholders that keep cuRobo's
    # chain complete -- no other link contributes collision geometry.
    assert set(spheres) == capsuled | placeholders
    assert not (capsuled & placeholders)
    assert set(spheres) <= {l.get('name') for l in root.findall('link')}
    # A placeholder must be inert: cuRobo keeps radii at or below -1 negative.
    for link in placeholders:
        assert [s['radius'] for s in spheres[link]] == [pytest.approx(PLACEHOLDER_RADIUS)]
        assert PLACEHOLDER_RADIUS <= -1.0
    assert metadata['sphere_count'] == sum(len(spheres[l]) for l in capsuled)
    assert metadata['sphere_count'] + 100 <= 1024
    # Active plus locked must account for every movable joint, with no overlap.
    assert not set(metadata['active_joints']) & set(metadata['locked_joints'])
    assert (set(metadata['active_joints']) | set(metadata['locked_joints'])
            == set(metadata['default_positions']))
    assert 'gripper_finger_r1_joint' not in metadata['default_positions']
    assert root.find('joint[@name="gripper_finger_r1_joint"]').get('type') == 'fixed'
    assert root.find('link[@name="gripper_finger_r1"]/collision/geometry/box') is not None
    for joint in root.findall('joint'):
        if joint.get('name') in metadata['joint_limits']:
            assert metadata['joint_limits'][joint.get('name')]['upper'] == float(joint.find('limit').get('upper'))


def test_default_posture_has_no_enabled_sphere_collisions(prepared, group):
    metadata, root = load_model(prepared, group)
    xrdf = yaml.safe_load(Path(metadata['xrdf_path']).read_text())
    spheres, ignore = xrdf['geometry']['robot']['spheres'], xrdf['self_collision']['ignore']
    frames = forward_kinematics(root, metadata['default_positions'], None)
    for a, b in itertools.combinations(spheres, 2):
        if b in ignore[a] or a in ignore[b]:
            continue
        ca = np.array([s['center'] for s in spheres[a]]) @ frames[a][:3, :3].T + frames[a][:3, 3]
        cb = np.array([s['center'] for s in spheres[b]]) @ frames[b][:3, :3].T + frames[b][:3, 3]
        ra, rb = np.array([s['radius'] for s in spheres[a]]), np.array([s['radius'] for s in spheres[b]])
        distance = np.linalg.norm(ca[:, None] - cb[None], axis=2) - ra[:, None] - rb[None]
        assert distance.min() >= 0, (a, b, float(distance.min()))


@pytest.mark.parametrize('target', ['robot.urdf', 'groups/right_arm/robot.xrdf'])
def test_artifact_tampering_is_detected(prepared, tmp_path, target):
    tampered = tmp_path / 'tampered'
    shutil.copytree(prepared, tampered)
    with (tampered / target).open('a') as stream:
        stream.write('\n<!-- changed -->\n')
    with pytest.raises(ValueError, match='changed'):
        load_model(tampered, 'right_arm')


def test_fk_matches_an_analytic_chain():
    root = ET.fromstring('''<robot><joint name="j" type="revolute">
    <parent link="base"/><child link="arm"/><origin xyz="1 0 0"/>
    <axis xyz="0 0 1"/></joint><joint name="tool" type="fixed">
    <parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>''')
    tip = forward_kinematics(root, {'j': np.pi / 2}, 'tip')
    np.testing.assert_allclose(tip[:3, 3], [1, 1, 0], atol=1e-10)


def test_ros_mesh_resources_are_readable(prepared):
    from urllib.parse import unquote, urlparse
    from resource_retriever import get

    _, root = load_model(prepared, 'right_arm')
    use_ros_mesh_uris(root)
    for uri in {mesh.get('filename') for mesh in root.findall('.//mesh')}:
        parsed = urlparse(uri)
        assert parsed.scheme == 'file'
        assert get(uri) == Path(unquote(parsed.path)).read_bytes()


def test_missing_mesh_rejected_before_startup(tmp_path):
    root = ET.fromstring(f'<robot><mesh filename="{tmp_path}/missing.dae"/></robot>')
    with pytest.raises(ValueError, match='Missing absolute mesh'):
        use_ros_mesh_uris(root)


def test_bundle_carries_everything_moveit_and_cumotion_need(prepared, group):
    metadata, root = load_model(prepared, group)
    kind, version = metadata['model'].split('_', 1)
    assert metadata['robot_name'] == f'RBY1_{kind.upper()}_v{version}'
    for required in ('robot.srdf', 'spheres.yaml', *BUNDLE_CONFIGS):
        assert (prepared / required).is_file(), required
    stored = ET.parse(prepared / 'robot.urdf').getroot()
    uris = {mesh.get('filename') for mesh in stored.findall('.//mesh')}
    assert len(uris) == metadata['mesh_count']
    for uri in uris:  # portable: relative, and present inside the bundle
        assert not Path(uri).is_absolute() and uri.startswith(MESH_DIRECTORY + '/')
        assert (prepared / uri).is_file()
    for mesh in root.findall('.//mesh'):  # load_model resolves them for consumers
        assert Path(mesh.get('filename')).is_file()


def test_groups_share_one_copy_of_the_heavy_artifacts(prepared):
    """A group must cost kilobytes, not another copy of the meshes."""
    assert bundle_groups(prepared) == sorted(g.replace('+', '_') for g in DEFAULT_GROUPS)
    assert not (prepared / 'robot.xrdf').exists()  # per-group now, not model-level
    for name in bundle_groups(prepared):
        entry = prepared / 'groups' / name
        assert {p.name for p in entry.iterdir()} == {'robot.xrdf', 'group.json'}
    heavy = sum(p.stat().st_size for p in (prepared / MESH_DIRECTORY).iterdir())
    groups = sum(p.stat().st_size for p in (prepared / 'groups').rglob('*') if p.is_file())
    assert groups < heavy / 10


def test_group_selection_drives_joints_controllers_and_tool(prepared):
    right, _ = load_model(prepared, 'right_arm')
    left, _ = load_model(prepared, 'left_arm')
    assert right['active_joints'] == [f'right_arm_{i}' for i in range(7)]
    assert left['active_joints'] == [f'left_arm_{i}' for i in range(7)]
    assert (right['tool_frame'], left['tool_frame']) == ('ee_right', 'ee_left')
    assert right['controllers'] == ['right_arm_controller']
    assert left['controllers'] == ['left_arm_controller']
    assert 'torso_0' in right['locked_joints']
    assert right['sphere_count'] == left['sphere_count']  # spheres are model-level


def test_composite_group_merges_joints_and_controllers(prepared):
    """SRDF groups legitimately name fixed joints; only movable ones form a c-space."""
    root = ET.parse(prepared / 'robot.urdf').getroot()
    resolved = srdf_groups(ET.parse(prepared / 'robot.srdf').getroot(), root)
    movable = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
    merged = [j for j in dict.fromkeys(resolved['right_arm']['joints'] + resolved['torso']['joints'])
              if j in movable]
    assert merged == [f'right_arm_{i}' for i in range(7)] + [f'torso_{i}' for i in range(6)]
    assert resolved['right_arm']['tips'][0] == 'ee_right'
    controllers = yaml.safe_load((prepared / 'ros2_controllers.yaml').read_text())
    assert sorted(controllers_for(controllers, merged)) == [
        'right_arm_controller', 'torso_controller']


def test_torso_group_builds_and_unlocks_the_torso_joints(prepared):
    """What blocked this was the sphere model, not the group machinery."""
    detail = prepare_group(prepared, 'right_arm+torso')
    assert detail['group'] == 'right_arm_torso'
    assert [j for j in detail['active_joints'] if j.startswith('torso_')] == [
        f'torso_{i}' for i in range(6)]
    assert sorted(detail['controllers']) == ['right_arm_controller', 'torso_controller']
    assert not any(j.startswith('torso_') for j in detail['locked_joints'])


def test_sdk_mask_is_what_lets_the_torso_move(prepared, sdk):
    """The torso capsules overlap by design; the SDK's own mask excludes the pair.

    Without honouring it, link_torso_2 and link_torso_4 read as a collision at
    the default posture and every query from a torso group would fail.
    """
    spheres, unchecked = sdk_collision_model(sdk)
    assert ['link_torso_2', 'link_torso_4'] in unchecked
    metadata, root = load_model(prepared, 'right_arm_torso')
    ignored = yaml.safe_load(Path(metadata['xrdf_path']).read_text())['self_collision']['ignore']
    assert 'link_torso_4' in ignored['link_torso_2'] or 'link_torso_2' in ignored['link_torso_4']
    # The pair really does overlap, so the exclusion is load bearing.
    frames = forward_kinematics(root, metadata['default_positions'], None)
    placed = {k: (np.array([s['center'] for s in spheres[k]]) @ frames[k][:3, :3].T + frames[k][:3, 3],
                  np.array([s['radius'] for s in spheres[k]])) for k in ('link_torso_2', 'link_torso_4')}
    (ca, ra), (cb, rb) = placed['link_torso_2'], placed['link_torso_4']
    assert (np.linalg.norm(ca[:, None] - cb[None], axis=2) - ra[:, None] - rb[None]).min() < 0


def test_capsule_origins_need_both_models_to_agree(prepared, sdk):
    """A capsule is placed in a link frame, so a moved frame silently misplaces it."""
    root = ET.parse(prepared / 'robot.urdf').getroot()
    links = sorted(yaml.safe_load((prepared / 'spheres.yaml').read_text()))
    assert check_frames_agree(sdk, root, links) < 1e-12
    moved = copy.deepcopy(sdk)
    moved.find('joint[@name="torso_0"]/origin').set('xyz', '0 0 0.01')
    with pytest.raises(ValueError, match='disagree'):
        check_frames_agree(moved, root, links)


def test_srdf_groups_find_a_tip_without_a_chain_element():
    """v1.0's SRDF lists a group's links instead of declaring a chain."""
    root = ET.fromstring(
        '<robot>'
        '<joint name="a" type="revolute"><parent link="base"/><child link="l1"/></joint>'
        '<joint name="b" type="fixed"><parent link="l1"/><child link="tool"/></joint>'
        '<joint name="c" type="fixed"><parent link="tool"/><child link="outside"/></joint>'
        '</robot>')
    assert terminal_links(root, ['base', 'l1', 'tool']) == ['tool']
    srdf = ET.fromstring('<robot><group name="arm">'
                         '<link name="l1"/><link name="tool"/><joint name="a"/></group></robot>')
    assert srdf_groups(srdf, root)['arm']['tips'] == ['tool']
    assert srdf_groups(srdf)['arm']['tips'] == []


def test_unknown_group_is_rejected(prepared):
    with pytest.raises(ValueError, match='Unknown group'):
        load_model(prepared, 'no_such_group')
    with pytest.raises(ValueError, match='select one'):
        load_model(prepared)


def test_incomplete_bundle_is_rejected(prepared, tmp_path):
    broken = tmp_path / 'broken'
    shutil.copytree(prepared, broken)
    (broken / BUNDLE_CONFIGS[0]).unlink()
    with pytest.raises(ValueError, match=BUNDLE_CONFIGS[0]):
        load_model(broken, 'right_arm')


def test_bundle_works_after_relocation(prepared, tmp_path, group):
    """A bundle copied to another machine or container must load unchanged."""
    moved = tmp_path / 'relocated'
    shutil.copytree(prepared, moved)
    metadata, root = load_model(moved, group)
    for mesh in root.findall('.//mesh'):
        assert Path(mesh.get('filename')).is_file()
        assert Path(mesh.get('filename')).is_relative_to(moved)
    original = load_model(prepared, group)[0]
    assert metadata.pop('xrdf_path') != original.pop('xrdf_path')  # only the path moves
    assert metadata == original


def test_overlapping_controllers_are_refused():
    """Two active controllers claiming one joint would fight over its interface."""
    controllers = {
        'controller_manager': {'ros__parameters': {
            'a_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
            'b_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        'a_controller': {'ros__parameters': {'joints': ['j0', 'j1']}},
        'b_controller': {'ros__parameters': {'joints': ['j1', 'j2']}},
    }
    with pytest.raises(ValueError, match='claim the same joints'):
        controllers_for(controllers, ['j0', 'j1', 'j2'])
    with pytest.raises(ValueError, match='No driver controller covers'):
        controllers_for(controllers, ['j0', 'j1', 'j9'])


def test_canonicalization_preserves_every_link_pose():
    """cuRobo rejects tilted axes; the rewrite must not move any frame."""
    original = ET.fromstring('''<robot>
    <link name="base"/><link name="tilted"/><link name="tool"/>
    <joint name="j" type="revolute"><parent link="base"/><child link="tilted"/>
      <origin xyz="0 0 0.3" rpy="0 0 0.2"/><axis xyz="0 0.939693 -0.34202"/>
      <limit lower="-1" upper="1" velocity="1"/></joint>
    <joint name="t" type="fixed"><parent link="tilted"/><child link="tool"/>
      <origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/></joint></robot>''')
    rewritten = copy.deepcopy(original)
    assert canonicalize_joint_axes(rewritten) == ['j']
    assert rewritten.find('joint[@name="j"]/axis').get('xyz') == '0.0 1.0 0.0'
    for q in (-1.0, -0.3, 0.0, 0.7, 1.0):
        before = forward_kinematics(original, {'j': q}, None)
        after = forward_kinematics(rewritten, {'j': q}, None)
        for link in before:
            np.testing.assert_allclose(after[link], before[link], atol=1e-12)
