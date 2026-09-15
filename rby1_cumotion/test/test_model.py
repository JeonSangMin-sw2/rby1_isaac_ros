import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from rby1_cumotion.model import (cover_box, forward_kinematics, load_model, prepare,
                                SUPPORTED_MODELS, use_ros_mesh_uris)


def test_spheres_cover_solid_box():
    lower, upper = np.array([-0.13, -0.03, 0.01]), np.array([0.22, 0.14, 0.05])
    spheres = cover_box(lower, upper, 0.06)
    # Include the interior and every boundary; testing only mesh vertices misses holes.
    points = np.array(list(itertools.product(*(np.linspace(a, b, 11) for a, b in zip(lower, upper)))))
    distances = np.linalg.norm(points[:, None] - np.array([s['center'] for s in spheres])[None], axis=2)
    assert (distances <= np.array([s['radius'] for s in spheres])[None]).any(axis=1).all()


@pytest.fixture(scope='module', params=list(itertools.product(SUPPORTED_MODELS, ('right', 'left'))))
def prepared(request, tmp_path_factory):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    model, arm = request.param
    try:
        description = get_package_share_directory('rby1_description')
        moveit = get_package_share_directory(f'rby1_moveit_{model}')
    except PackageNotFoundError:
        pytest.skip('Source the RBY1 driver workspace to test installed models')
    output = tmp_path_factory.mktemp(model)
    prepare(description, moveit, output, model=model, arm=arm)
    return output


def test_model_uses_driver_limits_and_all_collision_links(prepared):
    metadata, root = load_model(prepared)
    xrdf = yaml.safe_load((prepared / 'robot.xrdf').read_text())
    spheres = xrdf['geometry']['robot']['spheres']
    assert set(spheres) == {l.get('name') for l in root.findall('link') if l.find('collision') is not None}
    assert metadata['active_joints'] == [f'{metadata["arm"]}_arm_{i}' for i in range(7)]
    assert metadata['sphere_count'] + 100 <= 1024
    assert len(metadata['locked_joints']) == 15
    assert 'gripper_finger_r1_joint' not in metadata['default_positions']
    assert root.find('joint[@name="gripper_finger_r1_joint"]').get('type') == 'fixed'
    assert root.find('link[@name="gripper_finger_r1"]/collision/geometry/box') is not None
    for joint in root.findall('joint'):
        if joint.get('name') in metadata['joint_limits']:
            assert metadata['joint_limits'][joint.get('name')]['upper'] == float(joint.find('limit').get('upper'))


def test_default_posture_has_no_enabled_sphere_collisions(prepared):
    metadata, root = load_model(prepared)
    xrdf = yaml.safe_load((prepared / 'robot.xrdf').read_text())
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


def test_artifact_tampering_is_detected(prepared, tmp_path):
    for name in ('robot.xrdf', 'robot.urdf', 'model.json'):
        (tmp_path / name).write_bytes((prepared / name).read_bytes())
    with (tmp_path / 'robot.xrdf').open('a') as stream:
        stream.write('\n# changed\n')
    with pytest.raises(ValueError, match='changed'):
        load_model(tmp_path)


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

    _, root = load_model(prepared)
    use_ros_mesh_uris(root)
    for uri in {mesh.get('filename') for mesh in root.findall('.//mesh')}:
        parsed = urlparse(uri)
        assert parsed.scheme == 'file'
        assert get(uri) == Path(unquote(parsed.path)).read_bytes()


def test_missing_mesh_rejected_before_startup(tmp_path):
    root = ET.fromstring(f'<robot><mesh filename="{tmp_path}/missing.dae"/></robot>')
    with pytest.raises(ValueError, match='Missing absolute mesh'):
        use_ros_mesh_uris(root)
