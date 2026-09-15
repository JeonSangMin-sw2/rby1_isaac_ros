"""Generate XRDF from the installed driver model, without CUDA or mesh downloads.

Each collision mesh AABB is tiled with boxes and each box is enclosed by a sphere.
This covers the entire AABB (including mesh interiors), but can reject otherwise
valid paths. Only the driver's identity-transform, metre, Z_UP Collada format is
accepted. An unsupported mesh fails explicitly instead of silently losing geometry.
"""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# M v1.3's interlocking wrist needs a fitted collision model: filling its mesh
# AABB produces unavoidable false collisions between wrist links 4 and 6.
SUPPORTED_MODELS = ('m_1_2', 'a_1_2')
# Leave room below the 1024-thread CUDA block size for cuMotion's 100 reserved
# attachment spheres. AABB coverage must not grow without a practical bound.
MAX_MODEL_SPHERES = 900
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = xyz
    return result


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(*[list(map(float, element.get(k, '0 0 0').split())) for k in ('xyz', 'rpy')])


def mesh_vertices(path):
    root = ET.parse(path).getroot()
    unit = root.find('c:asset/c:unit', NS)
    up = root.find('c:asset/c:up_axis', NS)
    if unit is None or float(unit.get('meter')) != 1.0 or up is None or up.text != 'Z_UP':
        raise ValueError(f'Expected metre/Z_UP Collada: {path}')
    for node in root.findall('.//c:visual_scene//c:node', NS):
        for child in node:
            tag = child.tag.split('}')[-1]
            if tag in ('translate', 'rotate', 'scale', 'lookat', 'skew'):
                raise ValueError(f'Unsupported Collada transform in {path}')
            if tag == 'matrix' and not np.allclose(np.fromstring(child.text, sep=' ').reshape(4, 4), np.eye(4)):
                raise ValueError(f'Nonidentity Collada matrix in {path}')
    points = []
    for mesh in root.findall('c:library_geometries/c:geometry/c:mesh', NS):
        for position in mesh.findall('c:vertices/c:input[@semantic="POSITION"]', NS):
            source = mesh.find(f'c:source[@id="{position.get("source")[1:]}"]', NS)
            accessor = source.find('c:technique_common/c:accessor', NS)
            if int(accessor.get('stride', '1')) != 3 or int(accessor.get('offset', '0')) != 0:
                raise ValueError(f'Unsupported Collada vertex layout: {path}')
            points.append(np.fromstring(source.find('c:float_array', NS).text, sep=' ').reshape(-1, 3))
    if not points:
        raise ValueError(f'No vertices in {path}')
    points = np.concatenate(points)
    if not np.isfinite(points).all():
        raise ValueError(f'Nonfinite mesh vertices: {path}')
    return points


def cover_box(lower, upper, cell_size):
    """Enclose every cell in the solid AABB; vertices alone are not sufficient."""
    lower, upper = np.asarray(lower), np.asarray(upper)
    counts = np.maximum(1, np.ceil((upper - lower) / cell_size).astype(int))
    if np.prod(counts) > 10000:
        raise ValueError('Mesh bounds require too many spheres; check mesh scale/cell size')
    step = (upper - lower) / counts
    radius = float(np.linalg.norm(step) / 2 + 1e-6)
    return [{'center': (lower + (np.array(i) + 0.5) * step).tolist(), 'radius': radius}
            for i in itertools.product(*(range(int(n)) for n in counts))]


def use_ros_mesh_uris(root):
    """ROS resource_retriever requires file://; cuRobo uses filesystem paths."""
    for mesh in root.findall('.//mesh'):
        path = Path(mesh.get('filename'))
        if not path.is_absolute() or not path.is_file():
            raise ValueError(f'Missing absolute mesh path: {path}')
        mesh.set('filename', path.as_uri())


def forward_kinematics(root, positions, tip):
    frames = {'base': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in frames:
                continue
            t = origin(joint.find('origin'))
            kind = joint.get('type')
            if kind != 'fixed':
                q = positions[joint.get('name')]
                axis_element = joint.find('axis')
                axis = np.array(list(map(float, axis_element.get('xyz').split())))
                delta = np.eye(4)
                if kind == 'prismatic':
                    delta[:3, 3] = axis * q
                elif kind in ('revolute', 'continuous'):
                    axis /= np.linalg.norm(axis)
                    x, y, z = axis
                    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    delta[:3, :3] = np.eye(3) + math.sin(q) * skew + (1 - math.cos(q)) * (skew @ skew)
                else:
                    raise ValueError(f'Unsupported joint type: {kind}')
                t = t @ delta
            frames[child] = frames[parent] @ t
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('URDF must be a connected tree rooted at base')
    return frames if tip is None else frames[tip]


def prepare(description_dir, moveit_dir, output, model='m_1_2', arm='right', posture_file=None, cell_size=0.08):
    if model not in SUPPORTED_MODELS or arm not in ('right', 'left'):
        raise ValueError('Unsupported model or arm')
    if not math.isfinite(cell_size) or not 0.02 <= cell_size <= 0.1:
        raise ValueError('cell_size must be between 0.02 and 0.1 metres')
    model_type, version = model.split('_', 1)
    name = f'RBY1_{model_type.upper()}_v{version}'
    source = Path(description_dir) / f'urdf/rby1{model_type}/model_v{version}.urdf'
    semantic = Path(moveit_dir) / f'config/{name}.srdf'
    root, srdf = ET.parse(source).getroot(), ET.parse(semantic).getroot()
    active = [f'{arm}_arm_{i}' for i in range(7)]
    # The driver does not report finger positions. Model their entire travel as
    # fixed collision envelopes, so missing finger feedback never becomes an
    # invented joint state. The SDK still owns the physical grippers separately.
    finger_travel = {}
    for joint in root.findall('joint'):
        if joint.get('name').startswith('gripper_finger_'):
            axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
            limit = joint.find('limit')
            finger_travel[joint.find('child').get('link')] = (
                axis * float(limit.get('lower')), axis * float(limit.get('upper')))
            joint.set('type', 'fixed')
            for tag in ('limit', 'axis', 'mimic'):
                element = joint.find(tag)
                if element is not None:
                    joint.remove(element)
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') != 'fixed'}
    defaults = {j: 0.0 for j in joints}
    defaults.update({'right_arm_1': -0.65, 'left_arm_1': 0.65,
                     'right_arm_3': -0.8, 'left_arm_3': -0.8})
    if posture_file:
        values = yaml.safe_load(Path(posture_file).read_text())['initial_positions']
        if set(values) - set(joints):
            raise ValueError(f'Unknown posture joints: {set(values) - set(joints)}')
        defaults.update({k: float(v) for k, v in values.items()})
    limits = {}
    for key, joint in joints.items():
        limit = joint.find('limit')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        limits[key] = {'lower': lower, 'upper': upper, 'velocity': float(limit.get('velocity'))}
        if not math.isfinite(defaults[key]) or not lower <= defaults[key] <= upper:
            raise ValueError(f'Posture outside joint limits: {key}={defaults[key]}')
    spheres, meshes = {}, {}
    for link in root.findall('link'):
        entries = []
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'Unsupported collision geometry on {link.get("name")}')
            uri = mesh.get('filename')
            prefix = 'package://rby1_description/'
            if not uri.startswith(prefix):
                raise ValueError(f'Unexpected mesh URI: {uri}')
            path = Path(description_dir) / uri[len(prefix):]
            if path not in meshes:
                meshes[path] = mesh_vertices(path)
            scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
            points = meshes[path] * scale
            t = origin(collision.find('origin'))
            points = points @ t[:3, :3].T + t[:3, 3]
            lower, upper = points.min(axis=0), points.max(axis=0)
            if link.get('name') in finger_travel:
                start, end = finger_travel[link.get('name')]
                lower += np.minimum(start, end)
                upper += np.maximum(start, end)
                collision.clear()
                ET.SubElement(collision, 'origin', xyz=' '.join(map(str, (lower + upper) / 2)), rpy='0 0 0')
                geometry = ET.SubElement(collision, 'geometry')
                ET.SubElement(geometry, 'box', size=' '.join(map(str, upper - lower)))
            entries.extend(cover_box(lower, upper, cell_size))
        if entries:
            spheres[link.get('name')] = entries
    sphere_count = sum(map(len, spheres.values()))
    if sphere_count > MAX_MODEL_SPHERES:
        raise ValueError(f'{sphere_count} collision spheres exceed the {MAX_MODEL_SPHERES} budget; '
                         'increase --cell-size (default: 0.08 m)')
    # Preserve SRDF policy, including documented adjacent/default collision exclusions.
    ignore = {key: [] for key in spheres}
    for pair in srdf.findall('disable_collisions'):
        a, b = pair.get('link1'), pair.get('link2')
        if a in spheres and b in spheres:
            ignore[a].append(b)
    # Links with identical active ancestors are rigid relative to one another
    # after XRDF joint locking. Avoid testing intersections within that assembly.
    ancestry = {'base': ()}
    pending = list(root.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in ancestry]
        if not ready:
            raise ValueError('URDF must be rooted at base')
        for joint in ready:
            ancestry[joint.find('child').get('link')] = ancestry[joint.find('parent').get('link')] + (
                (joint.get('name'),) if joint.get('name') in active else ())
            pending.remove(joint)
    for a, b in itertools.combinations(spheres, 2):
        if ancestry[a] == ancestry[b] and b not in ignore[a] and a not in ignore[b]:
            ignore[a].append(b)
    xrdf = {
        'format': 'xrdf', 'format_version': 1.0,
        'default_joint_positions': defaults,
        'cspace': {'joint_names': active, 'acceleration_limits': [1.0] * 7, 'jerk_limits': [10.0] * 7},
        'tool_frames': [f'ee_{arm}'],
        'collision': {'geometry': 'robot', 'buffer_distance': 0.0},
        'self_collision': {'geometry': 'robot', 'ignore': ignore, 'buffer_distance': {}},
        'geometry': {'robot': {'spheres': spheres}},
    }
    # Absolute paths must be generated separately in each host/container environment.
    for mesh in root.findall('.//mesh'):
        uri = mesh.get('filename')
        mesh.set('filename', str((Path(description_dir) / uri.removeprefix('package://rby1_description/')).resolve()))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    urdf_bytes = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    xrdf_text = yaml.safe_dump(xrdf, sort_keys=False)
    (output / 'robot.urdf').write_bytes(urdf_bytes)
    (output / 'robot.xrdf').write_text(xrdf_text)
    (output / 'initial_positions.yaml').write_text(yaml.safe_dump({'initial_positions': defaults}))
    manifest = {'model': model, 'robot_name': name, 'arm': arm, 'group': f'{arm}_arm',
                'tool_frame': f'ee_{arm}', 'base_frame': 'base', 'active_joints': active,
                'default_positions': defaults, 'locked_joints': {k: v for k, v in defaults.items() if k not in active},
                'joint_limits': limits, 'sphere_count': sphere_count,
                'cell_size': cell_size,
                'urdf_sha256': hashlib.sha256(urdf_bytes).hexdigest(),
                'xrdf_sha256': hashlib.sha256(xrdf_text.encode()).hexdigest()}
    (output / 'model.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def load_model(directory):
    directory = Path(directory)
    metadata = json.loads((directory / 'model.json').read_text())
    if metadata['sphere_count'] > MAX_MODEL_SPHERES:
        raise ValueError('Collision sphere budget exceeded; regenerate with prepare_model')
    for kind in ('urdf', 'xrdf'):
        if hashlib.sha256((directory / f'robot.{kind}').read_bytes()).hexdigest() != metadata[f'{kind}_sha256']:
            raise ValueError('Model artifacts changed; regenerate with prepare_model')
    return metadata, ET.parse(directory / 'robot.urdf').getroot()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, default='m_1_2')
    parser.add_argument('--arm', choices=('right', 'left'), default='right')
    parser.add_argument('--output', required=True)
    parser.add_argument('--posture', help='YAML containing initial_positions overrides (radians/metres)')
    parser.add_argument('--cell-size', type=float, default=0.08)
    parser.add_argument('--description-dir', help='Override installed rby1_description share directory')
    parser.add_argument('--moveit-dir', help='Override installed rby1_moveit_<model> share directory')
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    result = prepare(args.description_dir or get_package_share_directory('rby1_description'),
                     args.moveit_dir or get_package_share_directory(f'rby1_moveit_{args.model}'),
                     args.output, args.model, args.arm, args.posture, args.cell_size)
    print(f'Prepared {result["model"]} {result["group"]}: {result["sphere_count"]} spheres in {args.output}')


if __name__ == '__main__':
    main()
