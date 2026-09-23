"""Generate XRDF from the installed driver model, without CUDA or mesh downloads.

Collision spheres come from the capsules the RBY1 SDK ships in its own URDFs --
the same envelopes the robot enforces on board -- so the planner and the robot
agree on what counts as a self-collision. A capsule is exactly a segment swept by
a sphere, so decomposing one is not an approximation: spacing the sphere centres
by d and using sqrt(r^2 + (d/2)^2) covers the capsule with a bounded excess.

Links the SDK leaves uncapsuled carry no spheres, matching the SDK's own policy.
Only the driver's identity-transform, metre, Z_UP Collada format is accepted. An
unsupported mesh fails explicitly instead of silently losing geometry.
"""

import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml

SUPPORTED_MODELS = ('m_1_0', 'm_1_1', 'm_1_2', 'm_1_3', 'a_1_0', 'a_1_1', 'a_1_2')
# Leave room below the 1024-thread CUDA block size for cuMotion's 100 reserved
# attachment spheres. Coverage must not grow without a practical bound.
MAX_MODEL_SPHERES = 900
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
# Sphere centres are spaced so the covering sphere exceeds the capsule radius by
# at most this fraction. Tightening it costs spheres roughly in proportion.
CAPSULE_TOLERANCE = 0.02
# cuRobo locks every controlled joint in the URDF, and a joint it can lock must
# lie on the chain it builds -- which is spanned by the tool frame and the links
# carrying spheres. A link the SDK leaves uncapsuled therefore still needs an
# entry, so it gets one sphere that cuRobo disables: radii at or below -1 are
# kept negative, so every distance to them stays negative and never collides.
PLACEHOLDER_RADIUS = -10.0
SDK_ENV = 'RBY1_SDK_MODELS'
DEFAULT_SDK_DIR = Path.home() / 'sdk/rby1-sdk/models'
MESH_PREFIX = 'package://rby1_description/'
MESH_DIRECTORY = 'meshes'
GROUP_DIRECTORY = 'groups'
# One tool frame per group: cuMotion's MoveIt plugin plans to a single end-effector
# pose, so a torso joins an arm as redundancy rather than as a second goal.
DEFAULT_GROUPS = ('right_arm', 'left_arm', 'right_arm+torso', 'left_arm+torso')
# Copied verbatim from the driver's MoveIt package so a bundle carries every
# setting MoveIt and ros2_control need, with no driver workspace present.
BUNDLE_CONFIGS = ('kinematics.yaml', 'joint_limits.yaml',
                  'moveit_controllers.yaml', 'ros2_controllers.yaml')


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


def rpy(rotation):
    """Invert transform()'s Rz(yaw) @ Ry(pitch) @ Rx(roll) convention."""
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[2, 1], rotation[2, 2]))
    if math.isclose(abs(rotation[2, 0]), 1.0, abs_tol=1e-9):  # cos(pitch) == 0
        return 0.0, pitch, math.atan2(-rotation[0, 1], rotation[1, 1])
    return (math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
            math.atan2(rotation[1, 0], rotation[0, 0]))


def set_origin(parent, matrix):
    """Write a 4x4 pose back onto parent's <origin>, creating it when absent."""
    element = parent.find('origin')
    if element is None:
        element = ET.SubElement(parent, 'origin')
    element.set('xyz', ' '.join(repr(float(v)) for v in matrix[:3, 3]))
    element.set('rpy', ' '.join(repr(float(v)) for v in rpy(matrix[:3, :3])))


def align_rotation(axis):
    """Smallest rotation R with R @ e == axis, for the canonical e nearest axis."""
    e = np.zeros(3)
    dominant = int(np.argmax(np.abs(axis)))
    e[dominant] = math.copysign(1.0, axis[dominant])
    cross, cosine = np.cross(e, axis), float(np.dot(e, axis))
    sine = float(np.linalg.norm(cross))
    if sine < 1e-12:  # already canonical; cosine > 0 by construction of e
        return e, np.eye(3)
    skew = np.array([[0, -cross[2], cross[1]], [cross[2], 0, -cross[0]], [-cross[1], cross[0], 0]])
    return e, np.eye(3) + skew + skew @ skew * ((1 - cosine) / sine ** 2)


def pose(rotation):
    return np.block([[rotation, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])


def canonicalize_joint_axes(root):
    """Rewrite tilted joint axes onto +-X/+-Y/+-Z without changing the kinematics.

    cuRobo's JointType enum and its CUDA kernel only encode axis-aligned joints
    ("Arbitrary axis of change is not supported"), but RB-Y1's shoulders are
    tilted 20 degrees. For origin T and unit axis a, choosing R with R @ e == a
    gives T @ Rot(a, q) == (T @ R) @ Rot(e, q) @ R.T. Splitting that identity
    across an inserted massless link keeps the moving joint canonical and parks
    R.T on a fixed joint, so every original link frame, mesh pose and joint
    value survives untouched and driver joint states still map directly.
    """
    links = {link.get('name') for link in root.findall('link')}
    rewritten = []
    for joint in root.findall('joint'):
        element = joint.find('axis')
        if joint.get('type') == 'fixed' or element is None:
            continue
        axis = np.array(list(map(float, element.get('xyz').split())))
        norm = np.linalg.norm(axis)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError(f'Degenerate joint axis: {joint.get("name")}')
        e, rotation = align_rotation(axis / norm)
        element.set('xyz', ' '.join(f'{v:.1f}' for v in e))  # exact +-1 for cuRobo
        if np.allclose(rotation, np.eye(3)):
            continue
        child = joint.find('child')
        intermediate = f'{child.get("link")}_axis_aligned'
        if intermediate in links:
            raise ValueError(f'Link name already taken: {intermediate}')
        links.add(intermediate)
        set_origin(joint, origin(joint.find('origin')) @ pose(rotation))
        ET.SubElement(root, 'link', name=intermediate)
        alignment = ET.SubElement(root, 'joint', name=f'{joint.get("name")}_axis_alignment',
                                  type='fixed')
        ET.SubElement(alignment, 'parent', link=intermediate)
        ET.SubElement(alignment, 'child', link=child.get('link'))
        set_origin(alignment, pose(rotation.T))
        child.set('link', intermediate)
        rewritten.append(joint.get('name'))
    return rewritten


def mesh_vertices(path):
    """Vertices of a Collada mesh, used to size the gripper travel envelope."""
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


def capsule_spheres(radius, length, frame, tolerance=CAPSULE_TOLERANCE):
    """Cover a capsule with spheres, exceeding it by at most `tolerance`.

    Balls of radius r centred on a segment leave gaps between them; widening
    each to sqrt(r^2 + (d/2)^2) for centre spacing d closes the gaps exactly.
    The spacing is chosen so that widening stays within the tolerance.
    """
    if not (radius > 0 and length >= 0 and tolerance > 0):
        raise ValueError('Capsule needs a positive radius and tolerance')
    spacing = 2 * radius * math.sqrt((1 + tolerance) ** 2 - 1)
    count = max(2, int(math.ceil(length / spacing)) + 1) if length > 0 else 1
    offsets = np.linspace(-length / 2, length / 2, count) if count > 1 else np.zeros(1)
    step = float(offsets[1] - offsets[0]) if count > 1 else 0.0
    widened = math.sqrt(radius ** 2 + (step / 2) ** 2)
    centres = np.zeros((count, 3))
    centres[:, 2] = offsets  # URDF capsules extend along their own Z, like cylinders
    centres = centres @ frame[:3, :3].T + frame[:3, 3]
    return [{'center': centre.tolist(), 'radius': widened} for centre in centres]


def sdk_urdf(sdk_dir, model):
    kind, version = model.split('_', 1)
    path = Path(sdk_dir) / f'rby1{kind}/urdf/model_v{version.replace("_", ".")}.urdf'
    if not path.is_file():
        raise ValueError(f'No SDK model at {path}; pass --sdk-dir or set {SDK_ENV}')
    return path


def sdk_collision_model(sdk_root, tolerance=CAPSULE_TOLERANCE):
    """Capsule spheres and the link pairs the SDK deliberately does not test.

    Each capsule carries the SDK's own coltype/colaffinity bitmask. Two links are
    tested only when one side's type meets the other's affinity, which is how the
    SDK excludes pairs that cannot meet -- torso segments above all. Honouring
    that mask is what lets a torso join a planning group: its capsules are
    generous envelopes that overlap by design.
    """
    spheres, masks = {}, {}
    for link in sdk_root.findall('link'):
        entries = []
        for collision in link.findall('collision'):
            capsule = collision.find('geometry/capsule')
            if capsule is None:
                continue
            entries.extend(capsule_spheres(float(capsule.get('radius')),
                                           float(capsule.get('length')),
                                           origin(collision.find('origin')), tolerance))
            kind = int(capsule.get('coltype', '0')), int(capsule.get('colaffinity', '0'))
            previous = masks.setdefault(link.get('name'), kind)
            if previous != kind:
                raise ValueError(f'Conflicting collision masks on {link.get("name")}')
        if entries:
            spheres[link.get('name')] = entries
    if not spheres:
        raise ValueError('SDK model declares no capsules')
    unchecked = []
    for a, b in itertools.combinations(sorted(spheres), 2):
        if not (masks[a][0] & masks[b][1] or masks[b][0] & masks[a][1]):
            unchecked.append([a, b])
    return spheres, unchecked


def check_frames_agree(sdk_root, root, links, samples=25, tolerance=1e-9):
    """Capsule origins are link-local, so both models must place links alike."""
    movable = [{j.get('name') for j in model.findall('joint') if j.get('type') != 'fixed'}
               for model in (sdk_root, root)]
    shared = sorted(movable[0] & movable[1])
    generator = np.random.default_rng(0)
    worst, name = 0.0, None
    for _ in range(samples):
        values = {joint: float(generator.uniform(-0.6, 0.6)) for joint in shared}
        frames = [forward_kinematics(model, {**{j: 0.0 for j in names}, **values}, None)
                  for model, names in zip((sdk_root, root), movable)]
        for link in links:
            if link not in frames[0] or link not in frames[1]:
                raise ValueError(f'SDK capsule link {link} is missing from the driver model')
            error = float(np.abs(frames[0][link] - frames[1][link]).max())
            if error > worst:
                worst, name = error, link
    if worst > tolerance:
        raise ValueError(f'SDK and driver models disagree on {name} by {worst:.3e}; '
                         'capsule origins would land in the wrong place')
    return worst


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


def terminal_links(root, links):
    """Links in the set that nothing else in the set descends from.

    v1.0's SRDF lists a group's links and joints outright instead of declaring a
    <chain>, so the tool frame has to be read off the kinematics.
    """
    parents = {joint.find('child').get('link'): joint.find('parent').get('link')
               for joint in root.findall('joint')}
    inner, ancestors = set(links), set()
    for link in links:
        node = parents.get(link)
        while node is not None:
            if node in inner:
                ancestors.add(node)
            node = parents.get(node)
    return [link for link in links if link not in ancestors]


def srdf_groups(srdf, root=None):
    """Resolve every SRDF group to its joints and chain tips, following subgroups."""
    declared = {group.get('name'): group for group in srdf.findall('group')}
    resolved = {}

    def resolve(name, seen=()):
        if name in resolved:
            return resolved[name]
        if name in seen or name not in declared:
            raise ValueError(f'Cyclic or unknown SRDF group: {name}')
        joints, tips, links = [], [], []
        for child in declared[name]:
            if child.tag == 'joint':
                joints.append(child.get('name'))
            elif child.tag == 'link':
                links.append(child.get('name'))
            elif child.tag == 'chain':
                tips.append(child.get('tip_link'))
            elif child.tag == 'group':
                nested = resolve(child.get('name'), seen + (name,))
                joints.extend(nested['joints'])
                tips.extend(nested['tips'])
        if not tips and links and root is not None:
            tips = terminal_links(root, list(dict.fromkeys(links)))
        resolved[name] = {'joints': list(dict.fromkeys(joints)), 'tips': list(dict.fromkeys(tips))}
        return resolved[name]

    return {name: resolve(name) for name in declared}


def controllers_for(controllers, active):
    """Fewest driver trajectory controllers that exactly cover the active joints."""
    manager = controllers['controller_manager']['ros__parameters']
    candidates = {}
    for key, entry in manager.items():
        if not isinstance(entry, dict) or 'JointTrajectoryController' not in entry.get('type', ''):
            continue
        joints = controllers.get(key, {}).get('ros__parameters', {}).get('joints') or []
        if joints and set(joints) <= set(active):
            candidates[key] = set(joints)
    chosen, covered = [], set()
    for key in sorted(candidates, key=lambda k: (-len(candidates[k]), k)):
        if candidates[key] - covered:
            chosen.append(key)
            covered |= candidates[key]
    if covered != set(active):
        raise ValueError(f'No driver controller covers {sorted(set(active) - covered)}')
    # Two active controllers claiming one joint would fight over its command interface.
    for a, b in itertools.combinations(chosen, 2):
        if candidates[a] & candidates[b]:
            raise ValueError(f'Controllers {a} and {b} claim the same joints')
    return chosen


def reachable_joints(root, targets):
    """Movable joints on the path from the root link to any target link.

    cuRobo builds its chain from the tool frames and the links carrying collision
    spheres, and nothing else. A joint outside that chain cannot move a sphere or
    the tool, so asking cuRobo to lock it raises a KeyError instead of doing
    nothing -- it has never heard of the joint.
    """
    parents = {joint.find('child').get('link'): (joint.find('parent').get('link'), joint)
               for joint in root.findall('joint')}
    reached = set()
    for target in targets:
        node = target
        while node in parents:
            node, joint = parents[node]
            if joint.get('type') != 'fixed':
                reached.add(joint.get('name'))
    return reached


def prepare(description_dir, moveit_dir, output, model='m_1_2', posture_file=None,
            sdk_dir=None, groups=DEFAULT_GROUPS, tolerance=CAPSULE_TOLERANCE):
    """Build the model-level bundle, then add one directory per planning group."""
    if model not in SUPPORTED_MODELS:
        raise ValueError('Unsupported model')
    if not math.isfinite(tolerance) or not 0.001 <= tolerance <= 0.2:
        raise ValueError('tolerance must be between 0.001 and 0.2')
    model_type, version = model.split('_', 1)
    name = f'RBY1_{model_type.upper()}_v{version}'
    source = Path(description_dir) / f'urdf/rby1{model_type}/model_v{version}.urdf'
    semantic = Path(moveit_dir) / f'config/{name}.srdf'
    root = ET.parse(source).getroot()
    # Must precede collision extraction: canonicalizing moves child-frame poses.
    canonicalized = canonicalize_joint_axes(root)
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
    # MoveIt still collision-checks the fingers, so give it the swept envelope
    # in place of one arbitrary opening. cuMotion never sees these links.
    meshes = {}
    for link in root.findall('link'):
        if link.get('name') not in finger_travel:
            continue
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'Unsupported collision geometry on {link.get("name")}')
            uri = mesh.get('filename')
            if not uri.startswith(MESH_PREFIX):
                raise ValueError(f'Unexpected mesh URI: {uri}')
            path = Path(description_dir) / uri[len(MESH_PREFIX):]
            if path not in meshes:
                meshes[path] = mesh_vertices(path)
            scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
            placement = origin(collision.find('origin'))
            points = (meshes[path] * scale) @ placement[:3, :3].T + placement[:3, 3]
            lower, upper = points.min(axis=0), points.max(axis=0)
            start, end = finger_travel[link.get('name')]
            lower += np.minimum(start, end)
            upper += np.maximum(start, end)
            collision.clear()
            ET.SubElement(collision, 'origin', xyz=' '.join(map(str, (lower + upper) / 2)), rpy='0 0 0')
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'box', size=' '.join(map(str, upper - lower)))
    sdk_path = sdk_urdf(sdk_dir or os.environ.get(SDK_ENV) or DEFAULT_SDK_DIR, model)
    sdk_root = ET.parse(sdk_path).getroot()
    spheres, unchecked = sdk_collision_model(sdk_root, tolerance)
    # Capsule origins are expressed in link frames; if the two models placed a
    # link differently the spheres would silently sit in the wrong place.
    frame_error = check_frames_agree(sdk_root, root, sorted(spheres))
    stranded = [joint.find('child').get('link') for joint in root.findall('joint')
                if joint.get('type') != 'fixed'
                and joint.get('name') not in reachable_joints(root, spheres)]
    placeholders = terminal_links(root, stranded)
    for link in placeholders:
        spheres[link] = [{'center': [0.0, 0.0, 0.0], 'radius': PLACEHOLDER_RADIUS}]
    sphere_count = sum(len(v) for v in spheres.values()
                       if v[0]['radius'] > 0)
    if sphere_count > MAX_MODEL_SPHERES:
        raise ValueError(f'{sphere_count} collision spheres exceed the {MAX_MODEL_SPHERES} budget; '
                         f'raise --tolerance (default: {CAPSULE_TOLERANCE})')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Copy every referenced mesh in and store bundle-relative URIs. Absolute paths
    # differ between host and container; a self-contained bundle removes both the
    # duplicate model directories and the driver packages from the GPU container.
    mesh_directory = output / MESH_DIRECTORY
    shutil.rmtree(mesh_directory, ignore_errors=True)
    mesh_directory.mkdir()
    names = {}
    for mesh in root.findall('.//mesh'):
        uri = mesh.get('filename')
        if not uri.startswith(MESH_PREFIX):
            raise ValueError(f'Unexpected mesh URI: {uri}')
        path = (Path(description_dir) / uri[len(MESH_PREFIX):]).resolve()
        if path not in names:
            filename = path.name
            if filename in set(names.values()):  # distinct sources, same basename
                filename = f'{path.stem}_{hashlib.sha256(bytes(path)).hexdigest()[:8]}{path.suffix}'
            names[path] = filename
            shutil.copyfile(path, mesh_directory / filename)
        mesh.set('filename', f'{MESH_DIRECTORY}/{names[path]}')
    shutil.copyfile(semantic, output / 'robot.srdf')
    for config in BUNDLE_CONFIGS:
        shutil.copyfile(Path(moveit_dir) / 'config' / config, output / config)
    urdf_bytes = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    (output / 'robot.urdf').write_bytes(urdf_bytes)
    (output / 'spheres.yaml').write_text(yaml.safe_dump(spheres, sort_keys=False))
    (output / 'initial_positions.yaml').write_text(yaml.safe_dump({'initial_positions': defaults}))
    manifest = {'model': model, 'robot_name': name, 'base_frame': 'base',
                'default_positions': defaults, 'joint_limits': limits,
                'sphere_count': sphere_count, 'capsule_tolerance': tolerance,
                'placeholder_links': placeholders,
                'sdk_model': str(sdk_path), 'sdk_frame_error': frame_error,
                # Pairs the SDK's own coltype/colaffinity mask excludes. Carried
                # in the bundle so prepare_group needs no SDK checkout.
                'sdk_unchecked_pairs': unchecked,
                'canonicalized_joints': canonicalized, 'mesh_count': len(names),
                'bundle_configs': list(BUNDLE_CONFIGS),
                'urdf_sha256': hashlib.sha256(urdf_bytes).hexdigest(),
                # Fingerprint of the driver's inputs, so a bundle baked into an
                # image can be told apart from a stale one after a driver update.
                'source_sha256': hashlib.sha256(
                    source.read_bytes() + semantic.read_bytes()
                    + sdk_path.read_bytes()).hexdigest()}
    (output / 'model.json').write_text(json.dumps(manifest, indent=2) + '\n')
    shutil.rmtree(output / GROUP_DIRECTORY, ignore_errors=True)
    manifest['groups'] = [prepare_group(output, group)['group'] for group in groups]
    return manifest


def worst_sphere_overlap(root, spheres, ignore, positions):
    """Closest approach among the sphere pairs cuMotion will actually test."""
    frames = forward_kinematics(root, positions, None)
    worst, pair = math.inf, None
    for a, b in itertools.combinations(spheres, 2):
        if b in ignore[a] or a in ignore[b]:
            continue
        centres = [np.array([s['center'] for s in spheres[key]]) @ frames[key][:3, :3].T
                   + frames[key][:3, 3] for key in (a, b)]
        radii = [np.array([s['radius'] for s in spheres[key]]) for key in (a, b)]
        distance = float((np.linalg.norm(centres[0][:, None] - centres[1][None], axis=2)
                          - radii[0][:, None] - radii[1][None]).min())
        if distance < worst:
            worst, pair = distance, (a, b)
    return worst, pair


def prepare_group(bundle, group, tool_frame=None):
    """Add one planning group to a bundle. Needs no driver workspace, just the bundle.

    Spheres and the URDF are model-level; only the c-space, tool frame and the
    self-collision ignore set depend on which joints stay active, so a group
    costs kilobytes rather than another copy of the meshes.
    """
    bundle = Path(bundle)
    metadata = json.loads((bundle / 'model.json').read_text())
    root = ET.parse(bundle / 'robot.urdf').getroot()
    srdf = ET.parse(bundle / 'robot.srdf').getroot()
    spheres = yaml.safe_load((bundle / 'spheres.yaml').read_text())
    resolved = srdf_groups(srdf, root)
    active, tips = [], []
    for part in group.split('+'):
        if part not in resolved:
            raise ValueError(f'Unknown SRDF group {part}; available: {sorted(resolved)}')
        active.extend(resolved[part]['joints'])
        tips.extend(resolved[part]['tips'])
    # SRDF groups legitimately name fixed joints too (a_1_2's right_arm lists
    # ee_right_joint); only the movable ones form the c-space.
    movable = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
    active = [j for j in dict.fromkeys(active) if j in movable]
    tips = list(dict.fromkeys(tips))
    if not active:
        raise ValueError(f'Group {group} has no movable joints in this model')
    # cuMotion plans to exactly one end-effector pose (plan_single), so one tip wins.
    tool = tool_frame or (tips[0] if tips else None)
    if tool is None:
        raise ValueError(f'Group {group} declares no chain tip; pass an explicit tool frame')
    # The SDK's own mask comes first: it is what the robot enforces on board, and
    # it is the only source that knows a pair cannot meet mechanically. The SRDF
    # then adds MoveIt's adjacent/never exclusions on top.
    ignore = {key: [] for key in spheres}
    for a, b in metadata.get('sdk_unchecked_pairs', []):
        if a in spheres and b in spheres:
            ignore[a].append(b)
    for pair in srdf.findall('disable_collisions'):
        a, b = pair.get('link1'), pair.get('link2')
        if a in spheres and b in spheres and b not in ignore[a] and a not in ignore[b]:
            ignore[a].append(b)
    # Links with identical active ancestors are rigid relative to one another
    # after XRDF joint locking. Avoid testing intersections within that assembly.
    ancestry = {metadata['base_frame']: ()}
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
    defaults = metadata['default_positions']
    # Unlocking joints exposes link pairs that were previously rigid, so verify
    # here rather than letting cuMotion reject every query at runtime.
    clearance, pair = worst_sphere_overlap(root, spheres, ignore, defaults)
    if clearance < 0:
        raise ValueError(
            f'Group {group} starts in self-collision: {pair[0]} and {pair[1]} overlap by '
            f'{-clearance * 1000:.1f} mm at the default posture. Either the posture is '
            'genuinely unreachable, or this pair belongs in the SDK collision mask.')
    # cuRobo locks every controlled joint, so all of them must be on its chain.
    stranded = sorted(set(defaults) - reachable_joints(root, [tool, *spheres]))
    if stranded:
        raise ValueError(f'Group {group} leaves {stranded} off the kinematic chain cuRobo '
                         'builds; the bundle needs a placeholder sphere on their links')
    xrdf = {
        'format': 'xrdf', 'format_version': 1.0,
        'default_joint_positions': defaults,
        'cspace': {'joint_names': active, 'acceleration_limits': [1.0] * len(active),
                   'jerk_limits': [10.0] * len(active)},
        'tool_frames': [tool],
        'collision': {'geometry': 'robot', 'buffer_distance': 0.0},
        'self_collision': {'geometry': 'robot', 'ignore': ignore, 'buffer_distance': {}},
        'geometry': {'robot': {'spheres': spheres}},
    }
    controllers = controllers_for(
        yaml.safe_load((bundle / 'ros2_controllers.yaml').read_text()), active)
    entry = bundle / GROUP_DIRECTORY / group.replace('+', '_')
    entry.mkdir(parents=True, exist_ok=True)
    xrdf_text = yaml.safe_dump(xrdf, sort_keys=False)
    (entry / 'robot.xrdf').write_text(xrdf_text)
    detail = {'group': entry.name, 'srdf_groups': group.split('+'), 'tool_frame': tool,
              'active_joints': active, 'controllers': controllers,
              'locked_joints': {k: v for k, v in defaults.items() if k not in active},
              'xrdf_sha256': hashlib.sha256(xrdf_text.encode()).hexdigest()}
    (entry / 'group.json').write_text(json.dumps(detail, indent=2) + '\n')
    return detail


def bundle_groups(directory):
    entries = sorted(p.name for p in (Path(directory) / GROUP_DIRECTORY).glob('*/')
                     if (p / 'group.json').is_file())
    if not entries:
        raise ValueError('Bundle declares no planning groups; regenerate with prepare_model')
    return entries


def load_model(directory, group=None):
    directory = Path(directory).resolve()
    metadata = json.loads((directory / 'model.json').read_text())
    if metadata['sphere_count'] > MAX_MODEL_SPHERES:
        raise ValueError('Collision sphere budget exceeded; regenerate with prepare_model')
    if hashlib.sha256((directory / 'robot.urdf').read_bytes()).hexdigest() != metadata['urdf_sha256']:
        raise ValueError('Model artifacts changed; regenerate with prepare_model')
    for required in ('robot.srdf', 'spheres.yaml', *metadata.get('bundle_configs', ())):
        if not (directory / required).is_file():
            raise ValueError(f'Incomplete bundle, missing {required}; regenerate with prepare_model')
    available = bundle_groups(directory)
    if not group:
        if len(available) > 1:
            raise ValueError(f'Bundle provides several groups {available}; select one')
        group = available[0]
    if group not in available:
        raise ValueError(f'Unknown group {group}; bundle provides {available}')
    entry = directory / GROUP_DIRECTORY / group
    detail = json.loads((entry / 'group.json').read_text())
    if hashlib.sha256((entry / 'robot.xrdf').read_bytes()).hexdigest() != detail['xrdf_sha256']:
        raise ValueError('Group artifacts changed; regenerate with prepare_model')
    metadata.update(detail)
    metadata['xrdf_path'] = str(entry / 'robot.xrdf')
    root = ET.parse(directory / 'robot.urdf').getroot()
    # Stored URIs stay bundle-relative so the directory is portable; every
    # consumer downstream wants a real path in this environment.
    for mesh in root.findall('.//mesh'):
        path = directory / mesh.get('filename')
        if not path.is_file():
            raise ValueError(f'Bundle mesh is missing: {mesh.get("filename")}')
        mesh.set('filename', str(path))
    return metadata, root


def write_resolved_urdf(root):
    """cuMotion re-parses robot.urdf from disk, so hand it resolved mesh paths."""
    handle = tempfile.NamedTemporaryFile(mode='wb', prefix='rby1_resolved_',
                                         suffix='.urdf', delete=False)
    with handle:
        handle.write(ET.tostring(root, encoding='utf-8', xml_declaration=True))
    return handle.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, default='m_1_2')
    parser.add_argument('--output', required=True)
    parser.add_argument('--groups', nargs='+', default=list(DEFAULT_GROUPS),
                        help='SRDF group names; join several with "+" (e.g. right_arm+torso)')
    parser.add_argument('--posture', help='YAML containing initial_positions overrides (radians/metres)')
    parser.add_argument('--tolerance', type=float, default=CAPSULE_TOLERANCE,
                        help='Fraction by which capsule spheres may exceed the capsule')
    parser.add_argument('--sdk-dir', help=f'RBY1 SDK models directory (default: ${SDK_ENV} '
                                          f'or {DEFAULT_SDK_DIR})')
    parser.add_argument('--description-dir', help='Override installed rby1_description share directory')
    parser.add_argument('--moveit-dir', help='Override installed rby1_moveit_<model> share directory')
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    result = prepare(args.description_dir or get_package_share_directory('rby1_description'),
                     args.moveit_dir or get_package_share_directory(f'rby1_moveit_{args.model}'),
                     args.output, args.model, args.posture, args.sdk_dir, args.groups,
                     args.tolerance)
    print(f'Prepared {result["model"]}: {result["sphere_count"]} spheres from SDK capsules, '
          f'{result["mesh_count"]} meshes, groups {result["groups"]} in {args.output}')


if __name__ == '__main__':
    main()
