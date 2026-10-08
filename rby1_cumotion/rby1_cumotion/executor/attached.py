"""Modules attached to the robot, as cuMotion spheres on the hand.

rby1_moveit_objects (or anything else) attaches modules to robot links in MoveIt's
planning scene. MoveIt (OMPL) plans with them; cuMotion's MoveIt plugin reads only
world objects. So the target executor mirrors them: every attached object that
rides rigidly on the planning group's tool frame becomes spheres on the
`attached_object` frame (model.ATTACHED_LINK, fixed to the tool), handed to the
cuMotion planner through its UpdateLinkSpheres action and to the executor's own
cuRobo (avoidance.Avoider).

The spheres *cover* each shape -- every point of it lies in some sphere -- so the
arm keeps clear of the whole module, at the price of a little extra margin.
Shapes: box, cylinder and sphere exactly; a mesh by its bounding box.
"""

import fnmatch
import math

import numpy as np

from rby1_cumotion.model import forward_kinematics, HEAD_JOINTS

# cuMotion reserves this many sphere slots on the attached-object frame.
MAX_SPHERES = 100
# Fewest spheres a shape gets when the budget is shared out.
PER_OBJECT_MIN = 4

BOX, SPHERE, CYLINDER, CONE = 1, 2, 3, 4  # shape_msgs/SolidPrimitive


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(pose):
    matrix = np.eye(4)
    q = pose.orientation
    matrix[:3, :3] = quaternion_matrix(q.x, q.y, q.z, q.w) if (q.x, q.y, q.z, q.w) != (0, 0, 0, 0) else np.eye(3)
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def cover_box(size, count):
    """Centres and radii of about `count` spheres covering a box of `size`, centred at 0."""
    size = np.maximum(np.asarray(size, dtype=float), 1e-6)
    # Split the longest cell side while the budget lasts: small cells, small spheres,
    # and so little of them sticking out past the faces.
    cells = np.ones(3, dtype=int)
    while True:
        axis = int(np.argmax(size / cells))
        if np.prod(cells) // cells[axis] * (cells[axis] + 1) > count:
            break
        cells[axis] += 1
    step = size / cells
    radius = 0.5 * float(np.linalg.norm(step))
    axes = [(np.arange(n) + 0.5) * s - d / 2 for n, s, d in zip(cells, step, size)]
    centres = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3)
    return centres, np.full(len(centres), radius)


def cover_cylinder(radius, height, count):
    """Spheres along the axis (z) covering a cylinder centred at 0."""
    n = max(1, min(count, math.ceil(height / (2 * radius) - 1e-9)))
    slab = height / n
    centres = np.zeros((n, 3))
    centres[:, 2] = (np.arange(n) + 0.5) * slab - height / 2
    return centres, np.full(n, math.hypot(radius, slab / 2))


def shape_spheres(shape, count):
    """(centres, radii) covering one primitive or mesh in its own frame."""
    if hasattr(shape, 'vertices'):  # shape_msgs/Mesh: its bounding box
        vertices = np.array([[v.x, v.y, v.z] for v in shape.vertices])
        if len(vertices) == 0:
            return np.zeros((0, 3)), np.zeros(0)
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        centres, radii = cover_box(high - low, count)
        return centres + (low + high) / 2, radii
    d = list(shape.dimensions)
    if shape.type == BOX:
        return cover_box(d[:3], count)
    if shape.type == SPHERE:
        return np.zeros((1, 3)), np.array([d[0]])
    if shape.type == CYLINDER:  # dimensions: height, radius
        return cover_cylinder(d[1], d[0], count)
    if shape.type == CONE:  # its bounding cylinder
        return cover_cylinder(d[1], d[0], count)
    raise ValueError(f'unsupported shape type {shape.type}')


def rigid_with(root, link, tool):
    """Does `link` stay put relative to `tool` whatever the joints do?"""
    parent = {j.find('child').get('link'): j for j in root.findall('joint')}

    def chain(name):
        joints = []
        while name in parent:
            joints.append(parent[name])
            name = parent[name].find('parent').get('link')
        return joints
    a, b = chain(link), chain(tool)
    shared = {id(j) for j in a} & {id(j) for j in b}
    return all(j.get('type') == 'fixed' for j in a + b if id(j) not in shared)


def turns_with_head(root, link):
    parent = {j.find('child').get('link'): j for j in root.findall('joint')}
    while link in parent:
        if parent[link].get('name') in HEAD_JOINTS:
            return True
        link = parent[link].find('parent').get('link')
    return False


def is_free(name, free):
    """Is attached object `name` one of the `free` ones (names, * and ? allowed)?"""
    return any(pattern and fnmatch.fnmatchcase(name, pattern) for pattern in free)


def free_names(objects, root, free):
    """What `free` names in MoveIt's scene: attached objects and robot links (sorted)."""
    names = [a.object.id for a in objects] + [link.get('name') for link in root.findall('link')]
    return sorted({name for name in names if is_free(name, free)})


def allow_touching(matrix, names):
    """`matrix` (moveit_msgs/AllowedCollisionMatrix) with `names` allowed to touch anything."""
    import copy
    allowed = copy.deepcopy(matrix)
    for name in names:
        if name in allowed.default_entry_names:
            allowed.default_entry_values[allowed.default_entry_names.index(name)] = True
        else:
            allowed.default_entry_names.append(name)
            allowed.default_entry_values.append(True)
    return allowed


def attached_spheres(objects, metadata, root, budget=MAX_SPHERES, free=()):
    """Spheres [x, y, z, r] on the attached-object frame, and a note per object.

    `objects`: moveit_msgs/AttachedCollisionObject. Objects on links that do not
    ride with the tool are left out, with the reason in their note. So are the
    `free` ones (names, * allowed): parts that are allowed to touch things -- the
    fingers that close on what they pick up.
    """
    tool = metadata['tool_frame']
    frames = forward_kinematics(root, metadata['default_positions'], None)
    riding, notes = [], {}
    for attached in objects:
        name, link = attached.object.id, attached.link_name
        frame = attached.object.header.frame_id or link
        if is_free(name, free):
            notes[name] = f'on {link}: left out (free_objects) -- cuMotion lets it touch anything'
        elif link not in frames or frame not in (link, ''):
            notes[name] = f'on {link} in frame {frame}: not read'
        elif rigid_with(root, link, tool):
            riding.append(attached)
        elif turns_with_head(root, link):
            notes[name] = (f'on {link}: the head envelope stands in for it (it covers modules within '
                           '2 cm of the head)')
        else:
            notes[name] = (f'on {link}, which does not move with {tool}: cuMotion does not see it (the other '
                           'arm keeps its own body model, wrist and hand in one)')
    if not riding:
        return [], notes
    shapes = sum(len(a.object.primitives) + len(a.object.meshes) for a in riding)
    each = max(PER_OBJECT_MIN, budget // max(shapes, 1))
    spheres = []
    for attached in riding:
        obj = attached.object
        link_in_tool = np.linalg.inv(frames[tool]) @ frames[attached.link_name]
        body = link_in_tool @ pose_matrix(obj.pose)
        count = 0
        for shape, pose in [*zip(obj.primitives, obj.primitive_poses), *zip(obj.meshes, obj.mesh_poses)]:
            centres, radii = shape_spheres(shape, each)
            at = body @ pose_matrix(pose)
            centres = centres @ at[:3, :3].T + at[:3, 3]
            spheres.extend([*c, r] for c, r in zip(centres, radii))
            count += len(radii)
        notes[obj.id] = f'on {attached.link_name}: {count} spheres'
    if len(spheres) > budget:
        raise ValueError(f'attached modules need {len(spheres)} spheres, cuMotion holds {budget}: '
                         'use fewer or simpler shapes')
    # Rounded for the message; radii up, by more than the centres can move, so they still cover.
    return [[*(round(float(v), 5) for v in sphere[:3]), math.ceil(float(sphere[3]) * 1e5) / 1e5 + 1e-5]
            for sphere in spheres], notes


def flatten(spheres):
    return [float(v) for sphere in spheres for v in sphere]
