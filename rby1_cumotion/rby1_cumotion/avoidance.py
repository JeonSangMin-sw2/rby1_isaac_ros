"""Obstacles that appear while the arm moves: look ahead, and replan around them.

target_executor sends a whole trajectory to the driver. While it runs, the next
`horizon` seconds of it are checked against the planning scene -- with cuRobo
and the planner's own sphere model, the same one cuMotion plans with. When a
collision lies ahead, cuRobo MotionGen plans again from where the arm will be a
moment later *and how fast it will be moving there*, to the same target; the
result replaces the running trajectory in the driver without a stop
(execution.splice). Obstacles that move are swept ahead along their measured
velocity, so the plan avoids where they are going, not only where they were.

The cuRobo parts need the container; the obstacle bookkeeping above them does not.
"""

import math

import numpy as np

from rby1_cumotion.planning import quaternion

BOX, SPHERE, CYLINDER, MESH = 'box', 'sphere', 'cylinder', 'mesh'
# cuRobo reports these when no collision-free goal or start exists: waiting will
# not help, unlike an optimiser that merely did not converge this time.
HOPELESS = ('IK_FAIL', 'INVALID_START_STATE_WORLD_COLLISION', 'INVALID_START_STATE_SELF_COLLISION',
            'INVALID_START_STATE_JOINT_LIMITS', 'INVALID_QUERY')


def pose_matrix(position, orientation):
    """geometry_msgs position + (x, y, z, w) orientation as a 4x4."""
    x, y, z, w = orientation
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = position
    return matrix


def objects_from_scene(collision_objects):
    """MoveIt CollisionObjects as plain dicts: {name, kind, dims, pose (4x4)}.

    A mesh also carries its `vertices` and `faces`; its pose is the centre of its
    bounding box and `dims` that box, which stands in for it where a distance has to
    be worked out here (moving obstacles). The collision checks use the mesh itself.
    """
    from shape_msgs.msg import SolidPrimitive
    kinds = {SolidPrimitive.BOX: BOX, SolidPrimitive.SPHERE: SPHERE, SolidPrimitive.CYLINDER: CYLINDER}
    found = []
    for obj in collision_objects:
        base = pose_matrix((obj.pose.position.x, obj.pose.position.y, obj.pose.position.z),
                           (obj.pose.orientation.x, obj.pose.orientation.y,
                            obj.pose.orientation.z, obj.pose.orientation.w))
        if not any((obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z,
                    obj.pose.orientation.w)):
            base[:3, :3] = np.eye(3)  # an unset pose means identity
        for index, (primitive, pose) in enumerate(zip(obj.primitives, obj.primitive_poses)):
            kind = kinds.get(primitive.type)
            if kind is None:
                continue
            local = pose_matrix((pose.position.x, pose.position.y, pose.position.z),
                                (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                 pose.orientation.w))
            found.append({'name': f'{obj.id}_{index}', 'id': obj.id, 'kind': kind,
                          'dims': [float(v) for v in primitive.dimensions], 'pose': base @ local})
        for index, (mesh, pose) in enumerate(zip(obj.meshes, obj.mesh_poses)):
            vertices = np.array([[v.x, v.y, v.z] for v in mesh.vertices], dtype=float)
            faces = [[int(i) for i in triangle.vertex_indices] for triangle in mesh.triangles]
            if len(vertices) == 0 or not faces:
                continue
            local = pose_matrix((pose.position.x, pose.position.y, pose.position.z),
                                (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                 pose.orientation.w))
            if not any((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)):
                local[:3, :3] = np.eye(3)
            low, high = vertices.min(axis=0), vertices.max(axis=0)
            centre = np.eye(4)
            centre[:3, 3] = (low + high) / 2
            found.append({'name': f'{obj.id}_mesh{index}', 'id': obj.id, 'kind': MESH,
                          'dims': (high - low).tolist(), 'pose': base @ local @ centre,
                          'vertices': vertices - centre[:3, 3], 'faces': faces})
    return found


class MotionTracker:
    """Velocity of each obstacle from successive scene readings."""

    def __init__(self, still=0.01):
        self.last = {}
        self.velocity = {}
        self.still = still  # m/s below which an obstacle counts as standing

    def update(self, objects, now):
        velocity = {}
        for obj in objects:
            previous = self.last.get(obj['name'])
            if previous is not None and now > previous[1]:
                v = (obj['pose'][:3, 3] - previous[0]) / (now - previous[1])
                old = self.velocity.get(obj['name'])
                velocity[obj['name']] = v if old is None else 0.5 * v + 0.5 * old
        self.last = {obj['name']: (obj['pose'][:3, 3].copy(), now) for obj in objects}
        self.velocity = velocity
        return {name: v for name, v in velocity.items() if np.linalg.norm(v) >= self.still}


def swept(objects, velocities, duration, count=4):
    """Objects plus copies where the moving ones will be over the next `duration` s."""
    result = list(objects)
    for obj in objects:
        v = velocities.get(obj['name'])
        if v is None or duration <= 0:
            continue
        for k in range(1, count + 1):
            copy = dict(obj, name=f'{obj["name"]}_ahead{k}', pose=obj['pose'].copy())
            copy['pose'][:3, 3] = obj['pose'][:3, 3] + v * duration * k / count
            result.append(copy)
    return result


def surface_distance(points, obj):
    """Signed distance from each point to the obstacle's surface (negative inside)."""
    pose = obj['pose']
    local = (np.asarray(points, dtype=float) - pose[:3, 3]) @ pose[:3, :3]
    if obj['kind'] == SPHERE:
        return np.linalg.norm(local, axis=-1) - obj['dims'][0]
    if obj['kind'] in (BOX, MESH):  # a mesh by its bounding box
        excess = np.abs(local) - np.asarray(obj['dims'][:3]) / 2
    else:  # cylinder along its z axis: MoveIt dimensions are height, radius
        height, radius = obj['dims'][0], obj['dims'][1]
        excess = np.stack([np.linalg.norm(local[..., :2], axis=-1) - radius,
                           np.abs(local[..., 2]) - height / 2], axis=-1)
    outside = np.linalg.norm(np.maximum(excess, 0.0), axis=-1)
    inside = np.minimum(excess.max(axis=-1), 0.0)
    return outside + inside


def moving_collision(spheres, times, obstacles, velocities):
    """First waypoint whose spheres meet a moving obstacle where *it* will be then.

    `spheres` is (waypoints, n, 4): x, y, z, radius of the arm's collision
    spheres at each waypoint, `times` how far ahead each waypoint is. Each
    obstacle is moved along its velocity to that time, so an obstacle is only
    compared with the arm at the same moment -- sweeping it over the whole
    window instead would set where it will be against where the arm is now.
    """
    spheres = np.asarray(spheres, dtype=float)
    for index, (row, t) in enumerate(zip(spheres, times)):
        live = row[row[:, 3] > 0]  # placeholders have negative radius
        for obj in obstacles:
            v = velocities.get(obj['name'])
            if v is None:
                continue
            moved = dict(obj, pose=obj['pose'].copy())
            moved['pose'][:3, 3] = obj['pose'][:3, 3] + v * t
            if np.any(surface_distance(live[:, :3], moved) < live[:, 3]):
                return index
    return None


def clear_of(objects, spheres):
    """The objects whose surface stays clear of every one of the arm's `spheres` (n, 4)."""
    live = np.asarray(spheres, dtype=float)
    live = live[live[:, 3] > 0]
    return [obj for obj in objects
            if not np.any(surface_distance(live[:, :3], obj) < live[:, 3])]


def signature(objects):
    return tuple((o['name'], o['kind'], tuple(np.round(o['dims'], 4)),
                  tuple(np.round(o['pose'][:3, :].reshape(-1), 4)),
                  hash(np.round(o['vertices'], 4).tobytes()) if o['kind'] == MESH else 0) for o in objects)


def world_config(objects, ground=False):
    """A cuRobo collision world, built the way cuMotion builds its own."""
    from curobo.geom.types import Cuboid, Cylinder, Mesh, Sphere, WorldConfig
    cuboids, spheres, cylinders, meshes = [], [], [], []
    if ground:
        cuboids.append(Cuboid(name='ground', pose=[0, 0, -0.05, 1, 0, 0, 0], dims=[2.0, 2.0, 0.1]))
    for obj in objects:
        x, y, z, w = quaternion(obj['pose'][:3, :3])
        pose = [*obj['pose'][:3, 3].tolist(), float(w), float(x), float(y), float(z)]
        if obj['kind'] == BOX:
            cuboids.append(Cuboid(name=obj['name'], pose=pose, dims=obj['dims'][:3]))
        elif obj['kind'] == SPHERE:
            spheres.append(Sphere(name=obj['name'], pose=pose, radius=obj['dims'][0]))
        elif obj['kind'] == MESH:
            meshes.append(Mesh(name=obj['name'], pose=pose, vertices=np.asarray(obj['vertices']).tolist(),
                               faces=obj['faces']))
        else:  # MoveIt cylinder dimensions: height, radius
            cylinders.append(Cylinder(name=obj['name'], pose=pose, height=obj['dims'][0],
                                      radius=obj['dims'][1]))
    return WorldConfig(cuboid=cuboids, sphere=spheres, cylinder=cylinders,
                       mesh=meshes).get_collision_check_world()


def from_start(path, q, search=20, tolerance=0.02):
    """`path` from its point nearest `q` on, starting exactly at `q`.

    Planned from a moving start, MotionGen's first points lie a little behind
    the start state -- about the start velocity times 0.07 s -- as if carrying
    on from before it. The motion that matters begins where the path passes q.
    """
    path = np.asarray(path, dtype=float)
    head = path[:search]
    nearest = int(np.argmin(np.abs(head - q).max(axis=1)))
    if np.abs(head[nearest] - q).max() > tolerance:
        raise ValueError(f'replanned path passes {np.abs(head[nearest] - q).max():.3f} rad from '
                         'its start state')
    trimmed = path[nearest:].copy()
    trimmed[0] = q
    return trimmed


class Avoider:
    """cuRobo in this process: batch collision checks and MotionGen replans."""

    def __init__(self, metadata, root, planner, ground=False, cache=20, seeds=6, steps=32, iters=0):
        import torch
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        from isaac_ros_cumotion.update_kinematics import get_robot_config
        from rby1_cumotion.model import write_resolved_urdf
        self.torch = torch
        self.tensor = TensorDeviceType()
        self.ground = ground
        robot = get_robot_config(robot_file=metadata['xrdf_path'],
                                 urdf_file_path=write_resolved_urdf(root), logger=None)['robot_cfg']
        empty = world_config([], ground)
        caches = {'obb': cache, 'mesh': cache}
        self.world = RobotWorld(RobotWorldConfig.load_from_config(
            robot, empty, self.tensor, n_meshes=cache, n_cuboids=cache,
            collision_activation_distance=0.0, collision_checker_type=CollisionCheckerType.MESH))
        self.motion_gen = MotionGen(MotionGenConfig.load_from_robot_config(
            robot, empty, self.tensor, collision_checker_type=CollisionCheckerType.MESH,
            collision_cache=caches, interpolation_dt=0.025, ee_link_name=metadata['tool_frame'],
            num_trajopt_seeds=seeds, trajopt_tsteps=steps, grad_trajopt_iters=iters or None,
            finetune_trajopt_iters=iters or None, num_graph_seeds=planner['num_graph_seeds']))
        self.motion_gen.warmup(enable_graph=True)
        self.names = list(self.motion_gen.kinematics.joint_names)
        self.checked = self.planned = None

    def attach(self, spheres, link):
        """Spheres [x, y, z, r] on `link` (the attached-object frame) for checks and replans; [] clears."""
        for kinematics in (self.world.kinematics, self.motion_gen.kinematics):
            kinematics.kinematics_config.detach_object(link_name=link)
            if spheres:
                kinematics.kinematics_config.update_link_spheres(
                    link_name=link, start_sph_idx=0,
                    sphere_position_radius=self.tensor.to_device(self.torch.tensor(spheres, dtype=self.torch.float32)))

    def update(self, check_objects, plan_objects):
        """Obstacles for the look-ahead check and for replanning.

        Moving obstacles are swept differently for the two: the check covers the
        next `horizon` seconds of the arm, so sweeping further would set where an
        obstacle will be in a second against where the arm is in a moment; a
        replan covers seconds of motion and must stay clear of where they go.
        """
        key = signature(check_objects)
        if key != self.checked:
            self.world.update_world(world_config(check_objects, self.ground))
            self.checked = key
        self.plan_world(plan_objects)

    def plan_world(self, objects):
        key = signature(objects)
        if key != self.planned:
            self.motion_gen.update_world(world_config(objects, self.ground))
            self.planned = key

    def first_hit(self, rows, names, times=None, moving=(), velocities=None):
        """(first row in collision or None, whether a moving obstacle is what it meets).

        Rows are joint positions in `names` order. The cuRobo world holds the standing
        obstacles; `moving` ones are checked at each row's `times` (seconds ahead)
        along their `velocities`.
        """
        if len(rows) == 0:
            return None, False
        order = [names.index(name) for name in self.names]
        # Column fancy-indexing hands back a Fortran-ordered array; cuRobo's kernels
        # assert on anything not C-contiguous.
        rows = np.ascontiguousarray(np.asarray(rows, dtype=np.float32)[:, order])
        q = self.tensor.to_device(self.torch.from_numpy(rows)).contiguous()
        d_world, d_self = self.world.get_world_self_collision_distance_from_joints(q)
        hits = (d_world.view(-1) > 0).cpu().numpy() | (d_self.view(-1) > 0).cpu().numpy()
        first = int(np.nonzero(hits)[0][0]) if hits.any() else None
        if moving:
            spheres = self.world.get_kinematics(q).link_spheres_tensor.view(len(rows), -1, 4).cpu().numpy()
            index = moving_collision(spheres, times, moving, velocities)
            if index is not None and (first is None or index < first):
                return index, True
        return first, False

    def spheres(self, q, names):
        """The arm's collision spheres (n, 4) at joint positions `q` in `names` order."""
        order = [names.index(name) for name in self.names]
        row = np.ascontiguousarray(np.asarray(q, dtype=np.float32)[order][None])
        state = self.world.get_kinematics(self.tensor.to_device(self.torch.from_numpy(row)).contiguous())
        return state.link_spheres_tensor.view(-1, 4).cpu().numpy()

    def replan(self, q, v, target, names, attempts=2):
        """Joint path from (q, v) to the 4x4 tool target, in `names` order; or (None, status)."""
        from curobo.types.math import Pose
        from curobo.types.state import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        order = [names.index(name) for name in self.names]

        def to(values):
            row = np.ascontiguousarray(np.asarray(values, dtype=np.float32)[order][None])
            return self.tensor.to_device(self.torch.from_numpy(row)).contiguous()
        start = JointState.from_position(to(q), joint_names=self.names)
        start.velocity[:] = to(v)
        x, y, z, w = quaternion(np.asarray(target)[:3, :3])
        tensor = self.torch.tensor
        goal = Pose(position=self.tensor.to_device(tensor([np.asarray(target)[:3, 3]], dtype=self.torch.float32)),
                    quaternion=self.tensor.to_device(tensor([[w, x, y, z]], dtype=self.torch.float32)))
        result = self.motion_gen.plan_single(start, goal, MotionGenPlanConfig(
            max_attempts=attempts, enable_graph=True))
        if not bool(result.success.item()):
            return None, str(getattr(result.status, 'name', result.status))
        plan = result.get_interpolated_plan().position.cpu().numpy().reshape(-1, len(self.names))
        back = [self.names.index(name) for name in names]
        return from_start(plan[:, back], q), None
