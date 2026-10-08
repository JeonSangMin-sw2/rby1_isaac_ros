"""Tracking mode: follow a moving 4x4 target, one step at a time.

In tracking mode (the executor's set_tracking service) the targets on the target
topic stop being destinations to plan a whole path to; each one only moves a goal.
The targets are smoothed and a moving one is aimed ahead of (TargetMotion), since
the robot follows its commands about 0.1 s late. At a fixed rate (tracking.rate) a
follower takes one step toward the goal from the state it last commanded, and that
step goes to the driver's stream_joint -- so camera frames arriving late or missing
do not reach the robot as jerks. Two followers (tracking.method):

  ik   Servo: cuRobo IK for the goal every step, the joints stepped toward the
       solution within a speed and an acceleration and riding along with a moving
       goal. No look-ahead: it stops short of an obstacle instead of going around.
  mpc  Tracker: cuRobo MPC. It looks 30 steps ahead, so it goes around obstacles
       in the planning scene, and it trails a moving goal by 0.2-0.3 s more.

Measured on the simulated robot (m_1_2 right_arm, RTX 5070, 50 Hz), the hand off a
marker's place on a 6 cm circle, mean: ik 1.4 mm at 6 cm/s and 7.5 mm at 13 cm/s
(3.5 and 9 mm with 1 mm of noise on the marker), a 10 cm jump within 1 cm in 0.4 s;
mpc 5 and 28 mm, 1.8 s. One step: ik 10 ms, mpc 6.5 ms.
"""

import numpy as np

from rby1_cumotion.executor.avoidance import pose_matrix, quaternion, signature, world_config


# A target this far from where the last ones said it would be has jumped, not moved.
JUMP = 0.03
# Below this speed (m/s) a target counts as at rest and is not aimed ahead of -- what is
# left then is camera noise, and aiming ahead would only multiply it; from one and a half
# times this speed it is aimed ahead of in full. Noisier samples raise the speed (NOISE_SPEEDS).
REST_SPEED = 0.02
# How many times the speed the samples' noise alone looks like a target must move to count
# as moving: with 1 mm of noise from 2.4 cm/s, with 3 mm from 7 cm/s. Lower follows a slow
# target under a noisy camera sooner and lets more of the noise through at rest.
NOISE_SPEEDS = 2.0
# A target at rest is where its samples have been on average over about this long (s).
REST_AVERAGE = 0.15
# Samples after a fresh start before a target can count as moving (its speed is a guess until then).
SETTLE_SAMPLES = 5


class TargetMotion:
    """Where a target is, how fast it moves and how it speeds up or turns, from noisy samples.

    A g-h-k filter on the position: each sample moves the estimate by a share of how
    far it is from where the last ones predicted it, the velocity by a smaller share
    and the acceleration by a smaller one still. `smoothing` (0..0.95) sets the shares:
    0 takes every sample as it is (and its noise, which aiming ahead multiplies);
    higher rides the noise out and is slower to notice a change of course. A target
    that jumps, or comes after a pause, starts afresh at rest. The orientation is the
    last sample's.

    A target at rest is not aimed ahead of, and is taken as the average of its last
    samples: only a speed that the samples' noise could not fake counts as moving (the
    noise is measured from how far the samples miss their prediction). So a noisier
    camera makes the arm calmer at rest and slower to start after a marker, not shakier.

    Aiming 0.17 s ahead of a target on a 6 cm circle with 1 mm of noise per sample at
    30 Hz misses by (mm, mean) 3 at 6 cm/s and 8 at 13 cm/s with smoothing 0.75; taking
    the velocity only (no acceleration) 4 and 11; every sample as it is, 14 and 14.
    """

    def __init__(self, smoothing=0.75, reset_after=0.5, jump=JUMP):
        # Shares of the miss taken by the position (g), the velocity (h) and the acceleration (k).
        self.gains = (1.0 - smoothing ** 3, 1.5 * (1.0 - smoothing ** 2) * (1.0 - smoothing),
                      0.5 * (1.0 - smoothing) ** 3)
        self.reset_after, self.jump = reset_after, jump
        self.time = self.position = self.target = self.resting = None
        self.velocity, self.acceleration = np.zeros(3), np.zeros(3)
        self.samples = 0     # since it started afresh
        self.noise = 0.0     # mean square of how far the samples miss their prediction (m^2): the camera's
        self.period = 1 / 30.0

    def update(self, target, now):
        """A new sample (4x4) that arrived at `now` (s)."""
        self.target = np.array(target, dtype=float)
        sample = self.target[:3, 3]
        elapsed = None if self.time is None else now - self.time
        if elapsed is not None and elapsed <= 1e-3:
            return  # two samples at once (delivered in a burst): the second says nothing new about the motion
        if elapsed is not None and elapsed < self.reset_after:
            velocity = self.velocity + self.acceleration * elapsed
            predicted = self.position + self.velocity * elapsed + 0.5 * self.acceleration * elapsed * elapsed
            miss = sample - predicted
        if elapsed is None or elapsed >= self.reset_after or np.linalg.norm(miss) > self.jump:
            self.position, self.velocity, self.acceleration = sample.copy(), np.zeros(3), np.zeros(3)
            self.resting = sample.copy()
            self.samples = 1
        else:
            g, h, k = self.gains
            self.position = predicted + g * miss
            self.velocity = velocity + h * miss / elapsed
            self.acceleration = self.acceleration + 2.0 * k * miss / (elapsed * elapsed)
            self.samples += 1
            self.noise += 0.1 * (float(miss @ miss) - self.noise)
            self.period += 0.1 * (elapsed - self.period)
        if elapsed is not None and elapsed > 0.0:
            self.resting = self.resting + (1.0 - np.exp(-elapsed / REST_AVERAGE)) * (self.position - self.resting)
        self.time = now

    def rest_speed(self):
        """The speed (m/s) below which the target counts as at rest: more for noisier samples."""
        # Each sample moves the velocity by h * miss / period: what noise alone makes of it.
        return max(REST_SPEED, NOISE_SPEEDS * self.gains[1] * np.sqrt(self.noise) / self.period)

    def _moving(self):
        """0 for a target at rest .. 1 for one that moves."""
        if self.samples < SETTLE_SAMPLES:
            return 0.0
        return min(max(2.0 * (float(np.linalg.norm(self.velocity)) / self.rest_speed() - 1.0), 0.0), 1.0)

    def _span(self, now, lead, carry):
        return lead + min(max(now - self.time, 0.0), carry)

    def heading(self, now=None, lead=0.0, carry=0.0):
        """The velocity (m/s) the target has `lead` s after `now` (without `now`: has now); none at rest."""
        span = 0.0 if now is None else self._span(now, lead, carry)
        return self._moving() * (self.velocity + self.acceleration * span)

    def at(self, now=None, lead=0.0, limit=0.0, carry=0.0):
        """The target (4x4) `lead` s after `now`, at most `limit` m ahead of where it was last seen.

        A sample older than `carry` s is not carried further along its course. Without
        `now`: where it was last seen, for a target that stopped coming.
        """
        aimed = self.target.copy()
        # At rest: the average of the last samples, so camera noise does not reach the arm.
        # Moving: the filtered position, and ahead of it.
        moving = self._moving()
        aimed[:3, 3] = self.resting + moving * (self.position - self.resting)
        if now is not None:
            span = self._span(now, lead, carry)
            shift = moving * (self.velocity * span + 0.5 * self.acceleration * span * span)
            norm = float(np.linalg.norm(shift))
            if norm > limit > 0.0:
                shift = shift * (limit / norm)
            aimed[:3, 3] += shift
        return aimed


# How hard the servo closes in on its goal near it: closing speed = SERVO_GAIN x distance (1/s).
# Low enough not to chase what little the IK solutions shake, high enough to settle in 0.3 s.
SERVO_GAIN = 15.0


def servo_step(position, velocity, goal, dt, max_speed, max_acceleration, goal_velocity=None, gain=SERVO_GAIN):
    """One step of every joint toward `goal`: (position, velocity) after `dt`.

    Far from the goal a joint closes in as fast as it can still stop on it from
    (sqrt(2 a distance)); near it, in proportion to the distance (`gain`), so it
    settles instead of chasing every small change. Speed and acceleration stay
    within their limits: a jump of the goal is taken as a ramp up and a ramp down.
    With `goal_velocity` (how fast the goal itself moves) the joint rides along with
    a moving goal instead of trailing it by v / gain.
    """
    position, velocity = np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)
    error = np.asarray(goal, dtype=float) - position
    distance = np.abs(error)
    # The two meet at distance a / gain^2 with the same slope, where braking along the
    # proportional law needs exactly a.
    near = max_acceleration / (gain * gain)
    far = np.sqrt(2.0 * max_acceleration * np.maximum(distance - near / 2.0, 0.0))
    wanted = np.sign(error) * np.where(distance <= near, gain * distance, far)
    if goal_velocity is not None:
        wanted = wanted + np.asarray(goal_velocity, dtype=float)
    wanted = np.clip(wanted, -max_speed, max_speed)
    velocity = velocity + np.clip(wanted - velocity, -max_acceleration * dt, max_acceleration * dt)
    return position + velocity * dt, velocity


def joint_rates(jacobian, velocity, damping=0.05):
    """Joint speeds that move the tool at `velocity` (m/s, base) without turning it.

    `jacobian`: 6 x n (tool position, then rotation). Damped least squares: near a
    singular arm shape the speeds stay small instead of blowing up.
    """
    twist = np.concatenate([np.asarray(velocity, dtype=float), np.zeros(3)])
    square = jacobian @ jacobian.T + damping * damping * np.eye(6)
    return jacobian.T @ np.linalg.solve(square, twist)


class Tracker:
    """cuRobo MPC toward a pose goal that may change every step."""

    note = None  # what stops it from following, for the status topic (the MPC always steps)
    lag = 0.3    # s it trails a moving goal by itself: targets are aimed that much further ahead

    def __init__(self, metadata, root, rate, iterations=1, cache=20, ground=False):
        import torch
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
        from isaac_ros_cumotion.update_kinematics import get_robot_config
        from rby1_cumotion.model import write_resolved_urdf
        self.torch = torch
        self.tensor = TensorDeviceType()
        self.ground = ground
        robot = get_robot_config(robot_file=metadata['xrdf_path'],
                                 urdf_file_path=write_resolved_urdf(root), logger=None)['robot_cfg']
        config = MpcSolverConfig.load_from_robot_config(
            robot, world_config([], ground), tensor_args=self.tensor, store_rollouts=False,
            step_dt=1.0 / rate, use_cuda_graph=True, use_cuda_graph_metrics=True, self_collision_check=True,
            collision_checker_type=CollisionCheckerType.MESH, collision_cache={'obb': cache, 'mesh': cache},
            particle_opt_iters=iterations)
        self.mpc = MpcSolver(config)
        self.names = list(self.mpc.rollout_fn.joint_names)
        self.goal_buffer = None
        self.state = None
        self.planned = None

    def _joint_state(self, positions, velocities=None):
        from curobo.types.state import JointState
        q = self.tensor.to_device(self.torch.tensor([positions], dtype=self.torch.float32))
        state = JointState.from_position(q, joint_names=self.names)
        if velocities is not None:
            state.velocity[:] = self.tensor.to_device(self.torch.tensor([velocities], dtype=self.torch.float32))
        return state

    def _row(self, values):
        return self.tensor.to_device(self.torch.tensor([list(values)], dtype=self.torch.float32))

    def pose_of(self, positions):
        """The tool pose (4x4, base frame) at joint `positions` (in `self.names` order)."""
        kin = self.mpc.rollout_fn.compute_kinematics(self._joint_state(positions))
        w, x, y, z = kin.ee_quat_seq[0].cpu().numpy().astype(float)
        return pose_matrix(kin.ee_pos_seq[0].cpu().numpy().astype(float), (x, y, z, w))

    def start(self, positions):
        """Begin from measured joint `positions` (self.names order), holding the pose there."""
        from curobo.rollout.rollout_base import Goal
        from curobo.types.math import Pose
        self.state = self._joint_state(positions)
        kin = self.mpc.rollout_fn.compute_kinematics(self.state)
        goal = Goal(current_state=self.state, goal_state=self._joint_state(positions),
                    goal_pose=Pose(position=kin.ee_pos_seq.clone(), quaternion=kin.ee_quat_seq.clone()))
        self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
        self.mpc.update_goal(self.goal_buffer)

    def aim(self, target, velocity=None):
        """A new goal: the tool at the 4x4 `target` (base frame). `velocity` is not used: the MPC trails."""
        from curobo.types.math import Pose
        target = np.asarray(target, dtype=float)
        x, y, z, w = quaternion(target[:3, :3])
        pose = Pose(position=self._row(target[:3, 3]), quaternion=self._row([w, x, y, z]))
        self.goal_buffer.goal_pose.copy_(pose)
        self.mpc.update_goal(self.goal_buffer)

    def step(self):
        """One MPC step from the state it last commanded: (positions, velocities) to command."""
        result = self.mpc.step(self.state, max_attempts=2)
        self.state.copy_(result.action)
        return (self.state.position[0].cpu().numpy().astype(float),
                self.state.velocity[0].cpu().numpy().astype(float))

    def resync(self, positions):
        """The robot is not where the MPC thinks: carry on from where it is, at rest."""
        self.state.copy_(self._joint_state(positions))
        self.mpc.reset()

    def world(self, objects):
        key = signature(objects)
        if key != self.planned:
            self.mpc.update_world(world_config(objects, self.ground))
            self.planned = key

    def attach(self, spheres, link):
        kinematics = self.mpc.rollout_fn.kinematics
        kinematics.kinematics_config.detach_object(link_name=link)
        if spheres:
            kinematics.kinematics_config.update_link_spheres(
                link_name=link, start_sph_idx=0,
                sphere_position_radius=self.tensor.to_device(self.torch.tensor(spheres, dtype=self.torch.float32)))

    def warm_up(self, positions, steps=3):
        """The first step captures CUDA graphs (seconds): take it before the robot relies on it."""
        self.start(positions)
        for _ in range(steps):
            self.step()
        self.start(positions)


class Servo:
    """cuRobo IK for the goal every step; the joints step toward the solution (servo_step).

    The same interface as Tracker. Each IK starts from the last solution and is held
    near it, so the solution is the nearby one; it is free of collisions with the
    scene and the robot itself, and so is every step on the way (checked) -- a step
    that would collide is not taken: the arm holds short of the obstacle. The joints
    follow the solution within a speed and an acceleration, and ride along with a
    moving goal: the joint speeds that move the tool as fast as the goal moves (from
    the Jacobian) are fed forward.
    """

    lag = 0.0  # it rides along with the goal

    def __init__(self, metadata, root, rate, max_speed, max_acceleration, cache=20, ground=False, seeds=4):
        import torch
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
        from isaac_ros_cumotion.update_kinematics import get_robot_config
        from rby1_cumotion.model import write_resolved_urdf
        self.torch = torch
        self.tensor = TensorDeviceType()
        self.ground = ground
        self.dt, self.max_speed, self.max_acceleration = 1.0 / rate, max_speed, max_acceleration
        self.seeds = seeds
        robot = get_robot_config(robot_file=metadata['xrdf_path'],
                                 urdf_file_path=write_resolved_urdf(root), logger=None)['robot_cfg']
        config = IKSolverConfig.load_from_robot_config(
            robot, world_config([], ground), tensor_args=self.tensor, num_seeds=seeds,
            position_threshold=0.002, rotation_threshold=0.02, self_collision_check=True, self_collision_opt=True,
            collision_checker_type=CollisionCheckerType.MESH, collision_cache={'obb': cache, 'mesh': cache},
            use_cuda_graph=True)
        self.ik = IKSolver(config)
        self.names = list(self.ik.kinematics.joint_names)
        self.position = self.velocity = self.solution = self.goal = None
        self.goal_velocity = np.zeros(3)
        self.planned = None
        self.note = None

    def _row(self, values):
        return self.tensor.to_device(self.torch.tensor([list(values)], dtype=self.torch.float32))

    def _rest(self, positions):
        self.position = np.array(positions, dtype=float)
        self.velocity = np.zeros(len(self.names))
        self.solution = self.position.copy()

    def pose_of(self, positions):
        """The tool pose (4x4, base frame) at joint `positions` (in `self.names` order)."""
        state = self.ik.fk(self._row(positions))
        w, x, y, z = state.ee_quaternion[0].cpu().numpy().astype(float)
        return pose_matrix(state.ee_position[0].cpu().numpy().astype(float), (x, y, z, w))

    def start(self, positions):
        """Begin from measured joint `positions` (self.names order), holding the pose there."""
        self._rest(positions)
        self.goal = None
        self.note = None

    def aim(self, target, velocity=None):
        """A new goal: the tool at the 4x4 `target` (base frame), which moves at `velocity` (m/s)."""
        self.goal = np.array(target, dtype=float)
        self.goal_velocity = np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)

    def jacobian(self, positions, step=1e-4):
        """6 x n: how the tool's position and rotation (base) change with each joint, by differences."""
        count = len(self.names)
        batch = np.tile(np.asarray(positions, dtype=float), (count + 1, 1))
        batch[1:] += step * np.eye(count)
        state = self.ik.fk(self.tensor.to_device(self.torch.tensor(batch, dtype=self.torch.float32)))
        position = state.ee_position.cpu().numpy().astype(float)
        w, x, y, z = state.ee_quaternion.cpu().numpy().astype(float).T
        # Rotation from the first pose to each other one, as a small rotation vector:
        # 2 x the vector part of q_i * conj(q_0).
        turned = 2.0 * np.stack([
            -w[1:] * x[0] + x[1:] * w[0] - y[1:] * z[0] + z[1:] * y[0],
            -w[1:] * y[0] + x[1:] * z[0] + y[1:] * w[0] - z[1:] * x[0],
            -w[1:] * z[0] - x[1:] * y[0] + y[1:] * x[0] + z[1:] * w[0]], axis=1)
        return np.concatenate([(position[1:] - position[0]).T, turned.T]) / step

    def step(self):
        """One step toward the goal's joint solution: (positions, velocities) to command."""
        from curobo.types.math import Pose
        note = None
        if self.goal is not None:
            x, y, z, w = quaternion(self.goal[:3, :3])
            # Each solve starts from the last solution and is held near it, so the solution is
            # the nearby one and moves only as the target does.
            seed = self._row(self.solution)
            # New tensors every time: the solver keeps the first goal's as its buffer.
            result = self.ik.solve_single(
                Pose(position=self._row(self.goal[:3, 3]), quaternion=self._row([w, x, y, z])),
                retract_config=seed, seed_config=seed[None], return_seeds=self.seeds, num_seeds=self.seeds)
            found = result.success.view(-1).cpu().numpy().astype(bool)
            if found.any():
                # Of the seeds that reached the target (one is the last solution, the rest
                # random), the solution nearest the last one: no swing to another arm shape.
                solutions = result.solution.view(len(found), -1).cpu().numpy().astype(float)[found]
                self.solution = solutions[np.abs(solutions - self.solution).max(axis=1).argmin()]
            else:
                self.solution = self.position.copy()
                note = 'the target has no collision-free joint solution near the arm: holding'
        rates = None
        if note is None and self.goal_velocity.any():
            rates = joint_rates(self.jacobian(self.solution), self.goal_velocity)
        position, velocity = servo_step(self.position, self.velocity, self.solution, self.dt, self.max_speed,
                                        self.max_acceleration, rates)
        if not bool(self.ik.check_valid(self._row(position)).view(-1)[0]):
            position, velocity = self.position, np.zeros(len(self.names))
            note = 'the next step would collide: holding short of it'
        self.position, self.velocity, self.note = position, velocity, note
        return position.copy(), velocity.copy()

    def resync(self, positions):
        """The robot is not where the commands went: carry on from where it is, at rest."""
        self._rest(positions)

    def world(self, objects):
        key = signature(objects)
        if key != self.planned:
            self.ik.update_world(world_config(objects, self.ground))
            self.planned = key

    def attach(self, spheres, link):
        config = self.ik.kinematics.kinematics_config
        config.detach_object(link_name=link)
        if spheres:
            config.update_link_spheres(
                link_name=link, start_sph_idx=0,
                sphere_position_radius=self.tensor.to_device(self.torch.tensor(spheres, dtype=self.torch.float32)))

    def warm_up(self, positions, steps=3):
        """The first solve captures CUDA graphs (seconds): take it before the robot relies on it."""
        self.start(positions)
        self.aim(self.pose_of(positions))
        for _ in range(steps):
            self.step()
        self.start(positions)
