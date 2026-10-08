"""Get the robot and the planner model ready, before any planner or MoveIt node starts.

The first stage of cumotion.launch.py. It runs once and exits; the launch starts
the planning stack and the target executor only if it succeeded, because
move_group and the cuMotion planner load the robot model when they start.

With the RB-Y1 driver (hardware driver), in order:
  1. robot kind and version from the driver -> which bundle
  2. emergency stop and control manager faults
  3. power and servos on (robot.enable_robot)
  4. a straight planning arm is bent to the ready pose (robot.ready_if_straight)
  5. the posture is measured, and the runtime bundle is made with the joints
     outside the planning group locked there

Doing 4 before 5 means every arm is where it will stay before anything is
locked, so moving an arm here never leaves the planner with a stale posture.

With hardware mock there is no robot: the bundle comes from `model` ($RBY1_MODEL,
default m_1_2) at its stored posture.

Prints PREPARED ... and exits 0, or PREPARE_FAILED: <reason> and exits 1.
"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import RobotState
from rby1_cumotion import settings
from rby1_cumotion.driver import (bundle_name, Driver, READY, read_robot, ready_goal,
                                  robot_problem, straight_arms)
from rby1_cumotion.model import activate, load_model
from rby1_cumotion.planning import running_nodes


def check_alone(proc='/proc'):
    """Refuse to start next to another cuMotion launch in this container."""
    found = [line for lines in running_nodes(('move_group', 'cumotion_planner_node'), proc).values()
             for line in lines]
    if found:
        raise RuntimeError(f'Another cuMotion launch is running in this container ({found[0]}). '
                           'Stop it first (Ctrl+C in its terminal): two do not fit in GPU memory '
                           "and would answer each other's planning requests")


def choose_source(kind, version, model, model_directory, bundles):
    """The bundle for the connected robot. A named one must agree with the driver."""
    named = model_directory or (str(Path(bundles) / model) if model else '')
    if named:
        source = Path(named)
        bundle_model = json.loads((source / 'model.json').read_text())['model']
        if bundle_model.split('_')[0] != kind.strip().lower():
            raise ValueError(f'Driver runs an RB-Y1 {kind!r} but {source} is {bundle_model}; '
                             'leave model out (launch argument and robot.model) to use the '
                             "connected robot's own")
        if version > 0 and bundle_name(kind, version) != bundle_model:
            raise ValueError(f'Driver reports {bundle_name(kind, version)} but {source} is {bundle_model}; '
                             "leave model out (launch argument and robot.model) to use the "
                             "connected robot's own")
        return source
    source = Path(bundles) / bundle_name(kind, version)
    if not (source / 'model.json').is_file():
        raise ValueError(f'No bundle for the connected robot at {source}; add it with '
                         'docker/make_bundles.sh and rebuild the image')
    return source


class Preparer:
    def __init__(self, node, namespace):
        self.node = node
        self.driver = Driver(node, namespace)
        self.positions, self.received, self.state = {}, 0.0, None
        node.create_subscription(JointState, self.driver.name('joint_states'), self.on_joints,
                                 qos_profile_sensor_data)
        node.create_subscription(RobotState, self.driver.name('robot_state'), self.on_state, 10)

    def on_joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))
        self.received = time.monotonic()

    def on_state(self, msg):
        self.state = msg

    def spin_until(self, done, timeout, what):
        deadline = time.monotonic() + timeout
        while not done():
            if time.monotonic() > deadline:
                raise TimeoutError(f'No {what} from the driver within {timeout:.0f} s')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def check(self, enable):
        self.spin_until(lambda: self.state is not None, 15.0, 'robot_state')
        problem = robot_problem(self.state)
        if problem:
            raise RuntimeError(problem)
        if enable:
            for service in ('robot_power', 'robot_servo'):
                self.driver.switch(service, True)
            log(self.node, 'power and servos on')

    def fresh_positions(self):
        since = time.monotonic()
        self.spin_until(lambda: self.received > since, 5.0, 'joint_states')
        return dict(self.positions)

    def ready(self, arms, move_time, settle=10.0, tolerance=0.05):
        client = self.driver.action(Rby1JointCommand, 'robot_joint')
        try:
            handle = self.driver.wait(client.send_goal_async(ready_goal(arms, move_time)), 10.0)
            if not handle.accepted:
                raise RuntimeError('Driver rejected the ready-pose command')
            result = self.driver.wait(handle.get_result_async(), move_time + 30.0)
            if not result.result.success:
                raise RuntimeError(f'Ready-pose move failed: {result.result.finish_code}')
        finally:
            client.destroy()
        target = {f'{arm}_{i}': q for arm in arms for i, q in enumerate(READY[arm])}
        deadline = time.monotonic() + settle
        while True:
            current = self.fresh_positions()
            off = {name: round(current[name] - q, 3) for name, q in target.items()
                   if abs(current[name] - q) > tolerance}
            if not off:
                return
            if time.monotonic() > deadline:
                raise RuntimeError(f'Arm did not reach the ready pose: {off} (rad off). Check '
                                   'that nothing blocks it, or start without the move '
                                   '(robot.ready_if_straight: false in the config)')


def log(node, text):
    node.get_logger().info(text)


def prepare_driver(robot, args):
    namespace = robot['driver_namespace']
    kind, version, _ = read_robot(namespace)
    source = choose_source(kind, version, robot['model'], args.model_directory,
                           os.environ.get('RBY1_BUNDLES', '/opt/rby1/bundles'))
    group = robot['group']
    metadata, _ = load_model(source, group.replace('+', '_'))
    rclpy.init(args=[])
    node = rclpy.create_node('cumotion_prepare')
    try:
        preparer = Preparer(node, namespace)
        preparer.check(robot['enable_robot'])
        positions = preparer.fresh_positions()
        if robot['ready_if_straight']:
            arms = straight_arms(positions, metadata['active_joints'], robot['straight_elbow'])
            if arms:
                log(node, f'{", ".join(arms)} straight (elbow near 0 rad): moving to the ready pose')
                preparer.ready(arms, robot['ready_time'])
                log(node, f'{", ".join(arms)} at the ready pose')
                positions = preparer.fresh_positions()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    directory = activate(source, group, positions=positions,
                         extra={'hardware': 'driver', 'model': source.name,
                                'driver_namespace': namespace},
                         body_ends_at_tool=robot['body_ends_at_tool'])
    return directory, source.name


def prepare_mock(robot, args):
    bundles = Path(os.environ.get('RBY1_BUNDLES', '/opt/rby1/bundles'))
    source = Path(args.model_directory or bundles / (robot['model'] or os.environ.get('RBY1_MODEL', 'm_1_2')))
    directory = activate(source, robot['group'],
                         extra={'hardware': 'mock', 'model': source.name},
                         body_ends_at_tool=robot['body_ends_at_tool'])
    return directory, source.name


def parse(argv):
    parser = argparse.ArgumentParser(prog='prepare', description=__doc__.split('\n\n')[0])
    parser.add_argument('--config', default='')
    parser.add_argument('--hardware', choices=('driver', 'mock'), default='driver')
    parser.add_argument('--model', default='', help='overrides robot.model')
    parser.add_argument('--model-directory', default='')
    parser.add_argument('--group', default='', help='overrides robot.group')
    parser.add_argument('--driver-namespace', default='', help='overrides robot.driver_namespace')
    return parser.parse_args(argv)


def robot_settings(args):
    robot = settings.load(args.config or None, 'robot')
    for key, value in (('model', args.model), ('group', args.group),
                       ('driver_namespace', args.driver_namespace)):
        if value:
            robot[key] = settings.coerce('robot', key, value, 'launch argument')
    return robot


def main(argv=None):
    args = parse(rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:])
    try:
        check_alone()
        robot = robot_settings(args)
        prepared = prepare_mock(robot, args) if args.hardware == 'mock' else prepare_driver(robot, args)
        directory, model = prepared
        record = json.loads((Path(directory) / 'active.json').read_text())
        print(f'PREPARED model={model} group={record["group"]} posture={record["posture"]} '
              f'hardware={record["hardware"]} bundle={directory}', flush=True)
    except (Exception, KeyboardInterrupt) as error:
        print(f'PREPARE_FAILED: {error}', flush=True)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
