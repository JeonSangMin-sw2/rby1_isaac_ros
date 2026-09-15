"""Move the local Docker simulator forward/back and write a feedback report.

Run after sim.launch.py is ready. This integration test executes motion.
"""

import json
from pathlib import Path
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from rby1_cumotion.model import forward_kinematics
from rby1_cumotion.move_arm import MoveArm
from rby1_cumotion.prepare_sim import check_simulator


def verify(node, report):
    check_simulator('rby1-cumotion-sim', node.metadata['model'])
    driver = {}

    def on_driver_state(msg):
        driver.clear()
        driver.update(zip(msg.name, msg.position))

    subscription = node.create_subscription(JointState, '/rby1/joint_states', on_driver_state, 10)

    def sample(seconds=1.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        state = node.snapshot()
        if not all(n in driver for n in state):
            raise RuntimeError('Missing driver feedback')
        return {n: driver[n] for n in state}

    def tip(state):
        return forward_kinematics(node.robot, state, node.metadata['tool_frame'])[:3, 3]

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(name + ' unavailable')
            return node.wait(client.call_async(request), 10)
        finally:
            node.destroy_client(client)

    before_plan = sample()
    request = GetStateValidity.Request()
    request.group_name = node.metadata['group']
    request.robot_state.joint_state.name = list(before_plan)
    request.robot_state.joint_state.position = list(before_plan.values())

    def validity():
        return call(GetStateValidity, '/check_state_validity', request)

    report['initial_state_valid'] = validity().valid
    assert report['initial_state_valid']
    obstacle = CollisionObject(id='rby1_cumotion_validation_obstacle')
    obstacle.header.frame_id = 'base'
    position = forward_kinematics(node.robot, before_plan, f'link_{node.metadata["arm"]}_arm_3')[:3, 3]
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = position.tolist()
    obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.2, 0.2, 0.2])]
    obstacle.primitive_poses = [pose]
    obstacle.operation = CollisionObject.ADD
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [obstacle]

    def apply_scene():
        result = call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise RuntimeError('Failed to update the validation obstacle')

    try:
        apply_scene()
        blocked = validity()
        report['obstacle_state_valid'] = blocked.valid
        report['collision_contacts'] = [(c.contact_body_1, c.contact_body_2) for c in blocked.contacts]
        assert not blocked.valid, 'Intersecting obstacle was not rejected by MoveIt'
    finally:
        obstacle.operation = CollisionObject.REMOVE
        apply_scene()
    assert validity().valid

    node.set_parameters([Parameter('execute', value=False)])
    node.run()
    after_plan = sample()
    report['plan_only_max_joint_change_rad'] = max(abs(after_plan[n] - before_plan[n]) for n in before_plan)
    assert report['plan_only_max_joint_change_rad'] < 0.001
    node.set_parameters([Parameter('execute', value=True), Parameter('velocity_scaling', value=0.02),
                         Parameter('acceleration_scaling', value=0.02)])
    initial = tip(after_plan)
    for offset in ([0.03, 0.0, 0.0], [-0.03, 0.0, 0.0]):
        before = sample(0.3)
        node.set_parameters([Parameter('offset_xyz', value=offset)])
        node.run()
        after = sample()
        measured = tip(after) - tip(before)
        error = float(np.linalg.norm(measured - np.array(offset)))
        item = {
            'offset_m': offset, 'measured_displacement_m': measured.tolist(), 'endpoint_error_m': error,
            'max_locked_change_rad': max(abs(after[n] - before[n]) for n in node.metadata['locked_joints']),
            'driver_vs_controller_max_error_rad': max(abs(after[n] - node.snapshot()[n]) for n in after),
        }
        report['measurements'].append(item)
        print('MEASURED', item, flush=True)
        assert error < 0.004, 'Measured end effector differs from the requested displacement'
    report['round_trip_error_m'] = float(np.linalg.norm(tip(after) - initial))
    report['success'] = True
    node.destroy_subscription(subscription)


def main():
    rclpy.init()
    node = MoveArm()
    node.declare_parameter('report_file', 'simulation_report.json')
    report = {'pipeline': node.param('pipeline'), 'model': node.metadata['model'],
              'success': False, 'measurements': []}
    try:
        verify(node, report)
    except BaseException as error:
        report['error'] = str(error)
        if node.active_goal and node.active_goal.accepted:
            node.wait(node.active_goal.cancel_goal_async(), 5.0)
        raise
    finally:
        Path(node.get_parameter('report_file').value).write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
