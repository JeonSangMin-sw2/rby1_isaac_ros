"""Expose cuRobo's IK settings as cumotion_planner_node parameters.

cuMotion solves IK with cuRobo inside MotionGen and never reads MoveIt's
kinematics.yaml, so the only way to tune it is through MotionGenConfig. The
stock node passes none of the IK arguments; this adds five, each defaulting to
cuRobo's own default so an unpatched and a patched node plan identically until a
value is set.

Idempotent, and an anchor that no longer matches exactly once fails the build
rather than silently leaving the node unpatched.
"""

import pathlib
import sys

NODE = pathlib.Path('/opt/ros/humble/lib/python3.10/site-packages/'
                    'isaac_ros_cumotion/cumotion_planner.py')

HELPER = '''def _rby1_ik_options(node):
    """IK arguments for MotionGenConfig, from this node's ik_* parameters."""
    def get(name):
        return node.get_parameter(name).get_parameter_value()
    iters = get('ik_opt_iters').integer_value
    return {
        'num_ik_seeds': get('ik_num_seeds').integer_value,
        'position_threshold': get('ik_position_threshold').double_value,
        'rotation_threshold': get('ik_rotation_threshold').double_value,
        # 0 keeps cuRobo's per-solver default from its YAML.
        'ik_opt_iters': iters if iters > 0 else None,
        'ik_particle_opt': get('ik_particle_opt').bool_value,
    }


class CumotionActionServer(Node):'''

EDITS = (
    # Declare the parameters next to the node's other settings, with cuRobo's
    # MotionGenConfig.load_from_robot_config defaults.
    ("        self.declare_parameter('override_moveit_scaling_factors', False)",
     "        self.declare_parameter('override_moveit_scaling_factors', False)\n"
     "        self.declare_parameter('ik_num_seeds', 32)\n"
     "        self.declare_parameter('ik_position_threshold', 0.005)\n"
     "        self.declare_parameter('ik_rotation_threshold', 0.05)\n"
     "        self.declare_parameter('ik_opt_iters', 0)\n"
     "        self.declare_parameter('ik_particle_opt', True)"),
    # Pass them where the node builds MotionGenConfig.
    ('            finetune_trajopt_iters=self.__trajopt_finetune_iters,\n',
     '            finetune_trajopt_iters=self.__trajopt_finetune_iters,\n'
     '            **_rby1_ik_options(self),\n'),
    ('class CumotionActionServer(Node):', HELPER),
)


def main():
    text = NODE.read_text()
    applied = 0
    for old, new in EDITS:
        if new in text:
            continue
        if text.count(old) != 1:
            sys.exit(f'{NODE}: expected one occurrence to patch, found {text.count(old)}. '
                     'isaac_ros_cumotion changed upstream; revisit docs/developer_manual.md §7.')
        text = text.replace(old, new)
        applied += 1
    NODE.write_text(text)
    print(f'cuMotion IK parameters: {applied} applied, {len(EDITS) - applied} already present')


if __name__ == '__main__':
    main()
