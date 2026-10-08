"""The robot and motion sections of config/cumotion.yaml.

The file is the single source of defaults; this module only knows each setting's
type and legal range. planner_params.py does the same for the planner section.
Nodes started by the launch and nodes run on their own read the same file.
"""

import math
from pathlib import Path

import yaml

CONFIG_NAME = 'cumotion.yaml'
GROUPS = ('right_arm', 'left_arm', 'right_arm+torso', 'left_arm+torso')
TRACKING_METHODS = ('mpc', 'ik')


PARTS = {'right_arm': 7, 'left_arm': 7, 'torso': 6}  # the parts with a ready pose, and their joints


def _between(low, high):
    return lambda v: math.isfinite(v) and low <= v <= high


def _ready_pose(value):
    """robot.ready_pose: joint angles (rad) for every part of PARTS, and nothing else."""
    unknown = sorted(set(value) - set(PARTS))
    if unknown:
        raise ValueError(f'robot.ready_pose: unknown parts {unknown}; it has {", ".join(PARTS)}')
    for part, count in PARTS.items():
        pose = value.get(part)
        if not (isinstance(pose, list) and len(pose) == count and all(
                isinstance(q, (int, float)) and not isinstance(q, bool) and math.isfinite(q) for q in pose)):
            raise ValueError(f'robot.ready_pose.{part} needs {count} joint angles (rad), got {pose!r}')
    return True


# section -> name -> (type, validator)
SECTIONS = {
    'robot': {
        'driver_namespace': (str, None),
        'model': (str, None),
        'group': (str, lambda v: v in GROUPS),
        'enable_robot': (bool, None),
        'ready_if_straight': (bool, None),
        'straight_elbow': (float, _between(0.0, 1.5)),
        'ready_time': (float, _between(0.5, 60.0)),
        'ready_pose': (dict, _ready_pose),
        'body_ends_at_tool': (bool, None),
        'free_objects': (list, lambda v: all(isinstance(name, str) for name in v)),
    },
    'reach': {
        'use_torso': (bool, None),
        'quick_check': (bool, None),
        'elbow_limit': (float, _between(-1.5, 0.0)),
        'torso_forward': (float, _between(0.0, 0.5)),
        'torso_down': (float, _between(0.0, 0.5)),
        'torso_pitch': (float, _between(0.0, 1.57)),
        'torso_yaw': (float, _between(0.0, 2.3)),
        'torso_roll': (float, _between(0.0, 0.79)),
        'turn_cost': (float, _between(0.1, 100.0)),
        'side_cost': (float, _between(0.1, 100.0)),
        'torso_margin': (float, _between(0.0, 0.2)),
        'torso_time': (float, _between(0.5, 60.0)),
        'torso_speed': (float, _between(0.01, 2.0)),
    },
    'motion': {
        'duration': (float, _between(0.0, 600.0)),
        'minimum_time': (float, _between(0.0, 600.0)),
        'linear_velocity_limit': (float, _between(0.001, 10.0)),
        'angular_velocity_limit': (float, _between(0.001, 50.0)),
        'step': (float, _between(0.001, 0.5)),
        'hold': (float, _between(0.0, 10.0)),
        'endpoint_tolerance': (float, _between(0.001, 1.0)),
    },
    'impedance': {
        'enabled': (bool, None),
        'stiffness': (float, _between(1.0, 5000.0)),
        'damping_ratio': (float, _between(0.01, 10.0)),
        'torque_limit': (float, _between(0.1, 300.0)),
    },
    'tracking': {
        'method': (str, lambda v: v in TRACKING_METHODS),
        'prepare': (bool, None),
        'rate': (float, _between(10.0, 100.0)),
        'iterations': (int, _between(1, 8)),
        'max_speed': (float, _between(0.05, 6.0)),
        'max_acceleration': (float, _between(0.1, 100.0)),
        'stale_after': (float, _between(0.1, 10.0)),
        'resync_tolerance': (float, _between(0.02, 1.0)),
        'command_delay': (float, _between(0.0, 1.0)),
        'smoothing': (float, _between(0.0, 0.95)),
        'lead': (float, _between(0.0, 2.0)),
        'lead_max': (float, _between(0.0, 0.5)),
    },
    'avoid': {
        'enabled': (bool, None),
        'plan_in_executor': (bool, None),
        'horizon': (float, _between(0.05, 5.0)),
        'lead': (float, _between(0.1, 2.0)),
        'give_up_after': (float, _between(0.5, 60.0)),
        'wait_before_replan': (float, _between(0.2, 30.0)),
        'max_replans': (int, _between(0, 20)),
        'check_period': (float, _between(0.01, 1.0)),
        'scene_period': (float, _between(0.02, 2.0)),
        'sweep_time': (float, _between(0.0, 5.0)),
        'replan_time': (float, _between(0.02, 2.0)),
        'stop_time': (float, _between(0.1, 3.0)),
        'replan_seeds': (int, _between(1, 32)),
        'replan_steps': (int, _between(4, 256)),
        'replan_iters': (int, lambda v: v == 0 or 10 <= v <= 5000),
    },
}


def default_path():
    """The installed file, or the source tree's when running from a checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('rby1_cumotion')) / 'config' / CONFIG_NAME
    except Exception:  # not installed: tests, or a plain checkout
        return Path(__file__).resolve().parent.parent / 'config' / CONFIG_NAME


def read(path=None):
    """The whole file as a mapping of sections."""
    path = Path(path) if path else default_path()
    if not path.is_file():
        raise ValueError(f'cuMotion config not found: {path}')
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path} must be a mapping of sections')
    unknown = sorted(set(data) - {*SECTIONS, 'planner'})
    if unknown:
        raise ValueError(f'{path}: unknown sections {unknown}')
    return data


def coerce(section, name, value, source):
    kind, valid = SECTIONS[section][name]
    if kind is bool:
        if isinstance(value, str):
            if value not in ('true', 'false'):
                raise ValueError(f'{section}.{name} must be true or false, got {value!r} ({source})')
            value = value == 'true'
        if not isinstance(value, bool):
            raise ValueError(f'{section}.{name} must be true or false, got {value!r} ({source})')
        return value
    try:
        value = kind(value)
    except (TypeError, ValueError):
        raise ValueError(f'{section}.{name} must be {kind.__name__}, got {value!r} ({source})') from None
    if valid is not None and not valid(value):
        raise ValueError(f'{section}.{name} is out of range: {value!r} ({source})')
    return value


def load(path=None, section=None):
    """Validated robot and motion settings: {'robot': {...}, 'motion': {...}}.

    Every key must be present and known -- a typo would otherwise leave its
    default silently in force.
    """
    path = Path(path) if path else default_path()
    data = read(path)
    result = {}
    for name in ([section] if section else SECTIONS):
        values = data.get(name)
        if not isinstance(values, dict):
            raise ValueError(f'{path}: missing section {name!r}')
        unknown = sorted(set(values) - set(SECTIONS[name]))
        missing = sorted(set(SECTIONS[name]) - set(values))
        if unknown:
            raise ValueError(f'{path}: unknown {name} settings {unknown}')
        if missing:
            raise ValueError(f'{path}: missing {name} settings {missing}')
        result[name] = {key: coerce(name, key, value, str(path)) for key, value in values.items()}
    return result[section] if section else result
