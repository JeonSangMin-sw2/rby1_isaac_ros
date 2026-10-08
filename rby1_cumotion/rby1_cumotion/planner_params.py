"""Which cuMotion planner settings exist, and what values each may take.

Defaults do not live here. They come from the planner section of
config/cumotion.yaml, so there is exactly one place to edit them. This module only knows every setting's name,
type and legal range, and turns the YAML plus any launch-argument overrides into
node parameters -- failing at launch on an unknown key, a missing key or a value
out of range, rather than at the first query.
"""

from pathlib import Path


from rby1_cumotion.settings import CONFIG_NAME, read


def _between(low, high):
    return lambda v: low <= v <= high


# name -> (type, validator). Every name is a real cumotion_planner_node
# parameter; the ik_* ones exist only once docker/patches/cumotion_ik.py has run.
SETTINGS = {
    'num_trajopt_time_steps': (int, _between(4, 256)),
    'num_trajopt_seeds': (int, _between(1, 32)),
    'num_graph_seeds': (int, _between(1, 32)),
    'max_attempts': (int, _between(1, 100)),
    'trajopt_finetune_iters': (int, _between(1, 5000)),
    'interpolation_dt': (float, _between(0.001, 0.5)),
    'ik_num_seeds': (int, _between(1, 256)),
    'ik_position_threshold': (float, _between(0.0001, 0.05)),
    'ik_rotation_threshold': (float, _between(0.001, 0.5)),
    'ik_opt_iters': (int, _between(0, 10000)),
    'ik_particle_opt': (bool, None),
    'collision_cache_mesh': (int, _between(0, 1000)),
    'collision_cache_cuboid': (int, _between(0, 1000)),
    'add_ground_plane': (bool, None),
    'read_esdf_world': (bool, None),
    'voxel_size': (float, _between(0.005, 0.5)),
    # Ignored by the node unless override_moveit_scaling_factors is true, so the
    # two are one setting here: empty keeps MoveIt's request scaling in charge,
    # a number takes it over. See cumotion_planner.py:649.
    'time_dilation_factor': (str, lambda v: v == '' or 0.0 < float(v) <= 1.0),
    'enable_curobo_debug_mode': (bool, None),
}

BOOLS = {'true': True, 'false': False}
# A launch argument left at this value defers to the YAML.
FROM_CONFIG = ''


def names():
    return tuple(SETTINGS)


def launch_arguments():
    """Every argument a parent launch must forward to planner.launch.py."""
    return ('config', *SETTINGS)


def as_text(value):
    """Canonical string form, so YAML values and launch strings parse alike."""
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def load_config(path):
    """Read the planner section and insist it names every setting and nothing else."""
    path = Path(path)
    data = read(path).get('planner')
    if not isinstance(data, dict):
        raise ValueError(f'{path}: missing section \'planner\'')
    unknown = sorted(set(data) - set(SETTINGS))
    missing = sorted(set(SETTINGS) - set(data))
    if unknown:
        raise ValueError(f'{path}: unknown planner settings {unknown}')
    if missing:
        raise ValueError(f'{path}: missing planner settings {missing}')
    return {name: as_text(value) for name, value in data.items()}


def declare(DeclareLaunchArgument, default_config=''):
    """Launch arguments: the config file, then one optional override per setting."""
    arguments = [DeclareLaunchArgument(
        'config', default_value=default_config,
        description=f'Settings file (default: the installed {CONFIG_NAME})')]
    for name, (kind, _valid) in SETTINGS.items():
        arguments.append(DeclareLaunchArgument(
            name, default_value=FROM_CONFIG,
            description=f'Override {name} from the planner config ({kind.__name__})'))
    return arguments


def parse(lookup, config):
    """Merge overrides onto the config, validate, and return node parameters.

    `lookup` takes a setting name and returns its launch-argument string, empty
    when not overridden. `config` is the mapping from load_config.
    """
    parsed = {}
    for name, (kind, valid) in SETTINGS.items():
        override = lookup(name)
        text = override if override != FROM_CONFIG else config[name]
        source = 'launch argument' if override != FROM_CONFIG else 'planner config'
        if kind is bool:
            if text not in BOOLS:
                raise ValueError(f'{name} must be true or false, got {text!r} ({source})')
            parsed[name] = BOOLS[text]
            continue
        if kind is str:
            value = text
        else:
            try:
                value = kind(text)
            except ValueError:
                raise ValueError(f'{name} must be {kind.__name__}, got {text!r} ({source})') from None
        try:
            acceptable = valid is None or valid(value)
        except ValueError:
            acceptable = False
        if not acceptable:
            raise ValueError(f'{name} is out of range: {text!r} ({source})')
        parsed[name] = value
    # A speed override is only honoured when the node is told to take over.
    dilation = parsed.pop('time_dilation_factor')
    parsed['override_moveit_scaling_factors'] = bool(dilation)
    if dilation:
        parsed['time_dilation_factor'] = float(dilation)
    return parsed


def resolve(lookup, default_config):
    """Everything a launch file needs: the config path, loaded and merged."""
    path = lookup('config') or default_config
    return parse(lookup, load_config(path))
