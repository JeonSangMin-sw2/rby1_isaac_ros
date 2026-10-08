"""Settings: config/cumotion.yaml is the one source of defaults, launch arguments override."""

from pathlib import Path
import re

import pytest
import yaml

from rby1_cumotion import planner_params, settings

PACKAGE = Path(__file__).resolve().parent.parent
LAUNCH = PACKAGE / 'launch'
CONFIG = PACKAGE / 'config' / planner_params.CONFIG_NAME
PATCH = PACKAGE.parent / 'docker' / 'patches' / 'cumotion_ik.py'


def no_overrides(overrides=None):
    """Mimic LaunchConfiguration.perform: every value arrives as a string."""
    values = {name: planner_params.FROM_CONFIG for name in planner_params.launch_arguments()}
    values.update(overrides or {})
    return values.__getitem__


@pytest.fixture
def config():
    return planner_params.load_config(CONFIG)


def test_shipped_config_names_every_setting_exactly_once(config):
    assert set(config) == set(planner_params.SETTINGS)


def test_shipped_defaults_parse_and_keep_moveit_in_charge_of_speed(config):
    parsed = planner_params.parse(no_overrides(), config)
    assert parsed['num_trajopt_time_steps'] == 32
    assert parsed['interpolation_dt'] == pytest.approx(0.025)
    assert parsed['ik_num_seeds'] == 32
    assert parsed['ik_position_threshold'] == pytest.approx(0.005)
    assert parsed['ik_particle_opt'] is True
    assert parsed['add_ground_plane'] is False
    assert parsed['override_moveit_scaling_factors'] is False
    assert 'time_dilation_factor' not in parsed


def test_a_launch_argument_overrides_only_its_own_entry(config):
    parsed = planner_params.parse(no_overrides({'trajopt_finetune_iters': '150'}), config)
    assert parsed['trajopt_finetune_iters'] == 150
    assert parsed['num_trajopt_time_steps'] == 32  # untouched, still from YAML


def test_editing_the_yaml_changes_the_default(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['planner']['num_trajopt_seeds'] = 12
    data['planner']['ik_num_seeds'] = 64
    edited = tmp_path / 'edited.yaml'
    edited.write_text(yaml.safe_dump(data))
    parsed = planner_params.resolve(no_overrides({'config': str(edited)}), CONFIG)
    assert parsed['num_trajopt_seeds'] == 12
    assert parsed['ik_num_seeds'] == 64


def test_a_speed_override_switches_both_parameters_together(config):
    parsed = planner_params.parse(no_overrides({'time_dilation_factor': '0.4'}), config)
    assert parsed['time_dilation_factor'] == pytest.approx(0.4)
    assert parsed['override_moveit_scaling_factors'] is True


@pytest.mark.parametrize('name,bad', [
    ('num_trajopt_time_steps', '3'),
    ('num_trajopt_seeds', '0'),
    ('interpolation_dt', '2'),
    ('ik_num_seeds', '0'),
    ('ik_position_threshold', '0.5'),
    ('ik_rotation_threshold', '0'),
    ('ik_opt_iters', '-1'),
    ('voxel_size', '0.0001'),
    ('time_dilation_factor', '1.5'),
])
def test_out_of_range_values_fail_at_launch(config, name, bad):
    with pytest.raises(ValueError, match=name):
        planner_params.parse(no_overrides({name: bad}), config)


@pytest.mark.parametrize('name,bad', [
    ('num_trajopt_time_steps', 'many'),
    ('add_ground_plane', 'yes'),
    ('ik_particle_opt', '1'),
])
def test_wrong_types_fail_at_launch(config, name, bad):
    with pytest.raises(ValueError, match=name):
        planner_params.parse(no_overrides({name: bad}), config)


def test_a_bad_yaml_value_is_blamed_on_the_file(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['planner']['interpolation_dt'] = 9.0
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='planner config'):
        planner_params.resolve(no_overrides({'config': str(path)}), CONFIG)


@pytest.mark.parametrize('change,message', [
    (lambda d: d['planner'].update(num_trajopt_seedz=6), 'unknown'),
    (lambda d: d['planner'].pop('ik_num_seeds'), 'missing'),
])
def test_config_must_name_every_setting_and_nothing_else(tmp_path, change, message):
    """A typo'd key would otherwise be ignored and its default silently used."""
    data = yaml.safe_load(CONFIG.read_text())
    change(data)
    path = tmp_path / 'typo.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=message):
        planner_params.load_config(path)


def test_every_setting_is_a_real_planner_parameter():
    """Checked against the installed node; the ik_* ones also need the image patch."""
    source = Path('/opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/'
                  'cumotion_planner.py')
    if not source.is_file():
        pytest.skip('isaac_ros_cumotion is not installed here; run inside the container')
    declared = set(re.findall(r"declare_parameter\(\s*'([^']+)'", source.read_text()))
    exposed = set(planner_params.names()) | {'override_moveit_scaling_factors'}
    missing = exposed - declared
    assert not missing, (f'{sorted(missing)} are not declared by the installed node; '
                         'if they are ik_*, the image lacks docker/patches/cumotion_ik.py')


def test_ik_patch_declares_what_planner_params_exposes():
    """The patch and this module must agree, or an IK setting would do nothing."""
    patched = set(re.findall(r"declare_parameter\('(ik_[a-z_]+)'", PATCH.read_text()))
    exposed = {name for name in planner_params.names() if name.startswith('ik_')}
    assert patched == exposed


def test_no_launch_file_restates_a_default():
    bringup = PACKAGE / 'rby1_cumotion' / 'bringup.py'
    for path in (bringup, LAUNCH / 'planner.launch.py'):
        source = path.read_text()
        assert 'planner_params.declare(DeclareLaunchArgument)' in source, path.name
        for setting in [*planner_params.names(), *settings.SECTIONS['motion']]:
            assert f"'{setting}':" not in source, f'{path.name} restates {setting}'
            assert f"DeclareLaunchArgument('{setting}'" not in source, f'{path.name} declares {setting}'
    assert 'planner_params.launch_arguments()' in bringup.read_text()
    assert 'planner_params.resolve(' in (LAUNCH / 'planner.launch.py').read_text()
    for name in ('cumotion.launch.py', 'demo.launch.py'):
        assert 'launch_description(' in (LAUNCH / name).read_text(), name


def test_every_override_defaults_to_the_config():
    recorded = []

    def fake(name, default_value=None, choices=None, description=None):
        recorded.append((name, default_value))
        return name

    planner_params.declare(fake)
    by_name = dict(recorded)
    assert set(by_name) == set(planner_params.launch_arguments())
    for name in planner_params.names():
        assert by_name[name] == planner_params.FROM_CONFIG, name


def test_one_file_holds_every_section():
    data = settings.read(CONFIG)
    assert set(data) == {'robot', 'motion', 'impedance', 'tracking', 'avoid', 'planner'}
    loaded = settings.load(CONFIG)
    assert loaded['robot']['group'] == 'right_arm' and loaded['robot']['driver_namespace'] == 'rby1'


def test_motion_defaults_match_the_driver_config():
    """The user's choice: the driver's own limits (driver_parameters.yaml) are the defaults."""
    motion = settings.load(CONFIG, 'motion')
    assert motion['duration'] == 0.0
    assert motion['minimum_time'] == 2.0
    assert motion['linear_velocity_limit'] == 1.5
    assert motion['angular_velocity_limit'] == pytest.approx(4.712388)


@pytest.mark.parametrize('section,change,message', [
    ('motion', lambda d: d.update(linear_velocity_limt=1.0), 'unknown motion'),
    ('motion', lambda d: d.pop('minimum_time'), 'missing motion'),
    ('motion', lambda d: d.update(step=0.9), 'motion.step is out of range'),
    ('motion', lambda d: d.update(linear_velocity_limit=0), 'out of range'),
    ('robot', lambda d: d.update(group='both_arms'), 'robot.group is out of range'),
    ('robot', lambda d: d.update(enable_robot='yes'), 'true or false'),
])
def test_robot_and_motion_sections_are_checked(tmp_path, section, change, message):
    data = yaml.safe_load(CONFIG.read_text())
    change(data[section])
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=message):
        settings.load(path)


def test_an_unknown_section_is_refused(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['moton'] = {}
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='unknown sections'):
        settings.read(path)
