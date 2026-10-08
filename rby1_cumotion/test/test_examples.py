"""The examples' separation of duties, and the benchmark's environment guard."""

import ast
from pathlib import Path

import pytest

from rby1_cumotion.planning import PIPELINES, running_nodes, SINGLETON_NODES

PACKAGE = Path(__file__).resolve().parent.parent / 'rby1_cumotion'
EXECUTION_NAMES = ('ExecuteTrajectory', 'execute_trajectory', 'FollowJointTrajectory')


def imported_names(path):
    tree = ast.parse(path.read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            names.add(node.module or '')
            names.update(alias.name for alias in node.names)
    return names


def referenced_names(path):
    """Identifiers the code actually uses, ignoring comments and docstrings."""
    tree = ast.parse(path.read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(alias.name for alias in node.names)
            if isinstance(node, ast.ImportFrom) and node.module:
                names.add(node.module)
    return names


PLAN_ONLY = ['planning.py', 'tools/check_plan.py', 'tools/benchmark.py', 'bringup/joint_state_relay.py',
             'bringup/driver.py']


@pytest.mark.parametrize('module', PLAN_ONLY)
def test_plan_only_examples_cannot_execute(module):
    """A plan-only example must not even be able to reach an execution action.

    Pointed at a live robot it should be inert, so this is a structural check
    rather than a behavioural one: the symbols simply are not there. Prose may
    still mention them -- check_plan's docstring explains what it will not do.
    """
    used = referenced_names(PACKAGE / module)
    for name in EXECUTION_NAMES:
        assert name not in used, f'{module} uses {name}'
    # Nothing may set plan_only to anything but True.
    for node in ast.walk(ast.parse((PACKAGE / module).read_text())):
        if isinstance(node, ast.Assign) and any(
                getattr(target, 'attr', None) == 'plan_only' for target in node.targets):
            assert node.value.value is True, ast.unparse(node)


@pytest.mark.parametrize('module', ['executor/target_executor.py'])
def test_executing_examples_share_one_execution_path(module):
    """It goes through execution.py to the driver and owns no second path."""
    imported = imported_names(PACKAGE / module)
    assert 'rby1_cumotion.executor.execution' in imported and 'DriverExecutor' in imported
    assert 'rby1_cumotion.planning' in imported
    source = (PACKAGE / module).read_text()
    for duplicated in ('def plan(', 'def plan_to(', 'def snapshot(', 'def warm(', 'def retime('):
        assert duplicated not in source, f'{module} re-implements {duplicated}'
    # MoveIt execution and ros2_control are not the path: the driver refuses its
    # trajectory action while ros2_control's hardware plugin holds the robot.
    used = referenced_names(PACKAGE / module)
    assert 'ExecuteTrajectory' not in used


def test_execution_goes_to_the_driver_trajectory_action():
    used = referenced_names(PACKAGE / 'executor' / 'execution.py')
    assert 'FollowJointTrajectory' in used
    assert 'ExecuteTrajectory' not in used


def test_every_example_is_installed_as_a_command():
    setup = (PACKAGE.parent / 'setup.py').read_text()
    for command in ('check_plan', 'benchmark', 'prepare_model', 'target_executor',
                    'joint_state_relay'):
        assert f"'{command} = rby1_cumotion." in setup, command
    # Targets are a plain topic any ROS node can publish; the example publisher
    # (16_target_shuttle_publisher) lives with the driver in rby1_examples.
    assert 'target_publisher' not in setup


def test_running_nodes_matches_the_executable_not_the_arguments(tmp_path):
    """A launch file that names a node in an argument is not that node.

    The earlier shell version used `pgrep -f`, which also matched the launcher
    and its own grep; miscounting here silently allows a polluted measurement.
    """
    def write(pid, cmdline):
        entry = tmp_path / str(pid)
        entry.mkdir()
        (entry / 'cmdline').write_bytes(cmdline.encode())

    write(11, '/opt/ros/humble/lib/moveit_ros_move_group/move_group\0--ros-args\0')
    write(12, '/usr/bin/python3\0/opt/ros/humble/bin/ros2\0launch\0demo.launch.py\0')
    write(13, '/bin/bash\0-lc\0ps -eo args | grep move_group\0')
    write(14, '/opt/ros/humble/lib/controller_manager/ros2_control_node\0--ros-args\0')
    # A ROS Python node hides behind the interpreter, so the name is argv[1].
    write(15, '/usr/bin/python3\0/opt/ros/humble/lib/isaac_ros_cumotion/cumotion_planner_node\0')
    (tmp_path / 'not-a-pid').mkdir()

    found = running_nodes(proc=str(tmp_path))
    assert len(found['move_group']) == 1
    assert len(found['ros2_control_node']) == 1
    assert len(found['cumotion_planner_node']) == 1
    assert 'pid 11' in found['move_group'][0]
    assert 'pid 15' in found['cumotion_planner_node'][0]


def test_duplicate_nodes_are_reported_with_pids(tmp_path):
    for pid in (21, 22):
        entry = tmp_path / str(pid)
        entry.mkdir()
        (entry / 'cmdline').write_bytes(
            b'/opt/ros/humble/lib/moveit_ros_move_group/move_group\0')
    found = running_nodes(proc=str(tmp_path))
    assert len(found['move_group']) == 2
    assert {'pid 21', 'pid 22'} == {entry.split(':')[0] for entry in found['move_group']}


def test_benchmark_refuses_a_dirty_environment():
    """The guard must fire on the exact shape that produced the bad numbers."""
    source = (PACKAGE / 'tools' / 'benchmark.py').read_text()
    tree = ast.parse(source)
    guard = next(node for node in ast.walk(tree)
                 if isinstance(node, ast.FunctionDef) and node.name == 'check_environment')
    body = ast.unparse(guard)
    assert 'running_nodes' in body and 'active_controllers' in body
    assert 'raise' in body
    # Both invariants, not just the easy one.
    assert 'expected_nodes' in body
    assert "self.metadata['controllers']" in body


def test_expected_node_counts_follow_the_pipeline():
    """OMPL runs launch no cuMotion planner, so demanding one rejects a clean run."""
    from rby1_cumotion.tools.benchmark import Benchmark

    class Stub:  # borrow the method without starting ROS
        expected_nodes = Benchmark.expected_nodes

        def __init__(self, pipeline, hardware='mock'):
            self.values = {'pipeline': pipeline, 'hardware': hardware}

        def param(self, name):
            return self.values[name]

    ompl = Stub('ompl').expected_nodes()
    cumotion = Stub('isaac_ros_cumotion').expected_nodes()
    assert ompl['cumotion_planner_node'] == (0, 1)
    assert cumotion['cumotion_planner_node'] == (1, 1)
    for expected in (ompl, cumotion):
        assert expected['move_group'] == (1, 1)
        assert expected['ros2_control_node'] == (1, 1)
        assert set(expected) == set(SINGLETON_NODES)
    # With the driver, any ros2_control_node would lock the robot away from it.
    driver = Stub('isaac_ros_cumotion', 'driver').expected_nodes()
    assert driver['ros2_control_node'] == (0, 0)
    assert driver['cumotion_planner_node'] == (1, 1)


def test_benchmark_warns_below_ten_runs():
    source = (PACKAGE / 'tools' / 'benchmark.py').read_text()
    assert 'runs < 10' in source and 'warn' in source


def test_pipelines_are_the_two_this_project_compares():
    assert PIPELINES == ('isaac_ros_cumotion', 'ompl')
    assert SINGLETON_NODES == ('move_group', 'cumotion_planner_node', 'ros2_control_node')


def test_no_example_shells_out():
    """No subprocess: `pkill -f "ros2 launch"` orphans the nodes it should remove,
    and `pgrep -f` counts its own command line. /proc is read directly instead."""
    # The examples talk to ROS only; none has a reason to leave the process.
    for name in sorted(str(p.relative_to(PACKAGE)) for p in PACKAGE.rglob('*.py')):
        used = referenced_names(PACKAGE / name)
        for forbidden in ('subprocess', 'system', 'popen', 'check_output', 'check_call'):
            assert forbidden not in used, f'{name} uses {forbidden}'


def test_planning_layer_has_no_leftover_module_state():
    """Two examples import this; module-level mutable state would leak between them."""
    tree = ast.parse((PACKAGE / 'planning.py').read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign):
            assert isinstance(node.value, (ast.Constant, ast.Tuple)), ast.unparse(node)


@pytest.mark.parametrize('module,says', [
    ('tools/check_plan.py', 'Never moves the robot'),
    ('executor/target_executor.py', 'Moves the robot'),
])
def test_docstrings_say_which_example_moves_the_robot(module, says):
    assert says in ast.get_docstring(ast.parse((PACKAGE / module).read_text()))


def test_no_node_shadows_rclpy_attributes():
    """Both have happened: `self.executor = ...` hit rclpy's executor setter, which
    calls add_node() on its argument; a method named `handle` hid Node.handle, so
    Node.__init__'s `with self.handle:` failed with AttributeError: __enter__.
    Checked against rclpy itself, so a new Node attribute is covered too."""
    from rclpy.node import Node
    reserved = {name for name in dir(Node) if not name.startswith('_')}
    for path in PACKAGE.rglob('*.py'):
        tree = ast.parse(path.read_text())
        for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
            for item in cls.body:
                if isinstance(item, ast.FunctionDef) and item.name in reserved:
                    # Overriding a Node method on purpose is fine only for these.
                    assert item.name in {'destroy_node'}, f'{path.name}: {cls.name}.{item.name}'
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if (isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name)
                            and target.value.id == 'self'):
                        assert target.attr not in reserved, f'{path.name} assigns self.{target.attr}'


def test_no_single_call_site_logs_at_two_severities():
    """`(logger.error if failed else logger.info)(text)` killed target_executor on its
    first FAILED: rclpy binds a call site to one severity and raises on the second."""
    for path in (PACKAGE / 'rby1_cumotion').glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.IfExp):
                branches = [ast.unparse(node.func.body), ast.unparse(node.func.orelse)]
                assert not any('get_logger()' in branch for branch in branches), \
                    f'{path.name}:{node.lineno} {ast.unparse(node.func)}'
