"""Who plans a target from standing, and how the torso's postures are checked (the options
avoid.plan_in_executor and reach.quick_check). The executor's methods on stand-ins: no cuRobo."""

from types import SimpleNamespace

import numpy as np
import pytest

from rby1_cumotion import settings
from rby1_cumotion.executor import target_executor
from rby1_cumotion.executor.reach import JOINTS as TORSO
from rby1_cumotion.executor.target_executor import TargetExecutor

ARM = [f'right_arm_{i}' for i in range(7)]
TARGET = np.eye(4)


def test_both_options_are_off_in_the_settings_file():
    config = settings.load(None)
    assert config['avoid']['plan_in_executor'] is False
    assert config['reach']['quick_check'] is False


def planner(plan_in_executor, reach=None):
    calls = []
    node = SimpleNamespace(
        reach=reach, avoid={'plan_in_executor': plan_in_executor}, calls=calls,
        measure=lambda target: calls.append('moveit') or 'from moveit',
        plan_sharing=lambda target: calls.append('sharing') or 'shared',
        refresh_scene=lambda wait=False: calls.append(('scene', wait)),
        fresh_snapshot=lambda: 'now',
        plan_here=lambda current, target: calls.append(('here', current)) or 'from here')
    return node


def test_moveit_plans_a_target_unless_plan_in_executor():
    node = planner(False)
    assert TargetExecutor.plan_target(node, TARGET) == 'from moveit'
    assert node.calls == ['moveit']
    node = planner(True)
    assert TargetExecutor.plan_target(node, TARGET) == 'from here'
    assert node.calls == [('scene', True), ('here', 'now')]  # the scene is read first, and waited for


def test_plan_in_executor_leaves_sharing_with_the_torso_as_it_is():
    node = planner(True, reach=object())
    assert TargetExecutor.plan_target(node, TARGET) == 'shared'
    assert TargetExecutor.plan_target(node, TARGET, torso=False) == 'from here'


def here(plan_in_executor, answers, monkeypatch):
    """plan_here on an avoider that gives `answers` in turn; the graph flag of each replan is kept."""
    monkeypatch.setattr(target_executor, 'validate_trajectory', lambda *args: None)
    asked, answers = [], list(answers)

    def replan(q, v, target, names, attempts=2, graph=True):
        asked.append(graph)
        return answers.pop(0)
    node = SimpleNamespace(metadata={'active_joints': ARM}, avoid={'plan_in_executor': plan_in_executor},
                           plan_around=lambda q, names: None, avoider=SimpleNamespace(replan=replan))
    return node, asked


def test_plan_here_searches_the_graph_unless_plan_in_executor(monkeypatch):
    path = np.zeros((3, 7))
    node, asked = here(False, [(path, None)], monkeypatch)
    planned = TargetExecutor.plan_here(node, dict.fromkeys(ARM, 0.0), TARGET)
    assert asked == [True] and len(planned['trajectory'].joint_trajectory.points) == 3
    node, asked = here(True, [(path, None)], monkeypatch)
    TargetExecutor.plan_here(node, dict.fromkeys(ARM, 0.0), TARGET)
    assert asked == [False]


def test_plan_in_executor_goes_back_to_the_graph_when_the_quick_plan_fails(monkeypatch):
    node, asked = here(True, [(None, 'FINETUNE_TRAJOPT_FAIL'), (np.zeros((2, 7)), None)], monkeypatch)
    planned = TargetExecutor.plan_here(node, dict.fromkeys(ARM, 0.0), TARGET)
    assert asked == [False, True] and len(planned['trajectory'].joint_trajectory.points) == 2
    # Both fail: the target is refused with what the second try said.
    node, asked = here(True, [(None, 'FINETUNE_TRAJOPT_FAIL'), (None, 'GRAPH_FAIL')], monkeypatch)
    with pytest.raises(RuntimeError, match='planning failed: GRAPH_FAIL'):
        TargetExecutor.plan_here(node, dict.fromkeys(ARM, 0.0), TARGET)
    # No pose at the target at all: a second try cannot find one either.
    node, asked = here(True, [(None, 'IK_FAIL')], monkeypatch)
    with pytest.raises(RuntimeError, match='IK_FAIL'):
        TargetExecutor.plan_here(node, dict.fromkeys(ARM, 0.0), TARGET)
    assert asked == [False]


POSTURE = dict(zip(TORSO, [0.0, 0.4, -0.5, 0.4, 0.0, 0.1]))
BEFORE = dict(zip(TORSO, [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]))
MEASURED = {name: value + 0.001 for name, value in POSTURE.items()}


def sharer(quick, hit_at=None):
    """plan_sharing for a target 10 cm beyond the arm; `hit_at`: the check (from 1) that meets something."""
    log = SimpleNamespace(torso=[], checks=[], moves=[])
    current = {**dict.fromkeys(ARM, 0.0), **BEFORE}
    one = np.array([0.0])

    def collision(rows, times, names, torso=None):
        log.checks.append(torso)
        return (0 if len(log.checks) == hit_at else None), False
    node = SimpleNamespace(
        reach=SimpleNamespace(short=lambda positions, target: 0.10, share=lambda positions, target: dict(POSTURE),
                              torso=lambda rows: (None, one, one, one, one, one)),
        reach_cfg={'quick_check': quick}, metadata={'active_joints': ARM}, log=log,
        state={name: (value,) for name, value in MEASURED.items()},
        fresh_snapshot=lambda: dict(current), refresh_scene=lambda wait=False: None, report=lambda text: None,
        set_torso=lambda posture: log.torso.append(dict(posture)), collision=collision,
        plan_here=lambda current, target, where='': 'planned' + where,
        move_torso=lambda current, posture: log.moves.append(dict(posture)),
        get_logger=lambda: SimpleNamespace(warn=lambda text: None))
    return node


def test_the_models_are_moved_for_every_check_without_quick_check():
    node = sharer(False)
    assert TargetExecutor.plan_sharing(node, TARGET) == 'planned with the torso moved'
    assert node.log.checks == [None, None, None]
    assert len(node.log.torso) == 4 and node.log.torso[2] == pytest.approx(POSTURE)
    assert node.log.torso[3] == pytest.approx(MEASURED)  # after the move, where the torso really is
    assert node.log.moves == [POSTURE]


def test_quick_check_asks_about_the_same_postures_and_moves_the_models_twice():
    slow, quick = sharer(False), sharer(True)
    TargetExecutor.plan_sharing(slow, TARGET)
    assert TargetExecutor.plan_sharing(quick, TARGET) == 'planned with the torso moved'
    for asked, moved in zip(quick.log.checks, slow.log.torso):  # the three postures on the way
        assert asked == pytest.approx(moved)
    assert quick.log.torso == [pytest.approx(POSTURE), pytest.approx(MEASURED)]
    assert quick.log.moves == [POSTURE]


@pytest.mark.parametrize('quick', [False, True])
def test_a_posture_that_meets_something_refuses_the_target_and_nothing_moves(quick):
    node = sharer(quick, hit_at=2)
    with pytest.raises(RuntimeError, match='67% into the torso'):
        TargetExecutor.plan_sharing(node, TARGET)
    assert node.log.moves == []
    # The models end where the torso is: put back when they were moved, untouched with quick_check.
    if quick:
        assert node.log.torso == []
    else:
        assert len(node.log.torso) == 3 and node.log.torso[-1] == pytest.approx(BEFORE)
