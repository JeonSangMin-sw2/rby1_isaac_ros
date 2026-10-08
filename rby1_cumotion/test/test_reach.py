"""reach.py: what an arm reaches, and the torso posture for what it does not. No ROS, no GPU."""

import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rby1_cumotion import settings
from rby1_cumotion.executor.reach import Follow, JOINTS as TORSO, limit_elbows, OutOfReach, Reach
from rby1_cumotion.model import forward_kinematics

# The torso and right arm of an RB-Y1 m v1.2, joint origins and limits as in its URDF.
JOINTS = [
    ('torso_0', 'base', 'link_torso_0', '0 0 0.2965', '1 0 0', -0.2618, 0.2618),
    ('torso_1', 'link_torso_0', 'link_torso_1', '0 0 0', '0 1 0', -0.5236, 1.5708),
    ('torso_2', 'link_torso_1', 'link_torso_2', '0 0 0.35', '0 1 0', -2.618, 1.5708),
    ('torso_3', 'link_torso_2', 'link_torso_3', '0 0 0.35', '0 1 0', -0.7854, 1.5708),
    ('torso_4', 'link_torso_3', 'link_torso_4', '0 0 0', '1 0 0', -0.5236, 0.5236),
    ('torso_5', 'link_torso_4', 'link_torso_5', '0 0 0.3094', '0 0 1', -2.3562, 2.3562),
    ('right_arm_0', 'link_torso_5', 'link_right_arm_0', '0 -0.22 0.0801', '0 1 0', -3.1416, 3.1416),
    ('right_arm_1', 'link_right_arm_0', 'link_right_arm_1', '0 0 0', '1 0 0', -3.1416, 0.0175),
    ('right_arm_2', 'link_right_arm_1', 'link_right_arm_2', '0 0 0', '0 0 1', -3.1416, 3.1416),
    ('right_arm_3', 'link_right_arm_2', 'link_right_arm_3', '0.031 0 -0.276', '0 1 0', -2.618, 0.0175),
    ('right_arm_4', 'link_right_arm_3', 'link_right_arm_4', '-0.031 0 -0.256', '0 0 1', -3.1416, 3.1416),
    ('right_arm_5', 'link_right_arm_4', 'link_right_arm_5', '0 0 0', '0 1 0', -1.5708, 1.9199),
    ('right_arm_6', 'link_right_arm_5', 'link_right_arm_6', '0 0 0', '0 0 1', -2.7053, 2.7053),
]
READY = settings.load(None, 'robot')['ready_pose']
REACH = settings.load(None, 'reach')


def robot(joints=JOINTS):
    root = ET.Element('robot')
    for name, parent, child, xyz, axis, lower, upper in joints:
        joint = ET.SubElement(root, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=xyz, rpy='0 0 0')
        ET.SubElement(joint, 'axis', xyz=axis)
        ET.SubElement(joint, 'limit', lower=str(lower), upper=str(upper))
    tool = ET.SubElement(root, 'joint', name='ee_right_joint', type='fixed')
    ET.SubElement(tool, 'parent', link='link_right_arm_6')
    ET.SubElement(tool, 'child', link='ee_right')
    ET.SubElement(tool, 'origin', xyz='0 0 -0.1261', rpy='0 0 0')
    return root


def ready():
    return {f'{part}_{i}': q for part in ('torso', 'right_arm') for i, q in enumerate(READY[part])}


def hand(root, positions, forward=0.0, down=0.0, left=0.0):
    target = forward_kinematics(root, positions, 'ee_right')
    target[:3, 3] += [forward, left, -down]
    return target


def test_the_arm_reaches_as_far_as_the_elbow_limit_lets_it():
    root = robot()
    bent = Reach(root, 'right_arm', dict(REACH, elbow_limit=-1.57), READY['torso'])
    # Elbow at a right angle: (0.031, 0, -0.276) + Ry(-1.57) (-0.031, 0, -0.256) = (0.287, 0, -0.307).
    assert bent.radius == pytest.approx(0.4205, abs=1e-3)
    limited = Reach(root, 'right_arm', REACH, READY['torso'])
    # Fully stretched the upper arm and the forearm are in line: |(0.031, -0.276)| + |(-0.031, -0.256)|.
    assert bent.radius < limited.radius < 0.2777 + 0.2579


def test_the_arm_is_stretched_where_its_links_line_up_not_at_zero():
    """The elbow is 31 mm off the line from shoulder to wrist: atan(31/276) + atan(31/256) = 13.3 deg."""
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    assert np.degrees(reach.stretched) == pytest.approx(-13.3, abs=0.1)
    assert REACH['elbow_limit'] == pytest.approx(reach.stretched - np.radians(5.0), abs=2e-3)
    # A limit that leaves the stretched angle inside the elbow's range does not keep the arm off its singularity.
    for limit in (0.0, -0.087, reach.stretched):
        with pytest.raises(ValueError, match='not short of where the right_arm is fully stretched'):
            Reach(root, 'right_arm', dict(REACH, elbow_limit=limit), READY['torso'])


def test_the_wrist_is_found_from_the_hand_target():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    positions = dict(ready(), right_arm_2=0.4, right_arm_5=-0.7)
    assert reach.wrist(forward_kinematics(root, positions, 'ee_right')) == pytest.approx(
        forward_kinematics(root, positions, 'link_right_arm_6')[:3, 3], abs=1e-9)


def away(reach, posture):
    """How far a torso posture {joint: rad} is from the ready pose, as the search counts it."""
    return sum((cost * (posture[name] - ready()[name])) ** 2 for name, cost in zip(TORSO, reach.cost))


def test_a_target_the_arm_reaches_leaves_a_ready_torso_alone():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    assert reach.share(ready(), hand(root, ready(), forward=0.05, down=0.05)) is None
    assert reach.short(ready(), hand(root, ready(), forward=0.05)) < 0.0


def test_the_torso_makes_up_what_the_arm_is_short_and_no_more():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    now = ready()
    target = hand(root, now, forward=0.25)
    short = reach.short(now, target)
    assert 0.0 < short < 0.15
    posture = reach.share(now, target)
    assert set(posture) == set(TORSO)
    after = dict(now, **posture)
    # The target is torso_margin inside the arm's reach now, and no further than that.
    assert -REACH['torso_margin'] - 3e-3 < reach.short(after, target) <= -REACH['torso_margin'] + 1e-9
    _, forward, down, lean, side, turn = reach.torso(np.array([[posture[name] for name in TORSO]]))
    assert -1e-9 <= forward[0] <= REACH['torso_forward'] and down[0] <= REACH['torso_down']
    assert -1e-9 <= lean[0] <= REACH['torso_pitch'] and abs(turn[0]) <= REACH['torso_yaw']
    assert abs(side[0]) <= REACH['torso_roll']
    for name in TORSO:
        lower, upper = reach.limits[name]
        assert lower <= posture[name] <= upper
    assert posture['torso_2'] <= 0.0  # the knee bends the way the ready pose has it (-0.2 rad)
    # A further target takes the torso further from the ready pose.
    assert away(reach, reach.share(now, hand(root, now, forward=0.30))) > away(reach, posture)


def test_where_the_torso_stands_follows_from_the_target_not_from_where_it_was():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    now = ready()
    far, near = hand(root, now, forward=0.28), hand(root, now, forward=0.05)
    leaned = dict(now, **reach.share(now, far))
    # A target the arm reaches from the ready pose: the torso goes back there.
    back = reach.share(leaned, near)
    assert back == pytest.approx({name: now[name] for name in TORSO})
    # The far target again: the torso is where it belongs already, from wherever it is asked.
    assert reach.share(leaned, far) is None
    assert reach.share(dict(now, **back), near) is None


def test_a_following_torso_sets_out_early_and_comes_back_late():
    """Following a moving target: the torso sets out before the arm is at its limit, and one at the
    edge of the arm's reach does not send it to and fro."""
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    margin = REACH['torso_margin']
    edge = hand(root, ready(), forward=0.14)    # the arm reaches it from the ready pose, by under the margin
    band = hand(root, ready(), forward=0.105)   # by more than the margin, less than twice
    inside = hand(root, ready(), forward=0.05)  # by more than twice the margin
    assert -margin < reach.short(ready(), edge) < 0.0
    assert -2 * margin < reach.short(ready(), band) < -margin
    assert reach.short(ready(), inside) < -2 * margin
    # Planned from standing: the torso stays at the ready pose for all three.
    for target in (edge, band, inside):
        assert reach.posture(target) == pytest.approx(reach.ready)
    # Following, at the ready pose: it sets out for the one within the margin of the arm's limit.
    assert np.abs(reach.posture(edge, out=False) - reach.ready).max() > 1e-3
    assert reach.posture(band, out=False) == pytest.approx(reach.ready)
    # Following, away: it comes back only with twice the margin to spare.
    assert np.abs(reach.posture(band, out=True) - reach.ready).max() > 1e-3
    assert reach.posture(inside, out=True) == pytest.approx(reach.ready)
    # Wherever it goes, the target ends twice the margin inside the arm's reach (the search is
    # good to some millimetres: never less inside than that, up to a centimetre more).
    after = dict(zip(TORSO, reach.posture(edge, out=False)))
    assert -2 * margin - 0.01 < reach.short(after, edge) <= -2 * margin + 1e-9
    # Started from the posture it had, the search ends at the same one: no jump between two asks.
    first = reach.posture(edge, out=True)
    again = reach.posture(edge, out=True, near=first)
    assert again == pytest.approx(first, abs=0.02)
    assert reach.posture(edge, out=True, near=again) == pytest.approx(again, abs=0.01)  # and settles


def test_leaning_sideways_reaches_further_out_to_the_side():
    """Turning does not: the shoulder is as far out as it gets already."""
    root = robot()
    target = hand(root, ready(), forward=0.10, left=-0.35)   # the right hand, out to the right
    with pytest.raises(OutOfReach, match='0 deg sideways'):
        Reach(root, 'right_arm', dict(REACH, torso_roll=0.0), READY['torso']).share(ready(), target)
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    posture = reach.share(ready(), target)
    side = reach.torso(np.array([[posture[name] for name in TORSO]]))[4][0]
    assert 0.05 < abs(side) <= REACH['torso_roll']
    for name in ('torso_0', 'torso_4'):
        lower, upper = reach.limits[name]
        assert lower <= posture[name] <= upper


def test_turning_reaches_further_forward_and_across_than_leaning_alone():
    """The right shoulder is 22 cm off the torso's axis: turned to the left it comes forward."""
    root = robot()
    target = hand(root, ready(), forward=0.42)
    with pytest.raises(OutOfReach, match='turn 0 deg'):
        Reach(root, 'right_arm', dict(REACH, torso_yaw=0.0), READY['torso']).share(ready(), target)
    posture = Reach(root, 'right_arm', REACH, READY['torso']).share(ready(), target)
    assert 0.05 < posture['torso_5'] <= REACH['torso_yaw']
    # A target the pitch joints bring in reach is not turned to: that is further from the ready pose.
    assert abs(Reach(root, 'right_arm', REACH, READY['torso']).share(
        ready(), hand(root, ready(), forward=0.22))['torso_5']) < 0.02


def test_a_target_beyond_the_torsos_range_is_refused_with_how_far():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    with pytest.raises(OutOfReach, match=r'out of reach: the target is \d+\.\d cm beyond .* 10 cm forward'):
        reach.share(ready(), hand(root, ready(), forward=0.80))
    # A torso that may not move at all refuses anything the arm does not reach.
    fixed = dict(REACH, torso_forward=0.0, torso_pitch=0.0, torso_down=0.0, torso_yaw=0.0, torso_roll=0.0)
    with pytest.raises(OutOfReach):
        Reach(root, 'right_arm', fixed, READY['torso']).share(ready(), hand(root, ready(), forward=0.25))


def test_a_wider_range_reaches_further():
    root = robot()
    target = hand(root, ready(), forward=0.45, down=0.20)
    with pytest.raises(OutOfReach):
        narrow = dict(REACH, torso_forward=0.02, torso_pitch=0.05, torso_yaw=0.0, torso_roll=0.0)
        Reach(root, 'right_arm', narrow, READY['torso']).share(ready(), target)
    assert Reach(root, 'right_arm', dict(REACH, torso_forward=0.30, torso_pitch=1.0, torso_down=0.3),
                 READY['torso']).share(ready(), target) is not None


def test_an_arm_that_is_not_shoulder_elbow_wrist_is_refused_when_loaded():
    crooked = [(name, parent, child, '0.05 0 0' if name == 'right_arm_5' else xyz, axis, lower, upper)
               for name, parent, child, xyz, axis, lower, upper in JOINTS]
    with pytest.raises(ValueError, match='not a shoulder-elbow-wrist arm'):
        Reach(robot(crooked), 'right_arm', REACH, READY['torso'])


def test_the_elbow_limit_goes_into_the_planning_model():
    root = robot()
    limit_elbows(root, REACH['elbow_limit'])
    elbow = next(joint for joint in root.findall('joint') if joint.get('name') == 'right_arm_3')
    assert float(elbow.find('limit').get('upper')) == REACH['elbow_limit']
    limit_elbows(root, -0.01)  # a looser limit than the model has does not widen it
    assert float(elbow.find('limit').get('upper')) == REACH['elbow_limit']
    with pytest.raises(ValueError, match='below the elbow'):
        limit_elbows(robot(), -3.0)


def test_a_following_torso_steps_toward_where_it_belongs_no_faster_than_allowed():
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    follow = Follow(reach, reach.ready, reach.ready, speed=0.3)
    try:
        assert not follow.out()
        far = hand(root, ready(), forward=0.25)
        follow.ask(far)
        for _ in range(200):  # the thread works the posture out
            if np.abs(follow.wanted - reach.ready).max() > 0.01:
                break
            time.sleep(0.01)
        wanted = follow.wanted.copy()
        # Where the target is twice the margin inside the arm's reach.
        assert -2 * REACH['torso_margin'] - 0.01 < reach.short(dict(zip(TORSO, wanted)), far) <= -2 * REACH[
            'torso_margin'] + 1e-9
        # It starts gently, never goes faster than allowed, and gets there.
        before, steps = reach.ready.copy(), []
        for _ in range(600):
            now = follow.step(0.02).copy()
            steps.append(float(np.abs(now - before).max()))
            before = now
        assert steps[0] < 0.1 * 0.3 * 0.02            # from rest: a fraction of one tick at 0.3 rad/s
        assert max(steps) <= 0.3 * 0.02 + 1e-12
        assert follow.commanded == pytest.approx(wanted, abs=1e-4) and follow.out()
        # The same target asked again: the posture is kept, not worked out anew.
        follow.ask(far)
        time.sleep(0.3)
        assert follow.wanted == pytest.approx(wanted, abs=1e-12)
        # Out of reach: said, and the torso stays where it was wanted last.
        follow.ask(hand(root, ready(), forward=0.90))
        for _ in range(200):
            if follow.note:
                break
            time.sleep(0.01)
        assert 'out of reach' in follow.note and follow.wanted == pytest.approx(wanted)
    finally:
        follow.close()
    assert not follow.thread.is_alive()


def test_a_target_is_handed_to_the_arms_model_as_if_the_torso_had_not_moved():
    """The arm's model has the torso at one posture. Arm joints that put the hand on the transformed
    target there put it on the real target with the torso where it really is."""
    root = robot()
    reach = Reach(root, 'right_arm', REACH, READY['torso'])
    moved = reach.posture(hand(root, ready(), forward=0.30, left=-0.20))
    follow = Follow(reach, reach.ready, moved, speed=0.3)
    try:
        assert Follow(reach, reach.ready, reach.ready, 0.3).as_modelled() == pytest.approx(np.eye(4), abs=1e-12)
        arm = {f'right_arm_{i}': q for i, q in enumerate([0.3, -0.7, 0.2, -1.2, 0.1, 0.4, -0.3])}
        real = forward_kinematics(root, dict(zip(TORSO, moved), **arm), 'ee_right')
        modelled = forward_kinematics(root, dict(zip(TORSO, reach.ready), **arm), 'ee_right')
        assert follow.as_modelled() @ real == pytest.approx(modelled, abs=1e-9)
    finally:
        follow.close()
