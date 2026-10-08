"""Sharing a far hand target between the arm and the torso (settings: reach).

The arm goes first: for a target the arm reaches from the ready pose, with its elbow
no straighter than reach.elbow_limit, the torso belongs at the ready pose. For a
target further away the arm is taken as stretched to that limit and the torso moves
the shoulder the rest of the way -- to the posture nearest the ready pose that does
it, and inside what the torso is allowed: the chest at most reach.torso_forward
forward of and reach.torso_down below where the ready pose has it, leaning at most
reach.torso_pitch forward (torso_1..3) and reach.torso_roll to a side (torso_0,
torso_4), turned at most reach.torso_yaw either way (torso_5). "Nearest" counts a
radian of turning and of leaning sideways for more than one of leaning forward
(reach.turn_cost, reach.side_cost), so those are used when leaning forward is not
enough. So where
the torso stands follows from the target alone, not from where it was: it comes back
when the target does. A target that is out of reach even then is refused
(OutOfReach); the torso is not moved to try.

This module only works out the torso's posture. No ROS, no cuRobo: the URDF tree and
numpy. Whether the arm really gets there from that posture -- its joint limits,
obstacles -- is the planner's to say.

The reach test: an RB-Y1 arm is a shoulder (joints 0-2 meet in a point), an elbow
(joint 3) and a wrist (joints 4-6 meet in a point), so how far the wrist is from the
shoulder depends on the elbow alone, and the wrist's place follows from the hand
target. The target is in reach when the wrist's place is within that distance of the
shoulder.
"""

import threading

import numpy as np

from rby1_cumotion.model import origin

JOINTS = tuple(f'torso_{i}' for i in range(6))  # a torso posture: roll, pitch x3, roll, yaw
ROLL, PITCH, YAW = (0, 4), (1, 2, 3), 5         # their places in a posture
KNEE = 2
CHEST = 'link_torso_5'
TOOLS = {'right_arm': 'ee_right', 'left_arm': 'ee_left'}
# The search for a posture: samples around the best so far, ever closer. The last
# spread, 1.5 mrad, is under 2 mm at the shoulder.
SPREADS = (0.35, 0.15, 0.06, 0.025, 0.01, 0.004, 0.0015)  # rad
SAMPLES = 12000  # per spread
STILL = 2e-3     # rad: a torso this near where it belongs is not moved
APPROACH = 4.0   # 1/s: a following torso's speed toward where it is wanted, per radian it is off


class OutOfReach(ValueError):
    """The target is beyond the arm and what the torso is allowed."""


def chain(root, tip, base='base'):
    """The joints from `base` to the link `tip`: [(name or None if fixed, origin 4x4, unit axis or None)]."""
    by_child = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
    joints, link = [], tip
    while link != base:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f'the URDF has no chain from {base} to {tip}')
        axis = None
        if joint.get('type') != 'fixed':
            axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
            axis = axis / np.linalg.norm(axis)
        joints.append((joint.get('name') if axis is not None else None, origin(joint.find('origin')), axis))
        link = joint.find('parent').get('link')
    return joints[::-1]


def frames(joints, positions, sampled=None):
    """Poses (N, 4, 4) of the chain's tip. `sampled`: {joint: (N,) angles}; the others from `positions`."""
    sampled = sampled or {}
    count = len(next(iter(sampled.values()))) if sampled else 1
    pose = np.tile(np.eye(4), (count, 1, 1))
    for name, fixed, axis in joints:
        pose = pose @ fixed
        if axis is None:
            continue
        angles = np.asarray(sampled[name]) if name in sampled else np.full(count, float(positions[name]))
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        turn = np.tile(np.eye(4), (count, 1, 1))
        turn[:, :3, :3] = (np.eye(3) + np.sin(angles)[:, None, None] * skew
                           + (1 - np.cos(angles))[:, None, None] * (skew @ skew))
        pose = pose @ turn
    return pose


def child_link(root, joint_name):
    for joint in root.findall('joint'):
        if joint.get('name') == joint_name:
            return joint.find('child').get('link')
    raise ValueError(f'the URDF has no joint {joint_name}')


def limit_elbows(root, limit):
    """The elbows' upper limit in the URDF tree becomes `limit` (rad) where it is above it."""
    for joint in root.findall('joint'):
        if joint.get('name') in ('right_arm_3', 'left_arm_3'):
            bounds = joint.find('limit')
            if float(bounds.get('upper')) > limit:
                if float(bounds.get('lower')) >= limit:
                    raise ValueError(f'reach.elbow_limit {limit} rad is below the elbow\'s lower limit')
                bounds.set('upper', repr(float(limit)))


class Reach:
    """What one arm reaches from where the torso is, and the torso posture for what it does not."""

    def __init__(self, root, arm, settings, ready_torso):
        """`settings`: the reach section. `ready_torso`: robot.ready_pose.torso, where the torso's range starts."""
        self.arm, self.settings = arm, settings
        zero = {joint.get('name'): 0.0 for joint in root.findall('joint') if joint.get('type') != 'fixed'}
        shoulder_link, wrist_link = child_link(root, f'{arm}_0'), child_link(root, f'{arm}_6')
        self.to_shoulder = chain(root, shoulder_link)
        self.to_chest = chain(root, CHEST)
        to_wrist, to_tool = chain(root, wrist_link), chain(root, TOOLS[arm])
        self.wrist_from_tool = np.linalg.inv(frames(to_tool, zero)[0]) @ frames(to_wrist, zero)[0]

        def stretch(elbow, others):
            pose = dict(zero, **{f'{arm}_{i}': elbow if i == 3 else others[i] for i in range(7)})
            return float(np.linalg.norm(frames(to_wrist, pose)[0, :3, 3] - frames(self.to_shoulder, pose)[0, :3, 3]))
        # Where the arm is fully stretched -- its singularity. Not at 0 when the elbow sits off
        # the line from shoulder to wrist (RB-Y1: 31 mm, which puts it at -13.3 deg).
        # Looked for over a whole turn, not the joint's range: limit_elbows may already have cut that.
        angles = np.arange(-np.pi, np.pi, 1e-3)
        lengths = np.linalg.norm(frames(to_wrist, zero, {f'{arm}_3': angles})[:, :3, 3]
                                 - frames(self.to_shoulder, zero)[0, :3, 3], axis=1)
        self.stretched = float(angles[int(np.argmax(lengths))])
        elbow = float(settings['elbow_limit'])
        if elbow > self.stretched - 1e-3:
            raise ValueError(
                f'reach.elbow_limit {elbow:.3f} rad ({np.degrees(elbow):.1f} deg) is not short of where the {arm} '
                f'is fully stretched, {self.stretched:.3f} rad ({np.degrees(self.stretched):.1f} deg): the elbow '
                'could still go through its singularity. Give an angle further bent than that')
        self.radius = stretch(elbow, np.zeros(7))
        # The reach test stands on the shoulder and the wrist each being one point.
        rng = np.random.default_rng(0)
        if any(abs(stretch(elbow, rng.uniform(-1.5, 1.5, 7)) - self.radius) > 1e-6 for _ in range(5)):
            raise ValueError(f'{arm} is not a shoulder-elbow-wrist arm in this URDF: the reach test does not hold')
        self.limits = {}
        for joint in root.findall('joint'):
            if joint.get('name') in JOINTS:
                self.limits[joint.get('name')] = (float(joint.find('limit').get('lower')),
                                                  float(joint.find('limit').get('upper')))
        self.ready = np.array([float(q) for q in ready_torso])
        self.ready_chest = frames(self.to_chest, dict(zip(JOINTS, self.ready)))[0, :3, 3]
        self.shoulder_in_chest = (np.linalg.inv(frames(self.to_chest, zero)[0])
                                  @ frames(self.to_shoulder, zero)[0])[:, 3]
        self.cost = np.ones(len(JOINTS))
        self.cost[list(ROLL)], self.cost[YAW] = settings['side_cost'], settings['turn_cost']
        # Where a posture may be at all: the joints' limits, the turn allowed, and the knee on
        # the side the ready pose has it -- through straight is the torso's own singularity,
        # and past it the robot stands the other way round.
        low = np.array([self.limits[name][0] for name in JOINTS])
        high = np.array([self.limits[name][1] for name in JOINTS])
        low[YAW] = max(low[YAW], self.ready[YAW] - settings['torso_yaw'])
        high[YAW] = min(high[YAW], self.ready[YAW] + settings['torso_yaw'])
        if self.ready[KNEE] < 0.0:
            high[KNEE] = 0.0
        elif self.ready[KNEE] > 0.0:
            low[KNEE] = 0.0
        self.low, self.high = low, high

    def wrist(self, target):
        """Where the wrist is with the tool at the 4x4 `target`."""
        return (np.asarray(target, dtype=float) @ self.wrist_from_tool)[:3, 3]

    def chest(self, posture):
        """The chest's pose (N, 4, 4) in base for torso postures (N, 6)."""
        posture = np.asarray(posture, dtype=float)
        return frames(self.to_chest, {}, {name: posture[:, i] for i, name in enumerate(JOINTS)})

    def torso(self, posture):
        """For torso postures (N, 6): the shoulder (N, 3), and how far the chest is forward and
        down (m) and leaning forward, leaning sideways and turned (rad) from the ready pose."""
        posture = np.asarray(posture, dtype=float)
        chest = self.chest(posture)
        off = posture - self.ready
        return (chest @ self.shoulder_in_chest)[:, :3], chest[:, 0, 3] - self.ready_chest[0], \
            self.ready_chest[2] - chest[:, 2, 3], off[:, PITCH].sum(axis=1), off[:, ROLL].sum(axis=1), off[:, YAW]

    def short(self, positions, target, posture=None):
        """m the wrist's place is beyond the arm, from where the torso is or from `posture` (6,);
        0 or less: the arm reaches."""
        if posture is None:
            posture = [positions[name] for name in JOINTS]
        return float(np.linalg.norm(self.torso(np.array([posture]))[0][0] - self.wrist(target)) - self.radius)

    def posture(self, target, out=None, near=None):
        """Where the torso belongs for the hand `target`, as (6,): the ready pose when the arm
        reaches the target from there, else the posture nearest the ready pose that brings it
        reach.torso_margin inside the arm's reach.

        `out`, for a torso that follows a moving target (Follow) -- whether it is away from the
        ready pose now. It then sets out before the arm is at its limit, when the target comes
        within the margin of it, brings the target twice the margin inside, and goes back
        only once the arm reaches the target from the ready pose with twice the margin to
        spare: the arm keeps room to follow while the torso catches up, and a target at the
        edge of the arm's reach does not send the torso to and fro. `near`: the posture it
        was given last; the search starts there, so the postures of a target that moves
        follow one another without jumps.

        Raises OutOfReach when the torso, inside its allowed range, does not bring it in reach.
        The same target gives the same posture: the search draws the same samples every time.
        """
        wrist, limits = self.wrist(target), self.settings
        margin = limits['torso_margin']
        stay, inside = (0.0, margin) if out is None else (-2.0 * margin if out else -margin, 2.0 * margin)
        if np.linalg.norm(self.torso(self.ready[None])[0][0] - wrist) - self.radius <= stay:
            return self.ready.copy()
        if near is not None:
            try:
                return self.search(wrist, inside, np.asarray(near, dtype=float), SPREADS[2:], stay=True)
            except OutOfReach:
                pass  # nothing in reach near there: look everywhere
        return self.search(wrist, inside, self.ready, SPREADS)

    def search(self, wrist, inside, centre, spreads, stay=False):
        """The allowed posture nearest the ready pose that has the wrist's place `inside` m within the
        arm's reach, looked for around `centre`.

        `stay`: nearness to `centre` counts as much as nearness to the ready pose. Many postures
        put the shoulder in the same place and are as near the ready pose, to within what the
        search can tell: without this, two searches for the same target ended 0.03 rad apart.
        """
        limits = self.settings
        start = np.array(centre, dtype=float)
        rng = np.random.default_rng(0)
        best, best_away, nearest = None, np.inf, np.inf
        for spread in spreads:
            tried = np.clip(centre + rng.normal(0.0, spread, (SAMPLES, len(JOINTS))), self.low, self.high)
            tried[0] = centre
            shoulder, forward, down, lean, side, _ = self.torso(tried)
            beyond = np.linalg.norm(shoulder - wrist, axis=1) - (self.radius - inside)
            allowed = ((forward >= -1e-9) & (forward <= limits['torso_forward']) & (down <= limits['torso_down'])
                       & (lean >= -1e-9) & (lean <= limits['torso_pitch']) & (np.abs(side) <= limits['torso_roll']))
            if not allowed.any():
                continue
            nearest = min(nearest, float(beyond[allowed].min()))
            reaches = allowed & (beyond <= 0.0)
            if reaches.any():
                away = (((tried - self.ready) * self.cost) ** 2).sum(axis=1)
                if stay:
                    away = away + (((tried - start) * self.cost) ** 2).sum(axis=1)
                pick = int(np.flatnonzero(reaches)[np.argmin(away[reaches])])
                if away[pick] < best_away:
                    best, best_away = tried[pick], float(away[pick])
                centre = best
            else:  # head for where the torso gets closest
                centre = tried[int(np.flatnonzero(allowed)[np.argmin(beyond[allowed])])]
        if best is None:
            raise OutOfReach(
                f'out of reach: the target is {max(nearest, 0.0) * 100:.1f} cm beyond what the arm (elbow no '
                f'straighter than {np.degrees(limits["elbow_limit"]):.0f} deg) and the torso reach -- the torso may '
                f'bring the chest {limits["torso_forward"] * 100:.0f} cm forward and {limits["torso_down"] * 100:.0f} '
                f'cm down, lean {np.degrees(limits["torso_pitch"]):.0f} deg forward and '
                f'{np.degrees(limits["torso_roll"]):.0f} deg sideways and turn '
                f'{np.degrees(limits["torso_yaw"]):.0f} deg (settings: reach)')
        return best

    def share(self, positions, target):
        """The torso joints {name: rad} to move for the hand `target` (posture), or None when the
        torso is where it belongs already."""
        best = self.posture(target)
        now = np.array([positions[name] for name in JOINTS])
        if np.abs(best - now).max() < STILL:
            return None
        return {name: float(best[i]) for i, name in enumerate(JOINTS)}


class Follow:
    """The torso of an arm that follows a moving target (tracking mode): each tick it steps
    toward where it belongs for the target, no faster than `speed` (rad/s) a joint.

    Where it belongs takes 75 ms to work out, longer than a tick, so a thread of its own
    does that for the latest target given with ask().

    The arm's model has the torso compiled in at `model_posture` and cannot follow a moving
    torso (moving it takes 0.4 s). as_modelled() gives the transform that puts a pose in
    base where the model would see it with its torso there: a target handed to the model
    through it comes back as arm joints that are right for the torso as it really is
    (checked on plans: hand 0.1 mm from the target with the chest 9 cm away and turned).
    """

    def __init__(self, reach, model_posture, posture, speed):
        self.reach, self.speed = reach, speed
        self.commanded = np.array(posture, dtype=float)
        self.wanted = self.commanded.copy()
        self.rate = np.zeros(len(JOINTS))  # rad/s, how fast it is commanded now
        self.note = None  # why the torso cannot bring the target in reach, while it cannot
        self.model_chest = reach.chest(np.array([model_posture], dtype=float))[0]
        self.asked, self.wake, self.closing = None, threading.Event(), False
        self.thread = threading.Thread(target=self.work, daemon=True)
        self.thread.start()

    def out(self):
        return bool(np.abs(self.commanded - self.reach.ready).max() > STILL)

    def ask(self, target):
        """The hand target (4x4, base) the torso is to stand for."""
        self.asked = np.array(target, dtype=float)
        self.wake.set()

    def work(self):
        while True:
            self.wake.wait()
            self.wake.clear()
            if self.closing:
                return
            target = self.asked
            # A posture that still has the target between one and three margins inside the arm's
            # reach is kept: many postures serve one target as well, and taking a fresh one at
            # every ask kept the torso moving under a target that stood still (and the hand 1-2
            # cm off with every change). It is worked out anew when the arm's room runs out, when
            # the torso is further out than it need be, or when the target is back in the arm's reach.
            margin = self.reach.settings['torso_margin']
            wrist = self.reach.wrist(target)

            def beyond(posture):
                return float(np.linalg.norm(self.reach.torso(posture[None])[0][0] - wrist) - self.reach.radius)
            away = np.abs(self.wanted - self.reach.ready).max() > STILL
            if away and -3.0 * margin <= beyond(self.wanted) <= -margin and beyond(self.reach.ready) > -2.0 * margin:
                self.note = None
                continue
            try:
                self.wanted, self.note = self.reach.posture(target, out=away, near=self.wanted), None
            except OutOfReach as error:  # the torso stays where it is wanted last
                self.note = str(error)

    def step(self, period):
        """The torso posture (6,) to command this tick, `period` s after the last.

        Toward where it is wanted, slowing as it gets there, and its speed changed no faster
        than brings it from rest to `speed` in half a second: started and stopped at once, the
        hand was 27 mm off for a moment each time.
        """
        want = np.clip((self.wanted - self.commanded) * APPROACH, -self.speed, self.speed)
        change = 2.0 * self.speed * period
        self.rate = self.rate + np.clip(want - self.rate, -change, change)
        self.commanded = self.commanded + self.rate * period
        return self.commanded

    def as_modelled(self):
        """4x4: a pose in base -> the same pose as the arm's model sees it, its torso at model_posture."""
        return self.model_chest @ np.linalg.inv(self.reach.chest(self.commanded[None])[0])

    def close(self):
        self.closing = True
        self.wake.set()
        self.thread.join(timeout=2.0)
