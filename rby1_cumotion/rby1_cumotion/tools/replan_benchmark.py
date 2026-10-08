"""How long a replan takes with obstacles on the arm's way: N runs per case, statistics.

  python3 -m rby1_cumotion.tools.replan_benchmark [--model m_1_2] [--group right_arm] [--runs 10]
      [--obstacles 0 1 2 3] [--speeds 0 0.1 0.3] [--settings 4,24,100 2,16,50] [--output report.json]

No robot and no ROS graph: cuRobo in this process, built the way target_executor
builds its replanner (avoidance.Avoider), so the numbers are those of the replan
that takes over a running trajectory -- not of the first plan, which goes through
MoveIt and the cuMotion planner node (see benchmark.py for that).

A case: the arm is 30% into an unobstructed move of its hand by `--move`, at the
speed it has there, when `n` boxes stand where the arm would pass on the rest of that
move -- every one is in the way -- but clear of the arm where it is and where it ends
(a box inside the arm at the target leaves no path at all: IK_FAIL). A box with a
speed is coming across: it is taken half a second before it gets there, and planned
around as the executor does it, with copies of it along the next second of its way
(avoid.sweep_time). Each run shifts the boxes by up to 1 cm, so the world is new every
time, as it is when a scene changes.

Reported per case: replans that succeeded, paths that are clear of the boxes (checked
apart from the planner), and the time of the world update plus the replan. A setting
is replan_seeds,replan_steps,replan_iters (config/cumotion.yaml, avoid:).

Needs the container (cuRobo). Does not move anything.
"""

import argparse
import json
import os
from pathlib import Path
import statistics
import time

import numpy as np

from rby1_cumotion import planner_params, settings
from rby1_cumotion.executor.avoidance import Avoider, BOX, clear_of, surface_distance, swept
from rby1_cumotion.model import forward_kinematics, load_model

START_AT = 0.3      # of the unobstructed move: where the arm is when the boxes appear
BOX_EDGE = 0.04     # m
JITTER = 0.01       # m, how far a run shifts each box
CROSSES_IN = 0.5    # s until a moving box gets to its place
CLEARANCE = 0.02    # m a box keeps from the arm where it is and where it ends
APART = 0.08        # m between boxes
# Where along the rest of the move the boxes stand (0 the arm now, 1 the target), by how many there are.
PLACES = {0: [], 1: [0.5], 2: [0.35, 0.65], 3: [0.25, 0.5, 0.75]}
OFFSETS = (0.0, -0.04, 0.04, -0.08, 0.08, -0.12, 0.12)  # m off the hand's path, tried in base x and z


def box(name, centre):
    pose = np.eye(4)
    pose[:3, 3] = centre
    return {'name': name, 'kind': BOX, 'dims': [BOX_EDGE] * 3, 'pose': pose}


def gap(obj, spheres):
    """How far the box's surface is from the nearest of the arm's spheres (negative: inside)."""
    return float((surface_distance(spheres[:, :3], obj) - spheres[:, 3]).min())


class Case:
    """One arm state, target and box layout; see the module text."""

    def __init__(self, avoider, metadata, root, move, sweep_time):
        self.avoider, self.root, self.sweep_time = avoider, root, sweep_time
        self.names = list(metadata['active_joints'])
        self.tool = metadata['tool_frame']
        self.posture = dict(metadata['default_positions'])
        for arm, pose in settings.load(None, 'robot')['ready_pose'].items():
            if f'{arm}_0' in self.names:
                self.posture.update({f'{arm}_{i}': value for i, value in enumerate(pose)})
        rest = np.array([self.posture[name] for name in self.names])
        self.target = self.hand(rest)
        self.target[:3, 3] += move
        avoider.plan_world([])
        free, status = avoider.replan(rest, np.zeros(len(rest)), self.target, self.names)
        if free is None:
            raise RuntimeError(f'no unobstructed path for the move {list(move)}: {status}')
        index = max(1, int(START_AT * (len(free) - 1)))
        self.q = free[index]
        self.v = (free[index + 1] - free[index - 1]) / (2 * 0.025)  # Avoider's interpolation_dt
        rest_of_move = free[index:]
        self.free_points = len(rest_of_move)
        self.way = np.array([self.hand(row)[:3, 3] for row in rest_of_move])
        self.arm = [self.live(row) for row in rest_of_move]
        self.found = {}  # box places by count

    def hand(self, row):
        positions = dict(self.posture, **dict(zip(self.names, map(float, row))))
        return forward_kinematics(self.root, positions, self.tool)

    def live(self, row):
        spheres = self.avoider.spheres(row, self.names)
        return spheres[spheres[:, 3] > 0]

    def in_the_way(self, point):
        """A box here is clear of the arm now and at the target, and the arm would pass through it."""
        obj = box('probe', point)
        return (gap(obj, self.arm[0]) >= CLEARANCE and gap(obj, self.arm[-1]) >= CLEARANCE
                and min(gap(obj, spheres) for spheres in self.arm[1:-1]) < 0.0)

    def near(self, place, taken):
        """A point for a box that is in the way: at `place` along the rest of the move, or the nearest that is."""
        for shift in sorted(range(-len(self.way), len(self.way)), key=abs):
            at = int(place * (len(self.way) - 1)) + shift
            if not 0 < at < len(self.way) - 1:
                continue
            for dx in OFFSETS:
                for dz in OFFSETS:
                    point = self.way[at] + [dx, 0.0, dz]
                    if all(np.linalg.norm(point - other) >= APART for other in taken) and self.in_the_way(point):
                        return point
        raise RuntimeError(f'no place for a box at {place} of the move that is in the way: change --move')

    def places(self, count):
        if count not in self.found:
            chosen = []
            for place in PLACES[count]:
                chosen.append(self.near(place, chosen))
            self.found[count] = chosen
        return self.found[count]

    def boxes(self, count, speed, rng):
        """The boxes of one run, as the planner is given them: with the copies ahead of moving ones."""
        objects, velocities = [], {}
        for index, place in enumerate(self.places(count)):
            point = place + rng.uniform(-JITTER, JITTER, 3)
            name = f'box{index}'
            if speed > 0:
                # Coming from the side that keeps it off the arm where it is and where it ends.
                clear = [d for d in ([0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0]) if all(
                    gap(box('probe', place + np.array(d) * speed * t), spheres) >= 0
                    for t in np.linspace(-CROSSES_IN, self.sweep_time - CROSSES_IN, 5)
                    for spheres in (self.arm[0], self.arm[-1]))]
                velocity = np.array(clear[0] if clear else [0, 0, 1], dtype=float) * speed
                point = point - velocity * CROSSES_IN
                velocities[name] = velocity
            objects.append(box(name, point))
        standing = [obj for obj in objects if obj['name'] not in velocities]
        ahead = [obj for obj in swept(objects, velocities, self.sweep_time)
                 if obj['name'] not in {o['name'] for o in standing}]
        return standing + clear_of(ahead, self.avoider.spheres(self.q, self.names))


def measure(avoider, case, count, speed, runs, rng):
    times, failed_times, points, statuses, clear = [], [], [], [], 0
    for _ in range(runs):
        planned = case.boxes(count, speed, rng)
        began = time.monotonic()
        avoider.plan_world(planned)
        try:
            path, status = avoider.replan(case.q, case.v, case.target, case.names)
        except ValueError as error:  # a path that does not pass its start state
            path, status = None, str(error)
        took = time.monotonic() - began
        if path is None:
            statuses.append(status)
            failed_times.append(took)
            continue
        times.append(took)
        points.append(len(path))
        avoider.update(planned, planned)
        clear += avoider.first_hit(path, case.names)[0] is None
    row = {'obstacles': count, 'speed': speed, 'boxes_planned': len(planned), 'runs': runs,
           'successes': len(times), 'clear': clear,
           'failures': {status: statuses.count(status) for status in sorted(set(statuses))}}
    for name, values in (('time', times), ('failed_time', failed_times), ('points', points)):
        row[name] = {'median': statistics.median(values), 'min': min(values), 'max': max(values)} if values else None
    return row


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default='m_1_2')
    parser.add_argument('--group', default='right_arm')
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--obstacles', type=int, nargs='+', default=[0, 1, 2, 3], choices=sorted(PLACES))
    parser.add_argument('--speeds', type=float, nargs='+', default=[0.0, 0.1, 0.3], help='m/s of the boxes')
    parser.add_argument('--move', type=float, nargs=3, default=[0.0, 0.30, 0.10], help='the hand\'s move, m in base')
    parser.add_argument('--settings', nargs='+', default=[], metavar='SEEDS,STEPS,ITERS',
                        help='replan settings to compare (default: those of the settings file)')
    parser.add_argument('--output', default='')
    options = parser.parse_args(args)
    if options.runs < 10:
        print(f'runs={options.runs} is below the 10 this project requires before quoting a time or a success rate')

    config = settings.load(None)
    planner = planner_params.parse(lambda name: planner_params.FROM_CONFIG,
                                   planner_params.load_config(str(settings.default_path())))
    metadata, root = load_model(Path(os.environ['RBY1_BUNDLES']) / options.model, options.group)
    chosen = [tuple(int(v) for v in text.split(',')) for text in options.settings] or [
        (config['avoid']['replan_seeds'], config['avoid']['replan_steps'], config['avoid']['replan_iters'])]
    report = {'model': options.model, 'group': options.group, 'move': options.move, 'box_edge': BOX_EDGE,
              'settings': []}
    for seeds, steps, iters in chosen:
        began = time.monotonic()
        avoider = Avoider(metadata, root, planner, ground=planner['add_ground_plane'],
                          seeds=seeds, steps=steps, iters=iters)
        case = Case(avoider, metadata, root, np.array(options.move), config['avoid']['sweep_time'])
        for _ in range(3):  # a moving start and a new goal run code the warmup did not
            avoider.replan(case.q, case.v, case.target, case.names)
        print(f'\nreplan_seeds={seeds} replan_steps={steps} replan_iters={iters} '
              f'(built and warmed in {time.monotonic() - began:.1f} s; the unobstructed rest of the move is '
              f'{case.free_points} points)')
        rows = []
        for count in options.obstacles:
            for speed in (options.speeds if count else [0.0]):
                row = measure(avoider, case, count, speed, options.runs, np.random.default_rng(count * 100 + 7))
                rows.append(row)
                took, lost = row['time'], row['failed_time']
                print(f'  obstacles={count} speed={speed:.2f} m/s ({row["boxes_planned"]} boxes planned around): '
                      f'success {row["successes"]}/{row["runs"]}, clear {row["clear"]}/{row["successes"]}'
                      + (f', time median {took["median"] * 1000:.0f} ms (range {took["min"] * 1000:.0f}-'
                         f'{took["max"] * 1000:.0f}), points median {row["points"]["median"]:.0f}' if took else '')
                      + (f'; failed {row["failures"]} in median {lost["median"] * 1000:.0f} ms (up to '
                         f'{lost["max"] * 1000:.0f})' if lost else ''))
        report['settings'].append({'replan_seeds': seeds, 'replan_steps': steps, 'replan_iters': iters,
                                   'cases': rows})
        del avoider, case
        import torch
        torch.cuda.empty_cache()
    if options.output:
        Path(options.output).write_text(json.dumps(report, indent=2) + '\n')
        print(f'report written to {options.output}')


if __name__ == '__main__':
    main()
