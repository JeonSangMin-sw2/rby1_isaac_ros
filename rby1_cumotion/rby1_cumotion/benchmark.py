"""Repeat one planning query N times and report statistics, not a single number.

Two invariants are checked before the first measurement, because breaking either
one produces numbers that look real and are not:

  1. exactly one move_group / cumotion_planner_node / ros2_control_node
  2. every controller the group needs is active

A stale launch leaves a second controller_manager competing for the same names;
plans then succeed while their results never reach the client, and the run reads
as a planner failure. Enforcing this here is what makes the output quotable.
"""

import json
import math
from pathlib import Path
import statistics
import time

import rclpy

from rby1_cumotion.model import active_record
from rby1_cumotion.planning import PlanningClient, running_nodes, run_node


class Benchmark(PlanningClient):
    def __init__(self):
        super().__init__('rby1_cumotion_benchmark', extra={
            'runs': 10, 'settle': 0.5, 'enforce_singletons': True, 'output': '',
            'hardware': '',  # empty: whatever the running cuMotion launch was started with
        })

    def expected_nodes(self):
        """How many of each node a healthy run of this pipeline and hardware has.

        The cuMotion planner is only launched for the cuMotion pipeline, so
        demanding one during an OMPL benchmark would reject a clean environment.
        With hardware:=driver there must be no ros2_control_node at all: its
        hardware plugin would claim the robot and the driver would then refuse
        follow_joint_trajectory. A second copy of anything is always wrong.
        """
        cumotion = self.param('pipeline') == 'isaac_ros_cumotion'
        driver = self.param('hardware') == 'driver'
        return {
            'move_group': (1, 1),
            'cumotion_planner_node': (1, 1) if cumotion else (0, 1),
            'ros2_control_node': (0, 0) if driver else (1, 1),
        }

    def check_environment(self):
        """Fail loudly before measuring; a clean environment is part of the result."""
        if self.param('enforce_singletons'):
            expected = self.expected_nodes()
            found = running_nodes()
            wrong = {name: entries for name, entries in found.items()
                     if not expected[name][0] <= len(entries) <= expected[name][1]}
            if wrong:
                lines = [f'{name}: {len(entries)} running, expected '
                         f'{expected[name][0]}..{expected[name][1]}'
                         for name, entries in wrong.items()]
                detail = '\n'.join(entry for entries in wrong.values() for entry in entries)
                raise RuntimeError(
                    'Measurement environment is not clean, so the numbers would be meaningless.\n'
                    f'{"; ".join(lines)}.\n{detail}\n'
                    'Reap the previous launch by process group, confirm none remain, then relaunch.')
        if self.param('hardware') == 'driver':
            # No controllers here; the robot behind MoveIt is the driver's joint
            # stream. A stale or missing one would plan from a made-up state.
            self.fresh_snapshot()
            return ['driver joint states live']
        # With mock hardware the robot is ros2_control; its controllers must run.
        active = self.active_controllers()
        missing = [name for name in self.metadata['controllers'] if name not in active]
        if missing:
            raise RuntimeError(f'Controllers {missing} are not active (active: {active}). '
                               'MoveIt would answer without a robot behind it.')
        return active

    def param(self, name):
        if name == 'hardware' and not self.get_parameter('hardware').value:
            record = active_record()
            return record.get('hardware', 'mock') if record else 'mock'
        return super().param(name)

    def run(self):
        if self.param('hardware') not in ('mock', 'driver'):
            raise ValueError('hardware must be mock or driver, matching the cuMotion launch')
        runs = int(self.param('runs'))
        if runs < 1:
            raise ValueError('runs must be at least 1')
        if runs < 10:
            self.get_logger().warn(f'runs={runs} is below the 10 this project requires before '
                                   'quoting a planning time or a success rate')
        offset = self.validate_parameters()
        self.await_planner()
        active = self.check_environment()
        self.get_logger().info(f'environment clean ({self.param("hardware")}): expected nodes '
                               f'running once each; {", ".join(active)}')
        # Warm up outside the measured set: the first plan captures CUDA graphs
        # and would otherwise dominate the statistics it is not representative of.
        if self.param('warmup'):
            self.warm()

        samples, failures = [], []
        for attempt in range(1, runs + 1):
            try:
                result = self.measure(offset)
                result.pop('trajectory')
                samples.append(result)
                self.get_logger().info(f'run {attempt}/{runs} planner_time={result["planner_time"]:.3f}s')
            except Exception as error:
                failures.append({'run': attempt, 'error': str(error)})
                self.get_logger().warn(f'run {attempt}/{runs} failed: {error}')
            deadline = time.monotonic() + self.param('settle')
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

        report = self.summarise(runs, samples, failures, active)
        if self.param('output'):
            Path(self.param('output')).write_text(json.dumps(report, indent=2) + '\n')
            self.get_logger().info(f'report written to {self.param("output")}')
        if not samples:
            raise RuntimeError(f'BENCHMARK_FAIL: 0/{runs} plans succeeded')

    def summarise(self, runs, samples, failures, active):
        report = {'pipeline': self.param('pipeline'), 'group': self.metadata['group'],
                  'model': self.metadata['model'], 'sphere_count': self.metadata['sphere_count'],
                  'active_joints': len(self.metadata['active_joints']),
                  'active_controllers': active, 'runs': runs,
                  'successes': len(samples), 'failures': failures,
                  'offset_xyz': list(self.param('offset_xyz'))}
        for field in ('planner_time', 'wall_time', 'points', 'duration'):
            values = [sample[field] for sample in samples]
            report[field] = {
                'median': statistics.median(values) if values else math.nan,
                'min': min(values) if values else math.nan,
                'max': max(values) if values else math.nan,
            }
        planner = report['planner_time']
        self.get_logger().info(
            f'BENCHMARK pipeline={report["pipeline"]} group={report["group"]} '
            f'model={report["model"]} spheres={report["sphere_count"]} '
            f'joints={report["active_joints"]} success={len(samples)}/{runs} '
            f'planner_time median={planner["median"]:.3f}s '
            f'range={planner["min"]:.3f}-{planner["max"]:.3f}s')
        return report


def main(args=None):
    run_node(Benchmark)


if __name__ == '__main__':
    main()
