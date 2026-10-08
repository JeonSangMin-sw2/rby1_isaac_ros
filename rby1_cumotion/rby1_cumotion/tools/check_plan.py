"""Plan a small Cartesian displacement and report it. Never moves the robot.

This node deliberately holds no execution client and never sets plan_only to
false, so pointing it at a live robot cannot produce motion. `target_executor`
is the node that executes.
"""

from rby1_cumotion.planning import PlanningClient, run_node


class CheckPlan(PlanningClient):
    def __init__(self):
        super().__init__('rby1_cumotion_check_plan')

    def run(self):
        offset = self.prepare()
        result = self.measure(offset)
        self.get_logger().info(
            f'PLAN_OK pipeline={result["pipeline"]} group={result["group"]} '
            f'planner_time={result["planner_time"]:.3f}s wall_time={result["wall_time"]:.3f}s '
            f'points={result["points"]} duration={result["duration"]:.3f}s')


def main(args=None):
    run_node(CheckPlan)


if __name__ == '__main__':
    main()
