import rclpy
from rclpy.node import Node
from robot_interfaces.srv import GetPlan

from robot_planner.planner import get_plan, fallback_plan

TARGET_SHAPE = {
    "pos_A": (0.2,  -1.6, 0.25),
    "pos_B": (0.2,  -1.6, 0.3),
    "pos_C": (0.2,  -1.6, 0.35),
}

class PlannerServer(Node):
    def __init__(self):
        super().__init__("planner_server")
        self._srv = self.create_service(GetPlan, "get_plan", self._handle)
        self.get_logger().info("Planner service ready at /get_plan")

    def _handle(self, request, response):
        block_positions = {}
        for name, x, y, z in zip(
            request.block_names,
            request.positions_x,
            request.positions_y,
            request.positions_z,
        ):
            block_positions[name] = (x, y, z)

        target_shape = {}
        for name, x, y, z in zip(
            request.target_names,
            request.targets_x,
            request.targets_y,
            request.targets_z,
        ):
            target_shape[name] = (x, y, z)

        # --- THE REVEAL: Print exactly what C++ sent us ---
        self.get_logger().info(f"Blocks received from C++: {list(block_positions.keys())}")
        self.get_logger().info(f"Targets received from C++: {list(target_shape.keys())}")

        predetermined_goals = {
            "block1": "pos_A",
            "block2": "pos_B",
            "block3": "pos_C",
        }

        try:
            plan = get_plan(block_positions, target_shape, goal_mapping=predetermined_goals)
        except Exception as exc:
            self.get_logger().warn(f"UP solver raised {exc} — using fallback")
            plan = []

        if not plan:
            self.get_logger().warn("Falling back to sequential plan")
            plan = fallback_plan(block_positions, target_shape, goal_mapping=predetermined_goals)

        place_actions = [a for a in plan if a.startswith("place")]

        self.get_logger().info(f"Returning {len(place_actions)} place actions")
        response.success = True
        response.message = f"{len(place_actions)} actions planned"
        response.plan_actions = place_actions
        return response

def main():
    rclpy.init()
    rclpy.spin(PlannerServer())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
