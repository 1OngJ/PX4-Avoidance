#include <chrono>
#include <local_planner/local_planner_node.h>

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avoidance::LocalPlannerNode>());

  rclcpp::shutdown();
  return 0;
}
