#include "RaptorEmulation.h"

int main(int argc, char * argv[])
{
  // initialize ros2 stuff
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaptorEmulation>());
  rclcpp::shutdown();
  return 0;
}
