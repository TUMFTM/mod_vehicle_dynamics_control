#include "ControlHandler.h"
#include <cstdlib>

const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto myControlHandler = std::make_shared<ControlHandler>();
  myControlHandler->init();
  rclcpp::spin(myControlHandler);
  rclcpp::shutdown();
  return 0;
}
