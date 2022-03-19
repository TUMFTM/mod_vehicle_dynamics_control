#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <iostream>
#include <fstream>

using std::placeholders::_1;

class RealTimeLogger : public rclcpp::Node
{
  public:
    RealTimeLogger()
    : Node("RealTimeLogger")
    {
        sub_debug = this->create_subscription<std_msgs::msg::String>(
            "/mod_control/debug", 100, std::bind(&RealTimeLogger::write_debug, this, _1));
        sub_debug_slow = this->create_subscription<std_msgs::msg::String>(
            "/mod_control/debug_slow", 100, std::bind(&RealTimeLogger::write_debug_slow, this, _1));
        sub_debug_latency = this->create_subscription<std_msgs::msg::String>(
            "/mod_control/debug_latency", 100, std::bind(&RealTimeLogger::write_debug_latency, this, _1));
        // create debug folder
        auto now = std::chrono::system_clock::now();
        auto now_c  = std::chrono::system_clock::to_time_t(now);
        char date_c[11];
        std::strftime(date_c, sizeof(date_c), "%Y_%m_%d", std::localtime(&now_c));
        auto date_str = std::string(date_c);
        char time_c[9];
        std::strftime(time_c, sizeof(time_c), "%H_%M_%S", std::localtime(&now_c));
        auto time_str = std::string(time_c);
        std::string log_path = "logs/" + date_str + "/" + time_str;
        std::string create_folder_cmd = "mkdir -p " + log_path; 
        system(create_folder_cmd.c_str());
        logging_file.open(log_path + "/rl_log.txt");
        logging_file_slow.open(log_path + "/rl_log_slow.txt");
        logging_file_latency.open(log_path + "/latency_logs.csv");
        RCLCPP_INFO(this->get_logger(), "Logger launched and logs to folder: %s", log_path.c_str());
        // copy debug files and version file to log folder 
        std::string copy_debug_file_cmd = "cp src/mod_control/control/debug.txt " + log_path + "/debug.txt";
        std::string copy_debug_slow_file_cmd = "cp src/mod_control/control/debug_slow.txt " + log_path + "/debug_slow.txt";
        std::string copy_version_file_cmd = "cp src/mod_control/control/version.txt " + log_path + "/version.txt";
        system(copy_debug_file_cmd.c_str());
        system(copy_debug_slow_file_cmd.c_str());
        system(copy_version_file_cmd.c_str());
    }
    ~RealTimeLogger()
    {
        logging_file.close();
        logging_file_slow.close();
    }

  private:
    void write_debug(const std_msgs::msg::String::SharedPtr msg)
    {
      logging_file << (msg->data);
    }
    void write_debug_slow(const std_msgs::msg::String::SharedPtr msg)
    {
      logging_file_slow << (msg->data);
    }
    void write_debug_latency(const std_msgs::msg::String::SharedPtr msg)
    {
      logging_file_latency << (msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_debug;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_debug_slow;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_debug_latency;
    std::ofstream logging_file;
    std::ofstream logging_file_slow;
    std::ofstream logging_file_latency;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealTimeLogger>());
  rclcpp::shutdown();
  return 0;
}
