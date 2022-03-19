#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <stdlib.h>

// ROS2 basics
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

// messages
#include <can_msgs/msg/frame.hpp>

#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>

#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>

#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>

// UDP stuff
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

// ROS CAN stuff
#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

static const double pi = 3.14159265359;

enum
{
  // USED
  ID_MISC_REPORT_DO             = 0x0518,
  ID_PT_REPORT_1                = 0x053C,
  // EDITED
  ID_BRAKE_CMD                  = 0x0578, //0x2F04,
  ID_ACCELERATOR_PEDAL_CMD      = 0x0579, //0x2F01
  ID_STEERING_CMD               = 0x057A, //0x2F03
  ID_GEAR_CMD                   = 0x057B, //0x2F05
  ID_CT_REPORT                  = 0x057C,
  ID_TIRE_PRESSURE_FL           = 0x0528,
  ID_TIRE_PRESSURE_FR           = 0x0529,
  ID_TIRE_PRESSURE_RL           = 0x052A,
  ID_TIRE_PRESSURE_RR           = 0x052B,
  ID_BASE_TO_CAR                = 0x04B0,
  // UNUSED
  ID_BRAKE_REPORT               = 0x1F04,
  ID_ACCEL_PEDAL_REPORT         = 0x1F02,
  ID_STEERING_REPORT            = 0x1F03,
  ID_GEAR_REPORT                = 0x1F05,
  ID_REPORT_WHEEL_SPEED         = 0x1F0B,
  ID_REPORT_IMU                 = 0x1F0A,
  ID_REPORT_TIRE_PRESSURE       = 0x1f07,
  ID_REPORT_SURROUND            = 0x1f10,
  ID_VIN                        = 0x1F08,
  ID_REPORT_DRIVER_INPUT        = 0x1F0F,
  ID_REPORT_WHEEL_POSITION      = 0x1F06,
  ID_MISC_REPORT                = 0x1F01,
  ID_LOW_VOLTAGE_SYSTEM_REPORT  = 0x1F11,
  ID_BRAKE_2_REPORT             = 0x1F12,
  ID_STEERING_2_REPORT          = 0x1F13,
  ID_OTHER_ACTUATORS_REPORT     = 0x1F14,
  ID_FAULT_ACTION_REPORT        = 0x1F15,
  ID_HMI_GLOBAL_ENABLE_REPORT   = 0x3f01,
  ID_TEST                       = 0x0718,
  ID_WHEEL_SPEED_REPORT_DO      = 0x0514,
  ID_BRAKE_PRESSURE_REPORT_DO   = 0x0515,
  ID_ACCELERATOR_REPORT_DO      = 0x0516,
  ID_STEERING_REPORT_DO         = 0x0517,
  ID_RC_TO_CT                   = 0x051B,
  ID_WHEEL_STRAIN_GAUGE         = 0x051E,
  ID_WHEEL_POTENTIOMETER        = 0x051F,
  ID_VEL_ACC_HIL                = 0x05DD,
  ID_POSITION_HEADING_HIL       = 0x05DC,
  ID_PT_REPORT_2                = 0x053D,
};

class RaptorEmulation : public rclcpp::Node
{
  public:
    // class basics
    RaptorEmulation();
    ~RaptorEmulation();

  private:
    // communication interfaces
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_force;
    rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr sub_rc_to_ct;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_from_simu_can_;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_to_simu_can_;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_to_simu_can_1;

    NewEagle::Dbc dbc_av21_;                       // dbc class for can frame reading and writing
    std::string dbc_av21_filename_;                // path to .dbc for can1

    // callbacks to write incoming data
    void on_sub_force(const std_msgs::msg::Float32::SharedPtr msg);
    void on_sub_from_can(const can_msgs::msg::Frame::SharedPtr msg);
    void on_sub_rc(const deep_orange_msgs::msg::RcToCt::SharedPtr msg);

    void send_misc_report_do();
    void send_pt_report();
    void send_pt_2_report();
    void send_wheel_report();
    void send_brake_report();
    void send_steering();
    void send_throttle();
    void send_base_to_car();

    void decode_can_msg();

    can_msgs::msg::Frame::SharedPtr in_can_msg_{nullptr};

    float steering_ratio;
    float front_wheel_radius;
    float rear_wheel_radius;

    double force_cmd_; 
    double brake_cmd_;
    double steering_cmd_;
    double accelerator_pedal_cmd_;
    int gear_cmd_;
    bool purple_flag_;
    uint8_t ct_state_;
    uint8_t vs_state_ack_;
    uint8_t track_cond_ack_;
    uint8_t sys_state_;
    int veh_flag;
    int track_flag;


    double wheel_fl_;
    double wheel_fr_;
    double wheel_rl_;
    double wheel_rr_;
    double steering_wheel_angle_;
    int steering_counter_;
    double front_brake_pressure_;
    double rear_brake_pressure_;
    int brake_counter_;
    double pedal_output_;
    int throttle_counter_;
    double gear_data_;
    double engine_rpm_;

    // timer to handler publishers and vehicle interface
    rclcpp::TimerBase::SharedPtr timer_;
    void pub_cmd_udp();

    // udp stuff
    io_service io_service;
    ip::udp::socket send_cmd_socket;
    ip::udp::socket recv_sensor_data_socket;
    ip::udp::socket recv_raptor_system_state_socket;
    ip::udp::socket send_cmd_raptor_system_socket;
    ip::udp::endpoint send_cmd_endpoint;
    ip::udp::endpoint recv_sensor_data_endpoint;
    ip::udp::endpoint recv_raptor_system_state_endpoint;
    ip::udp::endpoint send_cmd_raptor_system_endpoint;
    std::string send_cmd_ip;
    unsigned int send_cmd_port;
    std::string recv_sensor_data_ip;
    unsigned int recv_sensor_data_port;
    std::string recv_raptor_system_state_ip;
    unsigned int recv_raptor_system_state_port;
    std::string send_cmd_raptor_system_ip;
    unsigned int send_cmd_raptor_system_port;
};
