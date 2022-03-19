#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

// ROS2 basics
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// custom stuff
#include "tum_msgs/msg/tum_module_status.hpp"
#include "tum_msgs/msg/tum_state_estimate.hpp"
#include "tum_msgs/msg/tum_trajectory.hpp"
#include "tum_msgs/msg/tum_map_reference.hpp"
#include "tum_msgs/msg/tum_tpa_vdc_data.hpp"
#include "tum_msgs/msg/tum_request.hpp"
#include "tum_msgs/msg/tum_localization.hpp"
#include "tum_msgs/msg/tum_control_param.hpp"
// GPS stuff
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/dualantennaheading.hpp"
#include "novatel_oem7_msgs/msg/rawimu.hpp"
// vehicle stuff
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>

// ROS CAN stuff
#include <can_msgs/msg/frame.hpp>
#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

// state machine
#include "dbw_state_machine.hpp"

// Simulink stuff
#include "rtwtypes.h"
#include "rtw_modelmap.h"

// UDP stuff
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

using namespace std::chrono_literals;

// depending on control-only or SIL mode
#ifdef CONTROL
#include "controller_dev_py.h"
#include "controller_dev_py_types.h"
#else
#include "trajectory_planning_dev_py.h"
#include "trajectory_planning_dev_py_types.h"
#endif

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
  ID_BASE_TO_CAR                = 0x04B0,
  ID_DATA_TO_RC                 = 0x04BA,
  ID_TIRE_PRESSURE_FL           = 0x0528,
  ID_TIRE_PRESSURE_FR           = 0x0529,
  ID_TIRE_PRESSURE_RL           = 0x052A,
  ID_TIRE_PRESSURE_RR           = 0x052B,
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
  ID_TIRE_TEMP_FL1              = 0x052C,
  ID_TIRE_TEMP_FL2              = 0x052D,
  ID_TIRE_TEMP_FL3              = 0x052E,
  ID_TIRE_TEMP_FL4              = 0x052F,
  ID_TIRE_TEMP_FR1              = 0x0530,
  ID_TIRE_TEMP_FR2              = 0x0531,
  ID_TIRE_TEMP_FR3              = 0x0532,
  ID_TIRE_TEMP_FR4              = 0x0533,
  ID_TIRE_TEMP_RL1              = 0x0534,
  ID_TIRE_TEMP_RL2              = 0x0535,
  ID_TIRE_TEMP_RL3              = 0x0536,
  ID_TIRE_TEMP_RL4              = 0x0537,
  ID_TIRE_TEMP_RR1              = 0x0538,
  ID_TIRE_TEMP_RR2              = 0x0539,
  ID_TIRE_TEMP_RR3              = 0x053A,
  ID_TIRE_TEMP_RR4              = 0x053B,
  ID_DIAGNOSTIC_REPORT          = 0x053E,
};



class ControlHandler : public rclcpp::Node
{
  public:
    // class basics
    ControlHandler();
    ~ControlHandler();

    // initialization function (to be called once after startup)
    void init();
    // model step function (to be called once per cycle)
    void step();

  private:
    // model variables (depending on control-only or SIL mode)
    #ifdef CONTROL
    RT_MODEL_controller_dev_py_T *model;
    #else
    RT_MODEL_trajectory_planning_dev_py_T *model;
    #endif

    // software status and printing
    bool sil_mode;
    bool manual_mode;
    bool joystick_used;
    bool basestation_only;
    bool joystick_emergency_triggered;
    bool ct_sm_used; 
    bool race_control_used;
    bool enable_tubes;
    bool enable_mylaps;
    bool diagnostics_report_error;
    int status_objects_pointrcnn;
    int status_objects_clustering;
    int status_objects_radar;
    int status_prediction;
    int status_local_planner;
    int status_map_loc;
    int status_preprocessing;
    double timeout_objects_pointrcnn;
    double timeout_objects_clustering;
    double timeout_objects_radar;
    double timeout_prediction;
    double timeout_local_planner;
    double timeout_map_loc;
    double timeout_preprocessing;
    double timeout_trajectory_performance;
    double timeout_trajectory_emergency;
    double timeout_joystick;
    double timeout_imu_1;
    double timeout_imu_2;
    double timeout_wheelspeeds;
    double timeout_limit;
    double timeout_race_control;
    int old_joystick_counter;
    DbwStateMachine dbw_state_machine;
    int vehicle_id;
    int print_cnt;
    int error_code_old;
    int current_traj_id;
    int rolling_cnt;
    bool initialized;
    double current_global_s;
    int current_lap;
    int track_flag_ack; 
    int veh_flag_ack; 
    void print_status();

    // joystick handling
    int joystick_gear; 
    double joystick_steering; 
    double joystick_throttle; 
    double joystick_brake;
    bool reset_hardware;
    bool reset_hardware_old;
    bool manual_e_brake;
    bool driven_once;
    double battery_voltage;

    // tire temp handling
    double TireTemp_FL[16];
    double TireTemp_FR[16];
    double TireTemp_RL[16];
    double TireTemp_RR[16];

    // stuff for driving tube visualization
    io_service io_service;
    ip::udp::socket send_driving_tube_socket;
    ip::udp::endpoint send_driving_tube_endpoint;
    std::string send_driving_tube_ip;
    unsigned int send_driving_tube_port;
    double x_left_driving_tube_m[50];
    double y_left_driving_tube_m[50];
    double x_right_driving_tube_m[50];
    double y_right_driving_tube_m[50];
    // telemetry
    ip::udp::socket send_telemetry_socket;
    ip::udp::socket recv_basestation_socket;
    ip::udp::endpoint send_telemetry_endpoint;
    ip::udp::endpoint recv_basestation_endpoint;
    std::string send_telemetry_ip;
    unsigned int send_telemetry_port;
    std::string recv_basestation_ip;
    unsigned int recv_basestation_port;
    // TUM request handling 
    int tum_req_cnt_old;
    tum_msgs::msg::TUMRequest msg_tum_req;
    deep_orange_msgs::msg::RcToCt msg_rc_to_ct;

    // timing
    std::chrono::time_point<std::chrono::system_clock> tPrev;
    std::chrono::nanoseconds tControl_ns;
    std::chrono::nanoseconds tExec_ns;
    double dT; 

    rclcpp::TimerBase::SharedPtr timer_;

    // logging stuff
    void logging();
    std_msgs::msg::String log_line;
    std_msgs::msg::String log_line_slow;
    std_msgs::msg::String log_line_latency;
    double time;
    double old_logging_id;
    int N_DEBUG_SIGNALS;
    int N_DEBUG_SLOW_SIGNALS;
    //latency logging variables
    rclcpp::Clock ros_time;
    int latency_id;
    uint64_t latency_t_receive;
    uint64_t latency_t_send;
    int telemetry_cnt; 

    // Parameter handling
    int set_tum_parameter(std::string param, double value);
    double get_tum_parameter(std::string param);
    int find_parameter_idx(std::string param);
    int find_parameter_dt(std::string param);

    // communication interfacces (publishers) - Logger
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_slow;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_latency;

    // communication interfaces (publishers) - Software
    rclcpp::Publisher<tum_msgs::msg::TUMModuleStatus>::SharedPtr pub_status;
    rclcpp::Publisher<tum_msgs::msg::TUMStateEstimate>::SharedPtr pub_state_estimate;
    rclcpp::Publisher<tum_msgs::msg::TUMTpaVdcData>::SharedPtr pub_tpa_data;
    rclcpp::Publisher<tum_msgs::msg::TUMRequest>::SharedPtr pub_tum_request;
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr pub_race_control;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_manual_mode;

    // communication interfaces (subscribers) - Software 
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_objects_pointrcnn;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_objects_clustering;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_objects_radar;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_prediction;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_local_planner;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_map_loc;
    rclcpp::Subscription<tum_msgs::msg::TUMModuleStatus>::SharedPtr sub_status_preprocessing;
    rclcpp::Subscription<tum_msgs::msg::TUMTrajectory>::SharedPtr sub_trajectory;
    rclcpp::Subscription<tum_msgs::msg::TUMTrajectory>::SharedPtr sub_em_trajectory;
    rclcpp::Subscription<tum_msgs::msg::TUMMapReference>::SharedPtr sub_map_ref;
    rclcpp::Subscription<tum_msgs::msg::TUMLocalization>::SharedPtr sub_lidar_loc;
    rclcpp::Subscription<tum_msgs::msg::TUMControlParam>::SharedPtr sub_control_param;
    // communication interfaces (subscribers) - Sensors
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr sub_gps_1;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr sub_vel_1;
    rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr sub_imu_1;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr sub_gps_2;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr sub_vel_2;
    rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr sub_imu_2;
    rclcpp::Subscription<novatel_oem7_msgs::msg::DUALANTENNAHEADING>::SharedPtr sub_dual_antenna_1;
    rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr sub_steering;
    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr sub_wheelspeeds;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr sub_powertrain_signals;
    rclcpp::Subscription<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr sub_brake;
    // communication interfaces (subscribers) - Joystick and Race control
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr sub_joystick_command;
    rclcpp::Subscription<deep_orange_msgs::msg::MiscReport>::SharedPtr sub_misc_report;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_hardware;

    // CAN publisher and subscriber
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_from_simu_can_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_to_simu_can_;

    NewEagle::Dbc dbc_av21_;                       // dbc class for can frame reading and writing
    std::string dbc_av21_filename_;                // path to .dbc for can1

    // callbacks to write incoming data - Software
    void on_sub_status_objects_pointrcnn(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_objects_clustering(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_objects_radar(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_prediction(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_local_planner(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_map_loc(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_status_preprocessing(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg);
    void on_sub_trajectory(const tum_msgs::msg::TUMTrajectory::SharedPtr msg);
    void on_sub_em_trajectory(const tum_msgs::msg::TUMTrajectory::SharedPtr msg);
    void on_sub_map_ref(const tum_msgs::msg::TUMMapReference::SharedPtr msg);
    void on_sub_lidar_loc(const tum_msgs::msg::TUMLocalization::SharedPtr msg);
    void on_sub_control_param(const tum_msgs::msg::TUMControlParam::SharedPtr msg);
    // callbacks to write incoming data - Sensors
    void on_sub_gps_1(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg);
    void on_sub_vel_1(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg);
    void on_sub_imu_1(const novatel_oem7_msgs::msg::RAWIMU::SharedPtr msg);
    void on_sub_gps_2(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg);
    void on_sub_vel_2(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg);
    void on_sub_imu_2(const novatel_oem7_msgs::msg::RAWIMU::SharedPtr msg);
    void on_sub_dual_antenna_1(const novatel_oem7_msgs::msg::DUALANTENNAHEADING::SharedPtr msg);
    // callbacks to write incoming data
    void on_sub_reset_hardware(const std_msgs::msg::Bool::SharedPtr msg);

    // callbacks to writ incoming data - CAN
    void on_sub_from_can(const can_msgs::msg::Frame::SharedPtr msg);

    // Transformation to TCS
    std::vector<double> ControlHandler::llh_to_ecf(std::vector<double> llh);
    void ControlHandler::get_dircos_forward(double A, int MatrixFlavor, std::vector<std::vector<double>> &DC);
    std::vector<double> ControlHandler::ecf_to_uvw(std::vector<double> ecf);
    void ControlHandler::uvw_to_tcs(std::vector<double> uvw, std::vector<double> &tcs);
    void ControlHandler::llh_to_tcs(std::vector<double> llh, std::vector<double> &tcs);

    //Coordinates
    std::vector<double> origin;
    std::vector<double> origin_conv; 

    // CAN specific methods
    void decode_can_msg();

    can_msgs::msg::Frame::SharedPtr in_can_msg_{nullptr};

    void send_steering();
    void send_accel_pedal();
    void send_brake();
    void send_gear();
    void send_ct_report();

    void telemetry();

    double steering;
    double acc_pedal;
    double brake;
    uint8_t gear_cmd;
    uint8_t ct_state;
    uint8_t old_rc_counter;

    //misc_report variables
    uint8_t sys_state;

    std::string gps_origin;
};
