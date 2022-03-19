#include "ControlHandler.h"
#include <math.h>
#include <chrono>
#include <thread>

ControlHandler::ControlHandler() : Node("ControlHandler"), send_driving_tube_socket(io_service), send_telemetry_socket(io_service), recv_basestation_socket(io_service)
{
  // define parameters 
  this->declare_parameter<bool>("sil_mode", 0);
  this->get_parameter("sil_mode", sil_mode);
  this->declare_parameter<int>("vehicle_id", 1);
  this->get_parameter("vehicle_id", vehicle_id);
  this->declare_parameter<double>("timeout_limit", 0.5);
  this->get_parameter("timeout_limit", timeout_limit);
  this->declare_parameter<bool>("manual_mode", 0);
  this->get_parameter("manual_mode", manual_mode);
  this->declare_parameter<bool>("joystick_used", 0);
  this->get_parameter("joystick_used", joystick_used);
  this->declare_parameter<bool>("ct_sm_used", 0);
  this->get_parameter("ct_sm_used", ct_sm_used);
  this->declare_parameter<bool>("race_control_used", 0);
  this->get_parameter("race_control_used", race_control_used);
  this->declare_parameter<std::string>("gps_origin", "HIL");
  this->get_parameter("gps_origin", gps_origin);
  this->declare_parameter<bool>("enable_tubes", 1);
  this->get_parameter("enable_tubes", enable_tubes);
  this->declare_parameter<bool>("basestation_only", 0);
  this->get_parameter("basestation_only", basestation_only);

  dbc_av21_filename_ = "/dev_ws/src/mod_control/control/params/CAN1-INDY-V9.dbc";
  dbc_av21_ = NewEagle::DbcBuilder().NewDbc(dbc_av21_filename_.c_str());

  // this ifdef construction are necessary to load different models from simulink codegen
  // it can't be replaced with runtime decisions in c++. 
  #ifdef CONTROL
  model = controller_dev_py();
  controller_dev_py_initialize(model);
  RCLCPP_INFO(this->get_logger(), "Running in control mode ...");
  #else
  model = trajectory_planning_dev_py();
  trajectory_planning_dev_py_initialize(model);
  RCLCPP_INFO(this->get_logger(), "Running in SIL mode ...");
  #endif
  time = 0;
  old_logging_id = 0;
  ros_time = rclcpp::Clock::Clock();
  latency_id = -1;
  latency_t_receive = 0;
  latency_t_send = 0;
  telemetry_cnt = 0;

  // initialize software status
  status_map_loc = -1;
  status_objects_pointrcnn = -1;
  status_objects_clustering = -1;
  status_objects_radar = -1;
  status_local_planner = -1;
  status_prediction = -1;
  status_preprocessing = -1;
  print_cnt = 0;
  error_code_old = 0;
  current_traj_id = -1;
  initialized = false;
  current_global_s = -1;
  current_lap = -1;
  old_rc_counter = 0;
  // initialize timeouts with limit value
  timeout_objects_pointrcnn = timeout_limit;
  timeout_objects_clustering = timeout_limit;
  timeout_objects_radar = timeout_limit;
  timeout_prediction = timeout_limit;
  timeout_local_planner = timeout_limit;
  timeout_map_loc = timeout_limit;
  timeout_preprocessing = timeout_limit;
  timeout_trajectory_performance = timeout_limit;
  timeout_trajectory_emergency = timeout_limit;
  timeout_joystick = 0;
  timeout_imu_1 = 0.5*timeout_limit;
  timeout_imu_2 = 0.5*timeout_limit;
  timeout_wheelspeeds = 0.5*timeout_limit;
  timeout_race_control = 2 * timeout_limit;
  old_joystick_counter = 0;
  // initialize diagnostics raptor error
  diagnostics_report_error = false;

  // initialize tire temps
  for(int i = 0; i<16; i++)
  {
    TireTemp_FL[i] = 0;
    TireTemp_FR[i] = 0;
    TireTemp_RL[i] = 0;
    TireTemp_RR[i] = 0;
  }

  //Initialization of coordinates
  if(gps_origin.compare("IMS") == 0)
  {
    origin = {39.809786, -86.235148, 0.0}; 
    RCLCPP_INFO(this->get_logger(), "GPS Origin: IMS");
  }
  else if(gps_origin.compare("HIL") == 0)
  {
    origin = {39.809786, -86.235148, 0.0}; 
    RCLCPP_INFO(this->get_logger(), "GPS Origin: HIL");
  }
  else if(gps_origin.compare("LOR") == 0)
  {
    origin = {39.810197, -86.34254, 0.0}; 
    RCLCPP_INFO(this->get_logger(), "GPS Origin: LOR");
  }
  else if(gps_origin.compare("LVMS") == 0)
  {
    origin = {36.268252, -115.015914, 0.0};
    RCLCPP_INFO(this->get_logger(), "GPS Origin: LVMS");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "No valid GPS origin specified!");
  }
  origin_conv = {0,0,0};
  origin_conv[0] = origin[0] / (360/(2 * M_PI));
  origin_conv[1] = origin[1] / (360/(2 * M_PI));
  origin_conv[2] = origin[2];
  // initialize joystick 
  joystick_gear = 0; 
  joystick_steering = 0; 
  joystick_throttle = 0; 
  joystick_brake = 0; 
  battery_voltage = 0;
  joystick_emergency_triggered = false;
  reset_hardware = false;
  reset_hardware_old = false;
  manual_e_brake = false;
  driven_once = false;

  // initializie can variables
  steering = 0;
  acc_pedal = 0;
  brake = 0;
  gear_cmd = 0;
  ct_state = 0;

  // CAN misc report
  sys_state = 0;

  // run wireless comms on best effort
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  // create publishers (logger)
  pub_debug = this->create_publisher<std_msgs::msg::String>("/mod_control/debug", 1);
  pub_debug_slow = this->create_publisher<std_msgs::msg::String>("/mod_control/debug_slow", 1);
  pub_debug_latency = this->create_publisher<std_msgs::msg::String>("/mod_control/debug_latency", 1);

  // create publishers (software)
  pub_status = this->create_publisher<tum_msgs::msg::TUMModuleStatus>("/mod_control/status_control", 1);
  pub_state_estimate = this->create_publisher<tum_msgs::msg::TUMStateEstimate>("/mod_control/vehicle_state_estimate", 1);
  pub_tpa_data = this->create_publisher<tum_msgs::msg::TUMTpaVdcData>("/mod_control/tpa_vdc_data", 1);
  pub_tum_request = this->create_publisher<tum_msgs::msg::TUMRequest>("tum_req", 1);
  pub_race_control = this->create_publisher<deep_orange_msgs::msg::RcToCt>("/mod_control/rc_to_ct", 1);
  pub_manual_mode = this->create_publisher<std_msgs::msg::Bool>("/mod_control/manual_mode", 1);

  // create subscriptions (software)
  sub_status_objects_pointrcnn = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_objects/status_pointrcnn", 1, std::bind(&ControlHandler::on_sub_status_objects_pointrcnn, this, std::placeholders::_1));
  sub_status_objects_clustering = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_objects/status_clustering", 1, std::bind(&ControlHandler::on_sub_status_objects_clustering, this, std::placeholders::_1));
  sub_status_objects_radar = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_objects/status_radar", 1, std::bind(&ControlHandler::on_sub_status_objects_radar, this, std::placeholders::_1));
  sub_status_prediction = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_prediction/status_prediction", 1, std::bind(&ControlHandler::on_sub_status_prediction, this, std::placeholders::_1));
  sub_status_local_planner = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_local_planner/status_local_planner", 1, std::bind(&ControlHandler::on_sub_status_local_planner, this, std::placeholders::_1));
  sub_status_map_loc = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_map_loc/status_map_loc", 1, std::bind(&ControlHandler::on_sub_status_map_loc, this, std::placeholders::_1));
  sub_status_preprocessing = this->create_subscription<tum_msgs::msg::TUMModuleStatus>(
      "/mod_sensors/status_preprocessing", 1, std::bind(&ControlHandler::on_sub_status_preprocessing, this, std::placeholders::_1));
  sub_trajectory = this->create_subscription<tum_msgs::msg::TUMTrajectory>(
      "/mod_local_planner/trajectory_performance", 1, std::bind(&ControlHandler::on_sub_trajectory, this, std::placeholders::_1));
  sub_em_trajectory = this->create_subscription<tum_msgs::msg::TUMTrajectory>(
      "/mod_local_planner/trajectory_emergency", 1, std::bind(&ControlHandler::on_sub_em_trajectory, this, std::placeholders::_1));
  sub_map_ref = this->create_subscription<tum_msgs::msg::TUMMapReference>(
      "/mod_local_planner/map_reference", 1, std::bind(&ControlHandler::on_sub_map_ref, this, std::placeholders::_1));
  sub_control_param = this->create_subscription<tum_msgs::msg::TUMControlParam>(
      "/mod_control/control_param", 1, std::bind(&ControlHandler::on_sub_control_param, this, std::placeholders::_1));
  sub_lidar_loc = this->create_subscription<tum_msgs::msg::TUMLocalization>(
      "/mod_map_loc/localization", 1, std::bind(&ControlHandler::on_sub_lidar_loc, this, std::placeholders::_1));

  // create subscriptions (sensors)
  sub_gps_1 = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "novatel_top/bestpos", qos, std::bind(&ControlHandler::on_sub_gps_1, this, std::placeholders::_1));
  sub_vel_1 = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "novatel_top/bestvel", qos, std::bind(&ControlHandler::on_sub_vel_1, this, std::placeholders::_1));
  sub_imu_1 = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos, std::bind(&ControlHandler::on_sub_imu_1, this, std::placeholders::_1));
  sub_gps_2 = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "novatel_bottom/bestpos", qos, std::bind(&ControlHandler::on_sub_gps_2, this, std::placeholders::_1));
  sub_vel_2 = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "novatel_bottom/bestvel", qos, std::bind(&ControlHandler::on_sub_vel_2, this, std::placeholders::_1));
  sub_imu_2 = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_bottom/rawimu", qos, std::bind(&ControlHandler::on_sub_imu_2, this, std::placeholders::_1));
  sub_dual_antenna_1 = this->create_subscription<novatel_oem7_msgs::msg::DUALANTENNAHEADING>(
    "novatel_top/dualantennaheading", qos, std::bind(&ControlHandler::on_sub_dual_antenna_1, this, std::placeholders::_1));

  // create subscriptions (race control)
  sub_reset_hardware = this->create_subscription<std_msgs::msg::Bool>(
  "/mod_control/reset_hardware", 1, std::bind(&ControlHandler::on_sub_reset_hardware, this, std::placeholders::_1));

  // create publisher and subscriber for CAN
  sub_from_simu_can_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 20, std::bind(&ControlHandler::on_sub_from_can, this, std::placeholders::_1));
  in_can_msg_ = std::make_shared<can_msgs::msg::Frame>();
  pub_to_simu_can_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 20);

  // initialize UDP communication for live-visualization
  if(enable_tubes)
  {
    // initialize UDP communication for live-visualization
    send_driving_tube_ip = "10.0.0.200";
    send_driving_tube_port = 50100 + vehicle_id;
    send_driving_tube_socket.open(ip::udp::v4());
    send_driving_tube_endpoint = ip::udp::endpoint(ip::address::from_string(send_driving_tube_ip), send_driving_tube_port);
    RCLCPP_INFO(this->get_logger(), "Established connection to HIL with: %s:%u", send_driving_tube_ip.c_str(), send_driving_tube_port);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Driving tubes will not be visualized. Enable them via parameter enable_tubes.");
  }

  // initialize software (dummy)
  #ifdef CONTROL
  model->inputs->in_VehicleSystemStatus.AIDriver = 2; 
  model->inputs->in_VehicleSystemStatus.Hardware = 2; 
  model->inputs->in_VehicleSystemStatus.ParkBrakeApplied_b = 0; 
  model->inputs->in_VehicleSystemStatus.VehicleReady2Drive = 1; 
  model->inputs->in_ActuatorLimitations.SteeringAngleMax_rad = 0.2; 
  model->inputs->in_ActuatorLimitations.SteeringAngleMin_rad = -0.2; 
  model->inputs->in_ActuatorLimitations.DriveForceMax_N = 6000; 
  model->inputs->in_ActuatorLimitations.DriveForceMin_N = -35000;
  #endif
}

ControlHandler::~ControlHandler()
{
  #ifdef CONTROL
  controller_dev_py_terminate(model);
  #else
  trajectory_planning_dev_py_terminate(model);
  #endif
}

void ControlHandler::init()
{
  #ifdef CONTROL
  // set real-time priority (only in control since SIL does not have real-time enabled kernel settings)
  pid_t pid = getpid(); 
  std::string cmd = "chrt --rr -p 90 "; 
  std::string pid_string = std::to_string(pid);
  std::string cmd_full = cmd + pid_string; 
  system(cmd_full.c_str());
  #endif 

  // store timing to do correct stepping
  dT = 0.01;

  // read log file definitions
  std::ifstream inFile_debug("src/mod_control/control/debug.txt");
  N_DEBUG_SIGNALS = std::count(std::istreambuf_iterator<char>(inFile_debug),
             std::istreambuf_iterator<char>(), '\n');
  RCLCPP_INFO(this->get_logger(), "Found %i debug signals ...", N_DEBUG_SIGNALS);
  std::ifstream inFile_debug_slow("src/mod_control/control/debug_slow.txt");
  N_DEBUG_SLOW_SIGNALS = std::count(std::istreambuf_iterator<char>(inFile_debug_slow),
              std::istreambuf_iterator<char>(), '\n');
  RCLCPP_INFO(this->get_logger(), "Found %i slow debug signals ...", N_DEBUG_SLOW_SIGNALS);

  log_line.data.reserve(100000);
  log_line_slow.data.reserve(100000);
  log_line_latency.data.reserve(1000);
  // initialize log_line_latency file
  log_line_latency.data = "latency_id;t_receive;t_send;status\n";
  pub_debug_latency->publish(log_line_latency);

  // parse parameter file
  std::ifstream paramFile("config/mod_control_config.ini");
  std::string line;
  size_t idx;
  std::string param;
  std::string value;
  while(std::getline(paramFile, line))
  {
    // check whether line is a comment
    if(line.at(0) == '#')
    {
      continue;
    }
    // find occurence of equal to split into parameter and value
    size_t idx = line.find('=');
    param = line.substr(0, idx);
    value = line.substr(idx+1);
    // remove blanks
    param.erase(std::remove(param.begin(), param.end(), ' '), param.end());
    value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
    // actually set parameter
    set_tum_parameter(param, std::stod(value));
  }
  // wait for map reference (only in sil mode)
  while(!initialized && sil_mode)
  {
    // process messages until the callback for the map reference has been called
    // and sets the initialized variable to true
    rclcpp::spin_some(this->get_node_base_interface());
    RCLCPP_INFO(this->get_logger(), "Waiting for map reference ...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  // wait until everything is initialized and then go
  int cnt = 3;
  while(cnt > 0)
  {
    RCLCPP_INFO(this->get_logger(), "We are ready to roll in %i seconds ...", cnt);
    cnt--;
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // initialize timer for cyclic tasks
  // this has to be done after the initial period to make sure that map reference is set correctly
  timer_ = this->create_wall_timer(10ms, std::bind(&ControlHandler::step, this));

  #ifdef CONTROL
  // initialize telemetry (only when in non-HIL mode)
  if(!enable_tubes)
  {
    send_telemetry_ip = "10.42.3.1";
    send_telemetry_port = 23431;
    send_telemetry_socket.open(ip::udp::v4());
    send_telemetry_endpoint = ip::udp::endpoint(ip::address::from_string(send_telemetry_ip), send_telemetry_port);
    RCLCPP_INFO(this->get_logger(), "Established connection to send telemetry to : %s:%u", send_telemetry_ip.c_str(), send_telemetry_port);
    recv_basestation_ip = "10.42.3.200";
    recv_basestation_port = 23531;
    recv_basestation_socket.open(ip::udp::v4());
    recv_basestation_endpoint = ip::udp::endpoint(ip::address::from_string(recv_basestation_ip), recv_basestation_port);
    recv_basestation_socket.bind(recv_basestation_endpoint);
    recv_basestation_socket.non_blocking(true);
        send_telemetry_endpoint = ip::udp::endpoint(ip::address::from_string(send_telemetry_ip), send_telemetry_port);
    RCLCPP_INFO(this->get_logger(), "Established connection to receive basestation commands on : %s:%u", recv_basestation_ip.c_str(), recv_basestation_port);
  }
  else
  {
    send_telemetry_ip = "10.0.0.1" + std::to_string(vehicle_id);
    send_telemetry_port = 23431;
    send_telemetry_socket.open(ip::udp::v4());
    send_telemetry_endpoint = ip::udp::endpoint(ip::address::from_string(send_telemetry_ip), send_telemetry_port);
    RCLCPP_INFO(this->get_logger(), "Established connection to send telemetry to : %s:%u", send_telemetry_ip.c_str(), send_telemetry_port);
    recv_basestation_ip = "10.0.0.1" + std::to_string(vehicle_id);
    recv_basestation_port = 23531;
    recv_basestation_socket.open(ip::udp::v4());
    recv_basestation_endpoint = ip::udp::endpoint(ip::address::from_string(recv_basestation_ip), recv_basestation_port);
    recv_basestation_socket.bind(recv_basestation_endpoint);
    recv_basestation_socket.non_blocking(true);
        send_telemetry_endpoint = ip::udp::endpoint(ip::address::from_string(send_telemetry_ip), send_telemetry_port);
    RCLCPP_INFO(this->get_logger(), "Established connection to receive basestation commands on : %s:%u", recv_basestation_ip.c_str(), recv_basestation_port);
  }

  #endif

  // initialize TUM request handling 
  tum_req_cnt_old = 0;
}

void ControlHandler::step()
{
  // measure timing
  std::chrono::time_point<std::chrono::system_clock> tCurrent = std::chrono::system_clock::now(); 
  std::chrono::nanoseconds cycle_time_ns = tCurrent - tPrev; 
  // write estimate frequency 
  model->inputs->in_ExternalDebug.UpdateFrequency_Hz = 1000000000/cycle_time_ns.count();
  // write current unix time 
  model->inputs->in_ExternalDebug.UnixTime_s = time; 
  // write last cycle execution time 
  model->inputs->in_ExternalDebug.ExecutionTime_s = double(tExec_ns.count())/1000000000;
  // write last cycle controller time
  model->inputs->in_ExternalDebug.ControlTime_s = double(tControl_ns.count())/1000000000;
  // store current joystick timeout 
  model->inputs->in_ExternalDebug.LostHeartbeatTimeout_s = timeout_joystick;
   // store start time of this cycle for next runs calculation
  tPrev = tCurrent; 

  #ifdef CONTROL
  // update tire temps based on sensor geometry such that each part of the tire is covered equally
  model->inputs->in_VehicleSensorData.T_TireFL_Celsius[0] = (TireTemp_FL[0] + TireTemp_FL[1])/2;
  model->inputs->in_VehicleSensorData.T_TireFL_Celsius[1] = (TireTemp_FL[2] + TireTemp_FL[3] + TireTemp_FL[4] + TireTemp_FL[5])/4;
  model->inputs->in_VehicleSensorData.T_TireFL_Celsius[2] = (TireTemp_FL[6] + TireTemp_FL[7] + TireTemp_FL[8] + TireTemp_FL[9] + TireTemp_FL[10])/5;
  model->inputs->in_VehicleSensorData.T_TireFL_Celsius[3] = (TireTemp_FL[11] + TireTemp_FL[12] + TireTemp_FL[13] + TireTemp_FL[14] + TireTemp_FL[15])/5;
  model->inputs->in_VehicleSensorData.T_TireFR_Celsius[0] = (TireTemp_FR[0] + TireTemp_FR[1])/2;
  model->inputs->in_VehicleSensorData.T_TireFR_Celsius[1] = (TireTemp_FR[2] + TireTemp_FR[3] + TireTemp_FR[4] + TireTemp_FR[5])/4;
  model->inputs->in_VehicleSensorData.T_TireFR_Celsius[2] = (TireTemp_FR[6] + TireTemp_FR[7] + TireTemp_FR[8] + TireTemp_FR[9] + TireTemp_FR[10])/5;
  model->inputs->in_VehicleSensorData.T_TireFR_Celsius[3] = (TireTemp_FR[11] + TireTemp_FR[12] + TireTemp_FR[13] + TireTemp_FR[14] + TireTemp_FR[15])/5;
  model->inputs->in_VehicleSensorData.T_TireRL_Celsius[0] = (TireTemp_RL[0] + TireTemp_RL[1])/2;
  model->inputs->in_VehicleSensorData.T_TireRL_Celsius[1] = (TireTemp_RL[2] + TireTemp_RL[3] + TireTemp_RL[4] + TireTemp_RL[5])/4;
  model->inputs->in_VehicleSensorData.T_TireRL_Celsius[2] = (TireTemp_RL[6] + TireTemp_RL[7] + TireTemp_RL[8] + TireTemp_RL[9] + TireTemp_RL[10])/5;
  model->inputs->in_VehicleSensorData.T_TireRL_Celsius[3] = (TireTemp_RL[11] + TireTemp_RL[12] + TireTemp_RL[13] + TireTemp_RL[14] + TireTemp_RL[15])/5;
  model->inputs->in_VehicleSensorData.T_TireRR_Celsius[0] = (TireTemp_RR[0] + TireTemp_RR[1])/2;
  model->inputs->in_VehicleSensorData.T_TireRR_Celsius[1] = (TireTemp_RR[2] + TireTemp_RR[3] + TireTemp_RR[4] + TireTemp_RR[5])/4;
  model->inputs->in_VehicleSensorData.T_TireRR_Celsius[2] = (TireTemp_RR[6] + TireTemp_RR[7] + TireTemp_RR[8] + TireTemp_RR[9] + TireTemp_RR[10])/5;
  model->inputs->in_VehicleSensorData.T_TireRR_Celsius[3] = (TireTemp_RR[11] + TireTemp_RR[12] + TireTemp_RR[13] + TireTemp_RR[14] + TireTemp_RR[15])/5;
  
  telemetry();
  #endif

  // update internal time and watchdogs
  time += dT;
  rolling_cnt++;
  if (rolling_cnt > 255)
  {
    rolling_cnt = 0;
  }
  // increase timeouts by time increment and update comms status
  
  // timeouts are reset by message callbacks
  // only increase objects and map_loc if not in sil mode 
  // as they are not available there
  if(!sil_mode)
  {
    // timeout_objects_pointrcnn += dT;
    // timeout_objects_clustering += dT;
    // timeout_objects_radar += dT;
    // timeout_map_loc += dT;
    timeout_preprocessing += dT;
    timeout_imu_1 += dT;    
    timeout_imu_2 += dT;
    timeout_wheelspeeds += dT;
  }
  // increase timeout for joystick only if joystick is used 
  if(joystick_used)
  {
    // this limits the timeout such that resetting is possible 
    if(timeout_joystick < 6*timeout_limit)
    {
      timeout_joystick += dT;
    }
  }
  if(race_control_used)
  {
    timeout_race_control += dT;    
  }
  timeout_prediction += dT;
  timeout_local_planner += dT;
  timeout_trajectory_performance += dT;
  timeout_trajectory_emergency += dT;
  if(timeout_objects_pointrcnn > timeout_limit || timeout_prediction > timeout_limit ||
      timeout_local_planner > timeout_limit || timeout_map_loc > timeout_limit)
  {
    model->inputs->in_TrajectoryPlanning.Strategy_CommsStatus = 0;
    // only print timeouts when vehicle has been driving before
    if (model->outputs->out_TUMModeControl.TUMVehicleState >= 30)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout detected: ");
      RCLCPP_ERROR(this->get_logger(), "LOC: %4.2fs | OBJ: %4.2fs | LTPL: %4.2fs | PRED: %4.2fs | Joystick: %4.2fs  | Race Control: %4.2fs",
        timeout_map_loc, timeout_objects_pointrcnn, timeout_local_planner, timeout_prediction, timeout_joystick, timeout_race_control);
    }
  }
  else
  {
    model->inputs->in_TrajectoryPlanning.Strategy_CommsStatus = 2;
  }
  // handling of timeout resets (must be before timeout to ensure that it is not overwritten)
  // only reset on rising edge on reset hardware bit and not always
  // this prevents not safe stopping in case that the reset bit is still set
  if(reset_hardware != reset_hardware_old && reset_hardware)
  {
    // if reset bit is send, reset manual_e_brake and allow for full manual control again
    manual_e_brake = false;
    #ifdef CONTROL
    // reset software
    model->inputs->in_ActuatorLimitations.DriveForceMax_N = 6000;
    #endif
  }
  // store current reset hardware value into old value to detect rising edge
  reset_hardware_old = reset_hardware;

  // timeout handling 
  if(manual_mode)
  {
    // max allowed timeout in manual mode is less than in autonomous mode
    // also race control is not considered here
    if(timeout_joystick > timeout_limit)
    {
      // request manual e-brake in case that we are in manual mode
      manual_e_brake = true;  
    }
  }
  else
  {
    if(timeout_joystick > 4*timeout_limit || timeout_race_control > 40*timeout_limit)
    {
      // request a safe stop from the planner
      auto tum_req_msg = tum_msgs::msg::TUMRequest();
      tum_req_msg.tum_req = 1;
      pub_tum_request->publish(tum_req_msg);

      #ifdef CONTROL
      // limit control request to braking 
      model->inputs->in_ActuatorLimitations.DriveForceMax_N = -2000;
      #endif
    }
  }

  #ifdef CONTROL
  if(timeout_imu_1 > 0.5*timeout_limit)
  {
    // label IMU as faulty
    model->inputs->in_VehicleSensorData.valid_IMU1_b = 0;
  }
  if(timeout_imu_2 > 0.5*timeout_limit)
  {
    // label IMU as faulty
    model->inputs->in_VehicleSensorData.valid_IMU2_b = 0;
  }

  if(timeout_wheelspeeds > 0.5*timeout_limit)
  {
    model->inputs->in_VehicleSensorData.valid_Wheelspeeds_b = 0;
  }
  #endif
  if(timeout_trajectory_performance > timeout_limit || timeout_trajectory_emergency > timeout_limit)
  {
    model->inputs->in_TrajectoryPlanning.Trajectories_CommsStatus = 0;
  }
  else
  {
    model->inputs->in_TrajectoryPlanning.Trajectories_CommsStatus = 2;
  }
  // parse vehicle status
  int status_overall = 0;
  // objects and localization are not used in hil mode therefore they can be set artificially 
  // to 30 such that they do not interfere with the status evaluation
  if(sil_mode)
  {
    status_map_loc = 30; 
    status_objects_pointrcnn = 30; 
  }
  if(status_objects_pointrcnn >= -1 && status_prediction >= 10
    && status_map_loc >= -1 && status_local_planner >= 10)
  {
    status_overall = 10;
  }
  if(status_objects_pointrcnn >= -1 && status_prediction >= 20
    && status_map_loc >= -1 && status_local_planner >= 20 &&
    (!ct_sm_used || (dbw_state_machine.get_ct_state() >= 8 && dbw_state_machine.get_ct_state() < 255)) 
    && !manual_mode)
  {
    status_overall = 20;
  }
  if(status_objects_pointrcnn >= -1 && status_prediction >= 30
    && status_map_loc >= -1 && status_local_planner >= 30 &&
    (!ct_sm_used || (dbw_state_machine.get_ct_state() >= 8 && dbw_state_machine.get_ct_state() < 255)) 
    && !manual_mode)
  {
    status_overall = 30;
  }
  if(status_objects_pointrcnn >= 50 || status_prediction >= 50
    || status_map_loc >= 50 || status_local_planner >= 50 || diagnostics_report_error)
  {
    status_overall = 50;
  }
  model->inputs->in_TrajectoryPlanning.Strategy_Status = status_overall;

  // handle CT state machine
  dbw_state_machine.transition();
  //auto mes_ct_state = deep_orange_msgs::msg::CtReport();
  ct_state = dbw_state_machine.get_ct_state();

  // implement different main loop models depending on whether controller or
  // software-in-the-loop simulation is built
  std::chrono::time_point<std::chrono::system_clock> start_Control = std::chrono::system_clock::now(); 
  #ifdef CONTROL
  controller_dev_py_step(model);
  #else
  // step fast subrate once (2ms)
  trajectory_planning_dev_py_step0(model);
  // step slow subrate once (10ms)
  trajectory_planning_dev_py_step2(model);
  // step fast subrate another four times to also achieve a run-time of 10ms
  for(int i = 0; i<4; i++)
  {
    trajectory_planning_dev_py_step0(model);
  }
  #endif
  std::chrono::time_point<std::chrono::system_clock> end_Control = std::chrono::system_clock::now(); 
  tControl_ns = end_Control - start_Control; 

  // update tire temps 
  

  // send out status message
  auto mes_status = tum_msgs::msg::TUMModuleStatus();
  mes_status.watchdog = rolling_cnt;
  mes_status.status = model->outputs->out_TUMModeControl.TUMVehicleState;
  pub_status->publish(mes_status);
  // send out manual mode information 
  auto mes_manual_mode = std_msgs::msg::Bool();
  mes_manual_mode.data = manual_mode;
  pub_manual_mode->publish(mes_manual_mode);
  // send out state estimate
  auto mes_state_estimate = tum_msgs::msg::TUMStateEstimate();
  mes_state_estimate.time_ns = this->get_clock()->now().nanoseconds();
  mes_state_estimate.x_cg_m = model->outputs->out_VehicleDynamicState.Pos.x_m;
  mes_state_estimate.y_cg_m = model->outputs->out_VehicleDynamicState.Pos.y_m;
  mes_state_estimate.z_cg_m = 0;
  mes_state_estimate.psi_cg_rad = model->outputs->out_VehicleDynamicState.Pos.psi_rad;
  mes_state_estimate.vx_cg_mps = model->outputs->out_VehicleDynamicState.vx_mps;
  mes_state_estimate.vy_cg_mps = model->outputs->out_VehicleDynamicState.vy_mps;
  mes_state_estimate.dpsi_cg_radps = model->outputs->out_VehicleDynamicState.dPsi_radps;
  mes_state_estimate.beta_cg_rad = model->outputs->out_VehicleDynamicState.beta_rad;
  mes_state_estimate.ax_cg_mps2 = model->outputs->out_VehicleDynamicState.ax_mps2;
  mes_state_estimate.ay_cg_mps2 = model->outputs->out_VehicleDynamicState.ay_mps2;
  mes_state_estimate.kappa_radpm = model->outputs->out_VehicleDynamicState.kappa_radpm;
  mes_state_estimate.accuracy_pos_x_m = 0;
  mes_state_estimate.accuracy_pos_y_m = 0;
  mes_state_estimate.accuracy_pos_z_m = 0;
  mes_state_estimate.accuracy_pos_psi_rad = 0;
  mes_state_estimate.accuracy_vx_mps = 0;
  mes_state_estimate.accuracy_vy_mps = 0;
  mes_state_estimate.accuracy_dpsi_radps = 0;
  mes_state_estimate.status = model->outputs->out_VehicleDynamicState.SE_Status;
  pub_state_estimate->publish(mes_state_estimate);

  // send out commands (only in controller mode)
  #ifdef CONTROL
  // check if manual mode was enabled and if joystick is used 
  this->get_parameter("manual_mode", manual_mode);
  this->get_parameter("joystick_used", joystick_used);
  // if joystick is enabled bypass outputs directly from there
  if(manual_mode)
  {
    if(manual_e_brake)
    {
      joystick_throttle = 0;
      joystick_brake = 1800000;
    }
    brake = joystick_brake/1000;
    acc_pedal = joystick_throttle;
    gear_cmd = joystick_gear;
    steering = joystick_steering*19.5; // due to transmission
    RCLCPP_DEBUG(this->get_logger(), "Joystick: Throttle: %.2lf, Brake: %.2lf, Steering: %.2lf", joystick_throttle, joystick_brake, joystick_steering);
  }
  else
  {
    // conversions to AV-21 values
    acc_pedal= model->outputs->out_ThrottlePosition_perc*100;
    brake = model->outputs->out_BrakePressure_bar*100;
    gear_cmd = model->outputs->out_Gear;
    // convert to steering wheel angle in degree
    // steering ratio is 19.5
    steering = model->outputs->out_SteeringAngleAtWheel_rad*360*19.5/6.28;
    // if joystick is used, use joystick as safety feature
    // otherwise just pass through controller things 
    if(joystick_used && !basestation_only)
    {
      if(acc_pedal > (100.0-joystick_throttle))
      {
        acc_pedal = (100.0-joystick_throttle);
      }
      if(joystick_brake > 50000)
      {
        brake = joystick_brake/1000;
      }
    }
  }
  send_steering();
  send_accel_pedal();
  send_brake();         
  send_gear();
  send_ct_report();
  #endif
  latency_t_send = ros_time.now().nanoseconds();

  // send out tpa data
  auto mes_tpa_data = tum_msgs::msg::TUMTpaVdcData();
  mes_tpa_data.pos_x_m = model->outputs->out_tpa_vdcsignals.Position.x_m;
  mes_tpa_data.pos_y_m = model->outputs->out_tpa_vdcsignals.Position.y_m;
  mes_tpa_data.psi_rad = model->outputs->out_tpa_vdcsignals.Position.psi_rad;
  mes_tpa_data.vx_mps = model->outputs->out_tpa_vdcsignals.vx_mps;
  mes_tpa_data.vy_mps = model->outputs->out_tpa_vdcsignals.vy_mps;
  mes_tpa_data.dpsi_radps = model->outputs->out_tpa_vdcsignals.dPsi_radps;
  mes_tpa_data.beta_rad = model->outputs->out_tpa_vdcsignals.beta_rad;
  mes_tpa_data.ax_mps2 = model->outputs->out_tpa_vdcsignals.ax_mps2;
  mes_tpa_data.ay_mps2 = model->outputs->out_tpa_vdcsignals.ay_mps2;
  mes_tpa_data.delta_wheel_rad = model->outputs->out_tpa_vdcsignals.Delta_Wheel_rad;
  mes_tpa_data.throttlesignal_perc = model->outputs->out_tpa_vdcsignals.ThrottlePos_perc;
  mes_tpa_data.p_brake_f_bar = model->outputs->out_tpa_vdcsignals.p_BrakeF_bar;
  mes_tpa_data.p_brake_r_bar = model->outputs->out_tpa_vdcsignals.p_BrakeR_bar;
  mes_tpa_data.omega_wheel_fl_radps = model->outputs->out_tpa_vdcsignals.omega_WheelFL_radps;
  mes_tpa_data.omega_wheel_fr_radps = model->outputs->out_tpa_vdcsignals.omega_WheelFR_radps;
  mes_tpa_data.omega_wheel_rl_radps = model->outputs->out_tpa_vdcsignals.omega_WheelRL_radps;
  mes_tpa_data.omega_wheel_rr_radps = model->outputs->out_tpa_vdcsignals.omega_WheelRR_radps;
  pub_tpa_data->publish(mes_tpa_data);

  if(enable_tubes)
  {
    // send out UDP data for visualization 
    boost::system::error_code err;
    // data frame structure (x_left[50], y_left[50], x_right[50], y_right[50])
    float data_frame[200];
    for(int i = 0; i < 50; i++)
    {
      data_frame[i] = x_left_driving_tube_m[i];
      data_frame[50+i] = y_left_driving_tube_m[i];
      data_frame[100+i] = x_right_driving_tube_m[i];
      data_frame[150+i] = y_right_driving_tube_m[i];
    }
    send_driving_tube_socket.send_to(buffer(data_frame, sizeof(data_frame)), send_driving_tube_endpoint, 0, err);
  }

  // logging
  logging();
  // do all the printing
  print_status();

  // measure execution time full step 
  std::chrono::time_point<std::chrono::system_clock> tEnd = std::chrono::system_clock::now(); 
  tExec_ns = tEnd - tCurrent;
}


void ControlHandler::on_sub_from_can(const can_msgs::msg::Frame::SharedPtr msg)
{
  in_can_msg_ = msg;
  decode_can_msg();
}

void ControlHandler::decode_can_msg()
{
  #ifdef CONTROL
  // DECODE CURRENT CAN MSG USING THE IMPORTED DBC
  if (!in_can_msg_->is_rtr && !in_can_msg_->is_error)
  {
    switch (in_can_msg_->id)
    {
      case ID_MISC_REPORT_DO:
      {
        // SYS STATE MSG
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_MISC_REPORT_DO);
          if (in_can_msg_->dlc >= can_message->GetDlc())
          {
            can_message->SetFrame(in_can_msg_);  
            // parse other data 
            battery_voltage = can_message->GetSignal("battery_voltage")->GetResult();
            model->inputs->in_VehicleSensorData.BatteryVoltage_V = battery_voltage;
            sys_state = can_message->GetSignal("sys_state")->GetResult();
            switch (sys_state) {
              case 1:
                dbw_state_machine.update_sys_state(SysState::SS1_PWR_ON);
                break;
              case 2:
                dbw_state_machine.update_sys_state(SysState::SS2_SUBSYS_CON);
                break;
              case 3:
                dbw_state_machine.update_sys_state(SysState::SS3_ACT_TESTING);
                break;
              case 4:
                dbw_state_machine.update_sys_state(SysState::SS4_ACT_TEST_DONE);
                break;
              case 5:
                dbw_state_machine.update_sys_state(SysState::SS5_CRANKREADY);
                break;
              case 6:
                dbw_state_machine.update_sys_state(SysState::SS6_PRECRANK_CHECK);
                break;
              case 7:
                dbw_state_machine.update_sys_state(SysState::SS7_CRANKING);
                break;
              case 8:
                dbw_state_machine.update_sys_state(SysState::SS8_ENG_RUNNING);
                break;
              case 9:
                dbw_state_machine.update_sys_state(SysState::SS9_DRIVING);
                break;
              case 10:
                dbw_state_machine.update_sys_state(SysState::SS10_SHUT_ENG);
                break;
              case 11:
                dbw_state_machine.update_sys_state(SysState::SS11_PWR_OFF);
                break;
              case 13:
                dbw_state_machine.update_sys_state(SysState::SS13_CRANK_CHECK_INIT);
                break;
              default:
                RCLCPP_ERROR(this->get_logger(), "Incorrect Raptor Mode Received. sys_state: %d", sys_state);
            }
          }
        break;
      }
      case ID_PT_REPORT_1:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_PT_REPORT_1);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);

          // update ICE signals (only in control mode)
          model->inputs->in_VehicleSensorData.omega_Engine_radps = can_message->GetSignal("engine_speed_rpm")->GetResult()*2*3.1415/60;
          model->inputs->in_VehicleSensorData.GearEngaged = can_message->GetSignal("current_gear")->GetResult();
        }
        break;
      }

      case ID_PT_REPORT_2:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_PT_REPORT_2);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);

          // update ICE signals (only in control mode)
          model->inputs->in_VehicleSensorData.T_CoolantTemp_Celsius = can_message->GetSignal("coolant_temperature")->GetResult();
          model->inputs->in_VehicleSensorData.T_TransmissionOilTemp_Celsius = can_message->GetSignal("transmission_temperature")->GetResult();
          model->inputs->in_VehicleSensorData.p_Fuel_kPa = can_message->GetSignal("fuel_pressure_kPa")->GetResult();
          model->inputs->in_VehicleSensorData.p_EngineOil_kPa = can_message->GetSignal("engine_oil_pressure_kPa")->GetResult();
        }
        break;
      }

      case ID_WHEEL_SPEED_REPORT_DO:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          // update wheelspeeds (only available in control)
          timeout_wheelspeeds = 0;
          // convert wheelspeeds from kph to radps. Radius is taken from powertrain documentation
          // https://docs.google.com/document/d/1gJYag8tVhtnSl3MjWvsVMBEfaR9WXndrybvJtXeLbU8/edit#
          model->inputs->in_VehicleSensorData.omega_WheelFL_radps = can_message->GetSignal("wheel_speed_FL")->GetResult()/(3.6*0.301);
          model->inputs->in_VehicleSensorData.omega_WheelFR_radps = can_message->GetSignal("wheel_speed_FR")->GetResult()/(3.6*0.301);
          model->inputs->in_VehicleSensorData.omega_WheelRL_radps = can_message->GetSignal("wheel_speed_RL")->GetResult()/(3.6*0.3118);
          model->inputs->in_VehicleSensorData.omega_WheelRR_radps = can_message->GetSignal("wheel_speed_RR")->GetResult()/(3.6*0.3118);
          model->inputs->in_VehicleSensorData.valid_Wheelspeeds_b = 1;
        }
        break;
      }
      case ID_BRAKE_PRESSURE_REPORT_DO:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
        // update ICE signals (only in control mode)
        model->inputs->in_VehicleSensorData.p_BrakeF_bar = can_message->GetSignal("brake_pressure_fdbk_front")->GetResult()/100;
        model->inputs->in_VehicleSensorData.p_BrakeR_bar = can_message->GetSignal("brake_pressure_fdbk_rear")->GetResult()/100;

        }
        break;
      }
      case ID_STEERING_REPORT_DO:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_STEERING_REPORT_DO);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
            // convert from degree at steering wheel to radians at front wheel
            // steering ratio is 19.5
          model->inputs->in_VehicleSensorData.Delta_Wheel_rad = can_message->GetSignal("steering_motor_ang_avg_fdbk")->GetResult()*6.28/(360*19.5);
        }
        break;
      }
      case ID_ACCELERATOR_REPORT_DO:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          // RCLCPP_DEBUG(this->get_logger(), "Received accelrator report data");
        }
        break;
      }
      case ID_BASE_TO_CAR:
      {
        // only use this message if race control is enabled
        if(race_control_used)
        {
          // reset race control timeout
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_BASE_TO_CAR);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);
            msg_rc_to_ct.base_to_car_heartbeat = can_message->GetSignal("base_to_car_heartbeat")->GetResult();
            msg_rc_to_ct.track_flag = can_message->GetSignal("track_flag")->GetResult();
            msg_rc_to_ct.veh_flag = can_message->GetSignal("veh_flag")->GetResult();
            msg_rc_to_ct.veh_rank = can_message->GetSignal("veh_rank")->GetResult();
            msg_rc_to_ct.lap_status_whole = 0;
            msg_rc_to_ct.lap_status_fraction = 0;
            msg_rc_to_ct.round_target_speed = can_message->GetSignal("round_target_speed")->GetResult();

            // set acknowledge values 
            track_flag_ack = msg_rc_to_ct.track_flag;
            veh_flag_ack = msg_rc_to_ct.veh_flag;
            pub_race_control->publish(msg_rc_to_ct);
            switch (msg_rc_to_ct.track_flag)
              {
                case 1:
                  dbw_state_machine.update_track_condition(TrackCondition::TC1_RED);
                  break;
                case 2:
                  dbw_state_machine.update_track_condition(TrackCondition::TC2_ORANGE);
                  break;
                case 3:
                  dbw_state_machine.update_track_condition(TrackCondition::TC3_YELLOW);
                  break;
                case 4:
                  dbw_state_machine.update_track_condition(TrackCondition::TC4_GREEN);
                  break;
                case 255:
                  dbw_state_machine.update_track_condition(TrackCondition::TC_DEFAULT);
                  break;
              }
            // handle counter 
            if(can_message->GetSignal("base_to_car_heartbeat")->GetResult() != old_rc_counter)
            {
              timeout_race_control = 0;
              old_rc_counter = can_message->GetSignal("base_to_car_heartbeat")->GetResult();
            }
          }
        }
        break;
      }
      case ID_TIRE_PRESSURE_FL:
      {        
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_PRESSURE_FL);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_VehicleSensorData.p_TirePressureFL_bar = can_message->GetSignal("FL_Tire_Pressure")->GetResult();
          }
        break;
      }
      case ID_TIRE_PRESSURE_FR:
      {       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_PRESSURE_FR);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_VehicleSensorData.p_TirePressureFR_bar = can_message->GetSignal("FR_Tire_Pressure")->GetResult();
          }
        break;
      }

      case ID_TIRE_PRESSURE_RL:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_PRESSURE_RL);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_VehicleSensorData.p_TirePressureRL_bar = can_message->GetSignal("RL_Tire_Pressure")->GetResult();    
          }
        
        break;
      }
      case ID_TIRE_PRESSURE_RR:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_PRESSURE_RR);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_VehicleSensorData.p_TirePressureRR_bar = can_message->GetSignal("RR_Tire_Pressure")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FL1:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FL1);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FL[0] = can_message->GetSignal("FL_Tire_Temp_01")->GetResult();
            TireTemp_FL[1] = can_message->GetSignal("FL_Tire_Temp_02")->GetResult();
            TireTemp_FL[2] = can_message->GetSignal("FL_Tire_Temp_03")->GetResult();
            TireTemp_FL[3] = can_message->GetSignal("FL_Tire_Temp_04")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FL2:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FL2);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FL[4] = can_message->GetSignal("FL_Tire_Temp_05")->GetResult();
            TireTemp_FL[5] = can_message->GetSignal("FL_Tire_Temp_06")->GetResult();
            TireTemp_FL[6] = can_message->GetSignal("FL_Tire_Temp_07")->GetResult();
            TireTemp_FL[7] = can_message->GetSignal("FL_Tire_Temp_08")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FL3:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FL3);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FL[8] = can_message->GetSignal("FL_Tire_Temp_09")->GetResult();
            TireTemp_FL[9] = can_message->GetSignal("FL_Tire_Temp_10")->GetResult();
            TireTemp_FL[10] = can_message->GetSignal("FL_Tire_Temp_11")->GetResult();
            TireTemp_FL[11] = can_message->GetSignal("FL_Tire_Temp_12")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FL4:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FL4);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FL[12] = can_message->GetSignal("FL_Tire_Temp_13")->GetResult();
            TireTemp_FL[13] = can_message->GetSignal("FL_Tire_Temp_14")->GetResult();
            TireTemp_FL[14] = can_message->GetSignal("FL_Tire_Temp_15")->GetResult();
            TireTemp_FL[15] = can_message->GetSignal("FL_Tire_Temp_16")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FR1:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FR1);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FR[0] = can_message->GetSignal("FR_Tire_Temp_01")->GetResult();
            TireTemp_FR[1] = can_message->GetSignal("FR_Tire_Temp_02")->GetResult();
            TireTemp_FR[2] = can_message->GetSignal("FR_Tire_Temp_03")->GetResult();
            TireTemp_FR[3] = can_message->GetSignal("FR_Tire_Temp_04")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FR2:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FR2);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FR[4] = can_message->GetSignal("FR_Tire_Temp_05")->GetResult();
            TireTemp_FR[5] = can_message->GetSignal("FR_Tire_Temp_06")->GetResult();
            TireTemp_FR[6] = can_message->GetSignal("FR_Tire_Temp_07")->GetResult();
            TireTemp_FR[7] = can_message->GetSignal("FR_Tire_Temp_08")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FR3:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FR3);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FR[8] = can_message->GetSignal("FR_Tire_Temp_09")->GetResult();
            TireTemp_FR[9] = can_message->GetSignal("FR_Tire_Temp_10")->GetResult();
            TireTemp_FR[10] = can_message->GetSignal("FR_Tire_Temp_11")->GetResult();
            TireTemp_FR[11] = can_message->GetSignal("FR_Tire_Temp_12")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_FR4:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_FR4);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_FR[12] = can_message->GetSignal("FR_Tire_Temp_13")->GetResult();
            TireTemp_FR[13] = can_message->GetSignal("FR_Tire_Temp_14")->GetResult();
            TireTemp_FR[14] = can_message->GetSignal("FR_Tire_Temp_15")->GetResult();
            TireTemp_FR[15] = can_message->GetSignal("FR_Tire_Temp_16")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RL1:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RL1);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RL[0] = can_message->GetSignal("RL_Tire_Temp_01")->GetResult();
            TireTemp_RL[1] = can_message->GetSignal("RL_Tire_Temp_02")->GetResult();
            TireTemp_RL[2] = can_message->GetSignal("RL_Tire_Temp_03")->GetResult();
            TireTemp_RL[3] = can_message->GetSignal("RL_Tire_Temp_04")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RL2:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RL2);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RL[4] = can_message->GetSignal("RL_Tire_Temp_05")->GetResult();
            TireTemp_RL[5] = can_message->GetSignal("RL_Tire_Temp_06")->GetResult();
            TireTemp_RL[6] = can_message->GetSignal("RL_Tire_Temp_07")->GetResult();
            TireTemp_RL[7] = can_message->GetSignal("RL_Tire_Temp_08")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RL3:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RL3);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RL[8] = can_message->GetSignal("RL_Tire_Temp_09")->GetResult();
            TireTemp_RL[9] = can_message->GetSignal("RL_Tire_Temp_10")->GetResult();
            TireTemp_RL[10] = can_message->GetSignal("RL_Tire_Temp_11")->GetResult();
            TireTemp_RL[11] = can_message->GetSignal("RL_Tire_Temp_12")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RL4:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RL4);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RL[12] = can_message->GetSignal("RL_Tire_Temp_13")->GetResult();
            TireTemp_RL[13] = can_message->GetSignal("RL_Tire_Temp_14")->GetResult();
            TireTemp_RL[14] = can_message->GetSignal("RL_Tire_Temp_15")->GetResult();
            TireTemp_RL[15] = can_message->GetSignal("RL_Tire_Temp_16")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RR1:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RR1);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RR[0] = can_message->GetSignal("RR_Tire_Temp_01")->GetResult();
            TireTemp_RR[1] = can_message->GetSignal("RR_Tire_Temp_02")->GetResult();
            TireTemp_RR[2] = can_message->GetSignal("RR_Tire_Temp_03")->GetResult();
            TireTemp_RR[3] = can_message->GetSignal("RR_Tire_Temp_04")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RR2:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RR2);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RR[4] = can_message->GetSignal("RR_Tire_Temp_05")->GetResult();
            TireTemp_RR[5] = can_message->GetSignal("RR_Tire_Temp_06")->GetResult();
            TireTemp_RR[6] = can_message->GetSignal("RR_Tire_Temp_07")->GetResult();
            TireTemp_RR[7] = can_message->GetSignal("RR_Tire_Temp_08")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RR3:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RR3);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RR[8] = can_message->GetSignal("RR_Tire_Temp_09")->GetResult();
            TireTemp_RR[9] = can_message->GetSignal("RR_Tire_Temp_10")->GetResult();
            TireTemp_RR[10] = can_message->GetSignal("RR_Tire_Temp_11")->GetResult();
            TireTemp_RR[11] = can_message->GetSignal("RR_Tire_Temp_12")->GetResult();
          }
        break;
      }
      case ID_TIRE_TEMP_RR4:
      {
       
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_TIRE_TEMP_RR4);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            TireTemp_RR[12] = can_message->GetSignal("RR_Tire_Temp_13")->GetResult();
            TireTemp_RR[13] = can_message->GetSignal("RR_Tire_Temp_14")->GetResult();
            TireTemp_RR[14] = can_message->GetSignal("RR_Tire_Temp_15")->GetResult();
            TireTemp_RR[15] = can_message->GetSignal("RR_Tire_Temp_16")->GetResult();
          }
        break;
      }
      case ID_WHEEL_POTENTIOMETER:
      {
          NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_WHEEL_POTENTIOMETER);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_VehicleSensorData.s_DamperPotsFL_mm = can_message->GetSignal("wheel_potentiometer_FL")->GetResult();
            model->inputs->in_VehicleSensorData.s_DamperPotsFR_mm = can_message->GetSignal("wheel_potentiometer_FR")->GetResult();
            model->inputs->in_VehicleSensorData.s_DamperPotsRL_mm = can_message->GetSignal("wheel_potentiometer_RL")->GetResult();
            model->inputs->in_VehicleSensorData.s_DamperPotsRR_mm = can_message->GetSignal("wheel_potentiometer_RR")->GetResult();
          }
        break;
      }
      case ID_DIAGNOSTIC_REPORT:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_DIAGNOSTIC_REPORT);
          if (in_can_msg_->dlc >= can_message->GetDlc()) {
            can_message->SetFrame(in_can_msg_);   
            model->inputs->in_ExternalDebug.DiagSD = can_message->GetSignal("sd_system_warning")->GetResult() + 
                                                    2*can_message->GetSignal("sd_system_failure")->GetResult() + 
                                                    4*can_message->GetSignal("sd_brake_warning1")->GetResult() + 
                                                    8*can_message->GetSignal("sd_brake_warning2")->GetResult() + 
                                                    16*can_message->GetSignal("sd_brake_warning3")->GetResult() +
                                                    32*can_message->GetSignal("sd_steer_warning1")->GetResult() +
                                                    64*can_message->GetSignal("sd_steer_warning2")->GetResult() +
                                                    128*can_message->GetSignal("sd_steer_warning3")->GetResult();
            model->inputs->in_ExternalDebug.DiagMotec = can_message->GetSignal("motec_warning")->GetResult();
            model->inputs->in_ExternalDebug.DiagOther = can_message->GetSignal("est1_oos_front_brk")->GetResult() + 
                                                    2*can_message->GetSignal("est2_oos_rear_brk")->GetResult() + 
                                                    4*can_message->GetSignal("est3_low_eng_speed")->GetResult() + 
                                                    8*can_message->GetSignal("est4_sd_comms_loss")->GetResult() + 
                                                    16*can_message->GetSignal("est5_motec_comms_loss")->GetResult() +
                                                    32*can_message->GetSignal("est6_sd_ebrake")->GetResult() +
                                                    64*can_message->GetSignal("adlink_hb_lost")->GetResult() +
                                                    128*can_message->GetSignal("rc_lost")->GetResult();
            // Handling diagnostics errors 
            if (can_message->GetSignal("est1_oos_front_brk")->GetResult() || can_message->GetSignal("est2_oos_rear_brk")->GetResult() ||
            can_message->GetSignal("est3_low_eng_speed")->GetResult() || can_message->GetSignal("est4_sd_comms_loss")->GetResult() || 
            can_message->GetSignal("est5_motec_comms_loss")->GetResult() || can_message->GetSignal("est6_sd_ebrake")->GetResult())
            {
              diagnostics_report_error = true;
            }         
          }
        break;
      }
    }
  }
#endif
}


void ControlHandler::send_steering()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  can_message = dbc_av21_.GetMessageById(ID_STEERING_CMD);
  // RCLCPP_INFO(this->get_logger(), "Send Steering: %lf", steering);
  can_message->GetSignal("steering_motor_ang_cmd")->SetResult(steering);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void ControlHandler::send_accel_pedal()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  can_message = dbc_av21_.GetMessageById(ID_ACCELERATOR_PEDAL_CMD);

  // RCLCPP_INFO(this->get_logger(), "Send Accel: %lf", acc_pedal);
  can_message->GetSignal("acc_pedal_cmd")->SetResult(acc_pedal);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void ControlHandler::send_brake()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  can_message = dbc_av21_.GetMessageById(ID_BRAKE_CMD);

  // RCLCPP_INFO(this->get_logger(), "Send Brake: %lf", brake);
  can_message->GetSignal("brake_pressure_cmd")->SetResult(brake);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);

}

void ControlHandler::send_gear()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  can_message = dbc_av21_.GetMessageById(ID_GEAR_CMD);

  // RCLCPP_INFO(this->get_logger(), "Send gear: %lf", gear_cmd);
  can_message->GetSignal("desired_gear")->SetResult(gear_cmd);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void ControlHandler::send_ct_report()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  can_message = dbc_av21_.GetMessageById(ID_CT_REPORT);

  can_message->GetSignal("ct_state")->SetResult(ct_state);
  can_message->GetSignal("track_cond_ack")->SetResult(track_flag_ack);
  can_message->GetSignal("veh_sig_ack")->SetResult(veh_flag_ack);
  can_message->GetSignal("ct_state_rolling_counter")->SetResult(rolling_cnt);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void ControlHandler::logging()
{
  // get logging data
  real_T* data = model->outputs->out_Debug;
  // build logging string
  log_line.data = "";
  log_line.data += std::to_string(time);
  log_line.data += ";";
  for(int i = 0; i < N_DEBUG_SIGNALS; i++)
  {
    log_line.data += std::to_string(data[i]);
    log_line.data += ";";
  }
  log_line.data += "\n";
  pub_debug->publish(log_line);

  // do the same for the slow logging file
  data = model->outputs->out_Debug_Slow;
  log_line_slow.data = "";
  // check if the slow sample rate was executed via internal logging counter
  if(data[0] != old_logging_id)
  {
    log_line_slow.data += std::to_string(time);
    log_line_slow.data += ";";
    // log all signals
    for(int i = 0; i < N_DEBUG_SLOW_SIGNALS; i++)
    {
      log_line_slow.data += std::to_string(data[i]);
      log_line_slow.data += ";";
    }
    log_line_slow.data += "\n";
    pub_debug_slow->publish(log_line_slow);
    // store logging counterstatus_local_planner, status_prediction
    old_logging_id = data[0];
  }

  // do the same for the latency logging file
  log_line_latency.data = "";
  log_line_latency.data += std::to_string(latency_id);
  log_line_latency.data += ";";
  log_line_latency.data += std::to_string(latency_t_receive);
  log_line_latency.data += ";";
  log_line_latency.data += std::to_string(latency_t_send);
  log_line_latency.data += ";";
  log_line_latency.data += std::to_string(model->outputs->out_TUMModeControl.TUMVehicleState);
  log_line_latency.data += "\n";
  pub_debug_latency->publish(log_line_latency);
}

int ControlHandler::set_tum_parameter(std::string param, double value)
{
  int idx = find_parameter_idx(param);
  if(idx >= 0)
  {
    // load data type index
    int dt_idx = find_parameter_dt(param);
    // define data type labels used in generated code from simulink
    std::string double_str = "real_T";
    std::string boolean_str = "boolean_T";
    // check if the data type index is matched to a double variable in the data type map
    if(double_str.compare(model->DataMapInfo.mmi.staticMap->Maps.dataTypeMap[dt_idx].mwDataName) == 0)
    {
      // get pointer to value
      double* ptr = (double*)model->DataMapInfo.mmi.InstanceMap.dataAddrMap[idx];
      // write value
      *ptr = (double)value;
      RCLCPP_INFO(this->get_logger(), "Found %s and updated to %5.2f ...", param.c_str(), value);
      return idx;
    }
    // check if the data type index is matched to a boolean variable in the data type map
    if(boolean_str.compare(model->DataMapInfo.mmi.staticMap->Maps.dataTypeMap[dt_idx].mwDataName) == 0)
    {
      // get pointer to value
      unsigned char* ptr = (unsigned char*)model->DataMapInfo.mmi.InstanceMap.dataAddrMap[idx];
      // write value
      *ptr = (unsigned char)value;
      RCLCPP_INFO(this->get_logger(), "Found %s and updated to %5.2f ...", param.c_str(), value);
      return idx;
    }
  }
  // if no valid index was found return failure
  return -1;
}

double ControlHandler::get_tum_parameter(std::string param)
{
  int idx = find_parameter_idx(param);
  if(idx >= 0)
  {
    // load data type index
    int dt_idx = find_parameter_dt(param);
    // define data type labels used in generated code from simulink
    std::string double_str = "real_T";
    std::string boolean_str = "boolean_T";
    // check if the data type index is matched to a double variable in the data type map
    if(double_str.compare(model->DataMapInfo.mmi.staticMap->Maps.dataTypeMap[dt_idx].mwDataName) == 0)
    {
      // get pointer to value
      double* ptr = (double*)model->DataMapInfo.mmi.InstanceMap.dataAddrMap[idx];
      return *ptr;
    }
    // check if the data type index is matched to a boolean variable in the data type map
    if(boolean_str.compare(model->DataMapInfo.mmi.staticMap->Maps.dataTypeMap[dt_idx].mwDataName) == 0)
    {
      // get pointer to value
      unsigned char* ptr = (unsigned char*)model->DataMapInfo.mmi.InstanceMap.dataAddrMap[idx];
      return *ptr;
    }
  }
  // if no valid index was found return failure
  return 0;
}

int ControlHandler::find_parameter_idx(std::string param)
{
  // find parameter in data map
  int nParam = model->DataMapInfo.mmi.staticMap->Params.numModelParameters;
  for(int i = 0; i<nParam; i++)
  {
    // check if requested signal matches value
    if(param.compare(model->DataMapInfo.mmi.staticMap->Params.modelParameters[i].varName) == 0)
    {
      // get address index
      int dataidx = model->DataMapInfo.mmi.staticMap->Params.modelParameters[i].addrMapIndex;
      // stop execution if one successful match was found
      return dataidx;
    }
  }
  // if there was no match, return failure
  return -1;
}

int ControlHandler::find_parameter_dt(std::string param)
{
  // find parameter in data map
  int nParam = model->DataMapInfo.mmi.staticMap->Params.numModelParameters;
  for(int i = 0; i<nParam; i++)
  {
    // check if requested signal matches value
    if(param.compare(model->DataMapInfo.mmi.staticMap->Params.modelParameters[i].varName) == 0)
    {
      // get data index
      int dataidx = model->DataMapInfo.mmi.staticMap->Params.modelParameters[i].dataTypeIndex;
      // stop execution if one successful match was found
      return dataidx;
    }
  }
  // if there was no match, return failure
  return -1;
}

void ControlHandler::print_status()
{
  // read vehicle state information
  int TUMVehicleState = model->outputs->out_TUMModeControl.TUMVehicleState;
  double v_mps = model->outputs->out_VehicleDynamicState.v_mps;
  // only print the general status update every 20th iteration
  print_cnt++;
  if (print_cnt == 50)
  {
    print_cnt = 0;
    // print software status
    RCLCPP_INFO(this->get_logger(), "RCNN: %2d | CLST: %2d | RAD: %2d",
    status_objects_pointrcnn, status_objects_clustering, status_objects_radar);
    RCLCPP_INFO(this->get_logger(), "LOC: %2d | LTPL: %2d | PRED: %2d",
    status_map_loc, status_local_planner, status_prediction);
    RCLCPP_INFO(this->get_logger(), "CT: %2d | Raptor: %2d | Control: %2d",
    dbw_state_machine.get_ct_state(), dbw_state_machine.get_sys_state(), TUMVehicleState);
    // print other vehicle information
    RCLCPP_INFO(this->get_logger(), "Velocity: %3.1fmps | Lap %2d | global s: %3.1f",
      v_mps, current_lap, current_global_s);
    if(manual_e_brake)
    {
      RCLCPP_ERROR(this->get_logger(), "Manual mode E-Brake triggered");
    }
  }
  else
  {
    // print software status
    RCLCPP_DEBUG(this->get_logger(), "RCNN: %2d | CLST: %2d | RAD: %2d",
    status_objects_pointrcnn, status_objects_clustering, status_objects_radar);
    RCLCPP_DEBUG(this->get_logger(), "LOC: %2d | LTPL: %2d | PRED: %2d",
    status_map_loc, status_local_planner, status_prediction);
    RCLCPP_DEBUG(this->get_logger(), "CT: %2d | Raptor: %2d | Control: %2d",
    dbw_state_machine.get_ct_state(), dbw_state_machine.get_sys_state(), TUMVehicleState);
    // print other vehicle information
    RCLCPP_DEBUG(this->get_logger(), "Velocity: %3.1fmps | Lap %2d | global s: %3.1f",
      v_mps, current_lap, current_global_s);
  }

  // parse error codes
  real_T* data = model->outputs->out_Debug;
  int error_code_new = data[0];
  // if there is an error and something changed, print to console)
  if (error_code_new != 0 && error_code_old != error_code_new)
  {
    // store error code for next iteration
    error_code_old = error_code_new;
    // parse and print errors
    RCLCPP_ERROR(this->get_logger(), "Error code: %4d | see decoding below for details: ", error_code_new);
    if ((error_code_new >> 0) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Performance trajectory not OK");
    }
    if ((error_code_new >> 1) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Emergency trajectory not OK");
    }
    if ((error_code_new >> 2) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Path deviation too high (below critical)");
    }
    if ((error_code_new >> 3) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Path deviation too high (above critical)");
    }
    if ((error_code_new >> 4) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory communication not OK");
    }
    if ((error_code_new >> 5) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Module communication not OK");
    }
    if ((error_code_new >> 6) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller is not OK");
    }
    if ((error_code_new >> 7) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is too short for TMPC");
    }
    if ((error_code_new >> 8) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "Vehicle unstable");
    }
    if ((error_code_new >> 9) & 1LL)
    {
        RCLCPP_ERROR(this->get_logger(), "External module not OK");
    }
    RCLCPP_ERROR(this->get_logger(), "The current trajectory ID is: %5d", current_traj_id);
  }
}

void ControlHandler::on_sub_status_objects_pointrcnn(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_objects_pointrcnn = msg->status;
  // reset timeout values (time since last message)
  timeout_objects_pointrcnn = 0;
}

void ControlHandler::on_sub_status_objects_clustering(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_objects_clustering = msg->status;
  // reset timeout values (time since last message)
  timeout_objects_clustering = 0;
}

void ControlHandler::on_sub_status_objects_radar(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_objects_radar = msg->status;
  // reset timeout values (time since last message)
  timeout_objects_radar = 0;
}

void ControlHandler::on_sub_status_prediction(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_prediction = msg->status;
  // reset timeout values (time since last message)
  timeout_prediction = 0;
}

void ControlHandler::on_sub_status_local_planner(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_local_planner = msg->status;
  // reset timeout values (time since last message)
  timeout_local_planner = 0;
}

void ControlHandler::on_sub_status_map_loc(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_map_loc = msg->status;
  // reset timeout values (time since last message)
  timeout_map_loc = 0;
}

void ControlHandler::on_sub_status_preprocessing(const tum_msgs::msg::TUMModuleStatus::SharedPtr msg)
{
  // write current status for evaluation in main loop
  status_preprocessing = msg->status;
  // reset timeout values (time since last message)
  timeout_preprocessing = 0;
}

void ControlHandler::on_sub_trajectory(const tum_msgs::msg::TUMTrajectory::SharedPtr msg)
{
  // reset timeout
  latency_t_receive = ros_time.now().nanoseconds();
  timeout_trajectory_performance = 0;
  // process trajectory
  model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.LapCnt = msg->lap_cnt;
  model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.TrajCnt = msg->traj_cnt;
  for (int i = 0; i < 50; i++)
  {
    // update simulink model
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.s_loc_m[i] = msg->s_local_m[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.s_glob_m[i] = msg->s_global_m[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.x_m[i] = msg->x_m[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.y_m[i] = msg->y_m[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.psi_rad[i] = msg->psi_rad[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.kappa_radpm[i] = msg->kappa_radpm[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.v_mps[i] = msg->v_mps[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.ax_mps2[i] = msg->ax_mps2[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.banking_rad[i] = msg->bank_rad[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.ax_lim_mps2[i] = msg->ax_max_mps2[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.ay_lim_mps2[i] = msg->ay_max_mps2[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.tube_l_m[i] = msg->tube_l_m[i];
    model->inputs->in_TrajectoryPlanning.PerformanceTrajectory.tube_r_m[i] = msg->tube_r_m[i];
    // update driving tube visualization 
    double veh_width = 1;
    x_left_driving_tube_m[i] = msg->x_m[i] - (msg->tube_l_m[i] + veh_width) * cos(msg->psi_rad[i]);
    y_left_driving_tube_m[i] = msg->y_m[i] - (msg->tube_l_m[i] + veh_width) * sin(msg->psi_rad[i]);
    x_right_driving_tube_m[i] = msg->x_m[i] + (msg->tube_r_m[i] + veh_width) * cos(msg->psi_rad[i]);
    y_right_driving_tube_m[i] = msg->y_m[i] + (msg->tube_r_m[i] + veh_width) * sin(msg->psi_rad[i]);
    // latency id
    latency_id = msg->latency_id;
  }
  // write values to buffers for printing
  current_global_s = msg->s_global_m[0];
  current_lap = msg->lap_cnt;
  current_traj_id = msg->traj_cnt;
}

void ControlHandler::on_sub_em_trajectory(const tum_msgs::msg::TUMTrajectory::SharedPtr msg)
{
  // reset timeout
  timeout_trajectory_emergency = 0;
  // process trajectory
  model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.LapCnt = msg->lap_cnt;
  model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.TrajCnt = msg->traj_cnt;
  for (int i = 0; i < 50; i++)
  {
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.s_loc_m[i] = msg->s_local_m[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.s_glob_m[i] = msg->s_global_m[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.x_m[i] = msg->x_m[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.y_m[i] = msg->y_m[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.psi_rad[i] = msg->psi_rad[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.kappa_radpm[i] = msg->kappa_radpm[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.v_mps[i] = msg->v_mps[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.ax_mps2[i] = msg->ax_mps2[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.banking_rad[i] = msg->bank_rad[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.ax_lim_mps2[i] = msg->ax_max_mps2[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.ay_lim_mps2[i] = msg->ay_max_mps2[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.tube_l_m[i] = msg->tube_l_m[i];
    model->inputs->in_TrajectoryPlanning.EmergencyTrajectory.tube_r_m[i] = msg->tube_r_m[i];
  }
  // write values to buffers for printing
  current_global_s = msg->s_global_m[0];
  current_lap = msg->lap_cnt;
  current_traj_id = msg->traj_cnt;
}

void ControlHandler::on_sub_map_ref(const tum_msgs::msg::TUMMapReference::SharedPtr msg)
{
  // update map reference (only available in SIL)
  #ifndef CONTROL
  model->inputs->in_MapReference.InitialState[0] = msg->initial_x_m;
  model->inputs->in_MapReference.InitialState[1] = msg->initial_y_m;
  model->inputs->in_MapReference.InitialState[2] = msg->initial_psi_rad;
  // this is to acknowledge that the state has been set
  model->inputs->in_MapReference.InitialState[3] = 1;
  model->inputs->in_MapReference.GPSReference[0] = msg->gps_lat_deg;
  model->inputs->in_MapReference.GPSReference[1] = msg->gps_long_deg;
  model->inputs->in_MapReference.GPSReference[2] = msg->gps_height_m;
  #endif
  // set initialized to true such that software is ready to go
  initialized = true;
}

void ControlHandler::on_sub_lidar_loc(const tum_msgs::msg::TUMLocalization::SharedPtr msg)
{
  // only available in control mode 
  #ifdef CONTROL 
  model->inputs->in_VehicleSensorData.x_Loc3_m = 0;
  model->inputs->in_VehicleSensorData.y_Loc3_m = 0;
  model->inputs->in_VehicleSensorData.psi_YawAngleLoc3_rad = 0;
  model->inputs->in_VehicleSensorData.valid_Loc3_b = 0;
  // use internal control time as this is only a trigger for updating the sensor fusion
  // and not actually used for delay compensation
  model->inputs->in_VehicleSensorData.t_EstimateLoc3_s = time; 
  #endif
}

void ControlHandler::on_sub_control_param(const tum_msgs::msg::TUMControlParam::SharedPtr msg)
{
  // this callback uses the simulink parameter API to set parameters
  set_tum_parameter(msg->param, msg->value);
}

std::vector<double> ControlHandler::llh_to_ecf(std::vector<double> llh)
{
  // This function returns the x, y, and z earth centered fixed (ECF) coordinates of the point specified by llh [lat lon hgt]. Note that longitude is positive east of the Greenwich meridian.
  
  // Setup WGS-84 constants
  const double a = 6378137.0; // meters
  const double f = 1.0/298.257223563;

  double lat = llh[0];
  double lon = llh[1];
  double hgt = llh[2];

  double s_lat = sin(lat);  
  double N = a/sqrt(1 - f * (2-f) * pow(s_lat, 2.0));
  double Nplushgtclat = (N + hgt) * cos(lat);

  double x = Nplushgtclat * cos(lon);
  double y = Nplushgtclat * sin(lon);
  double z = (pow(1-f,2.0) * N + hgt) * s_lat;
  std::vector<double> ecf = {x, y, z}; 

  return ecf;
}

void ControlHandler::get_dircos_forward(double A, int MatrixFlavor, std::vector<std::vector<double>> &DC)
{
  // Fills a direction cosine matrix defined by positive right-hand rule Euler angles that transforms from an INS type basis to a body type basis.
  const int YAW_TYPE = 1;
  const int PITCH_TYPE = 2;
  const int ROLL_TYPE = 3;

  double cosA = cos(A);
  double sinA = sin(A);
 
  switch (MatrixFlavor)
  {
  case YAW_TYPE:
    DC[0][0] = cosA;
    DC[0][1] = sinA;
    DC[1][0] = -sinA;
    DC[1][1] = cosA;
    DC[2][2] = 1;
    break;

  case PITCH_TYPE:
    DC[0][0] = cosA;
    DC[0][2] = -sinA;
    DC[1][1] = 1;
    DC[2][0] = sinA;
    DC[2][2] = cosA;
    break;
    
  case ROLL_TYPE:
    DC[0][0] = 1;
    DC[1][1] = cosA;
    DC[1][2] = sinA;
    DC[2][1] = -sinA;
    DC[2][2] = cosA;
    break;
  
  default:
    break;
  } 

  
}


std::vector<double> ControlHandler::ecf_to_uvw(std::vector<double> ecf)
{
  // This function will rotate ECF coordinates into UVW coordinates: X axis (U axis) is colinear with longitude of origin
  int YAW_TYPE = 1;

  std::vector<std::vector<double>> DC = {{0,0,0},{0,0,0},{0,0,0}};
  get_dircos_forward(origin_conv[1], YAW_TYPE, DC);

  std::vector<double> uvw = {0,0,0};

  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j <  3; j++)
    {
      uvw[i] += DC[i][j] * ecf[j];
      
    }
    
  }

  return uvw;

}

void ControlHandler::uvw_to_tcs(std::vector<double> uvw, std::vector<double> &tcs)
{
  // This function will convert a position vector from UVW to TCS coordinates relative to origin

  // Transform Origin to ECF and UVW
  std::vector<double> origin_ECF = {0,0,0};
  std::vector<double> origin_UVW = {0,0,0};
  origin_ECF = llh_to_ecf(origin_conv);
  origin_UVW = ecf_to_uvw(origin_ECF);
  // Define Rotation Types
  int YAW_TYPE = 1;
  int PITCH_TYPE = 2;
  int ROLL_TYPE = 3;
  std::vector<std::vector<double>> DC1 = {{0,0,0},{0,0,0},{0,0,0}};
  std::vector<std::vector<double>> DC2= {{0,0,0},{0,0,0},{0,0,0}};
  std::vector<std::vector<double>> DC3= {{0,0,0},{0,0,0},{0,0,0}};
  get_dircos_forward(M_PI/2, ROLL_TYPE, DC1);
  get_dircos_forward(M_PI/2, PITCH_TYPE, DC2);
  get_dircos_forward(- origin_conv[0], ROLL_TYPE, DC3);

  std::vector<std::vector<double>> DC21 = {{0,0,0},{0,0,0},{0,0,0}};
  std::vector<std::vector<double>> DC31 = {{0,0,0},{0,0,0},{0,0,0}};

  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)

    {
      double product = 0; 
      for (size_t k = 0; k < 3; k++)
      {
        product += DC2[i][k] * DC1[k][j];
      }
      DC21[i][j] = product;
    }
    
  }
  
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    { double product = 0;
      for (size_t k = 0; k < 3; k++)
      {
        product += DC3[i][k] * DC21[k][j];
      }
      
      DC31[i][j] = product;
    }
    
  }
  std::vector<double> tcs_offset = {0,0,0};
  tcs_offset[0] = uvw[0] - origin_UVW[0];
  tcs_offset[1] = uvw[1] - origin_UVW[1];
  tcs_offset[2] = uvw[2] - origin_UVW[2];

  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j <  3; j++)
    {
      tcs[i] += DC31[i][j] * tcs_offset[j];
    }
    
  }

}


void ControlHandler::llh_to_tcs(std::vector<double> llh, std::vector<double> &tcs)
{
  // This function returns the x, y, and z topocentric (TCS) coordinates of the point specified by llh [lat lon hgt], relative to the input origin [lat lon alt]
  // convert lat, lon and origin lat, lon from deg to rad
  std::vector<double> llh_conv = {0,0,0};
  llh_conv[0] = llh[0] / (360/(2 * M_PI));
  llh_conv[1] = llh[1] / (360/(2 * M_PI));
  llh_conv[2] = llh[2];

  // transform from llh to ecf
  std::vector<double> ecf = llh_to_ecf(llh_conv);

  // transform from ecf to uvw
  std::vector<double> uvw = ecf_to_uvw(ecf);

  // transform uvw to tcs
  uvw_to_tcs(uvw, tcs);

}


void ControlHandler::on_sub_gps_1(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
{
  // update position (only available in control)
  #ifdef CONTROL
  // reset timeout
  // corrections are done for UTM to local
  double lat = msg->lat;
  double lon = msg->lon;
  double height = msg->hgt;

  std::vector<double> llh = {lat, lon , height};
  std::vector<double> tcs = {0,0,0};

  ControlHandler::llh_to_tcs(llh, tcs);

  double x_m = tcs[0];
  double y_m = tcs[1];

  // transform from antenna to receiver 
  double heading_rad = model->outputs->out_VehicleDynamicState.Pos.psi_rad;
  double x_m_recv = x_m + 0*cos(heading_rad) + 1.571*sin(heading_rad);
  double y_m_recv = y_m + 0*sin(heading_rad) - 1.571*cos(heading_rad);

  model->inputs->in_VehicleSensorData.x_Loc1_m = x_m_recv;
  model->inputs->in_VehicleSensorData.y_Loc1_m = y_m_recv;
  model->inputs->in_VehicleSensorData.accuracy_Loc1[0] = msg->lat_stdev;
  model->inputs->in_VehicleSensorData.accuracy_Loc1[1] = msg->lon_stdev;
  model->inputs->in_VehicleSensorData.accuracy_Loc1[2] = msg->pos_type.type;
  if(msg->lat_stdev < 0.2 && msg->lon_stdev < 0.2 && msg->pos_type.type == 50)
  {
    model->inputs->in_VehicleSensorData.valid_Loc1_b = 1;
  }
  else
  {
    model->inputs->in_VehicleSensorData.valid_Loc1_b = 0;
  }
  // use internal control time as this is only a trigger for updating the sensor fusion
  // and not actually used for delay compensation
  model->inputs->in_VehicleSensorData.t_EstimateLoc1_s = time; 
  RCLCPP_DEBUG(this->get_logger(), "Received position data: ");
  RCLCPP_DEBUG(this->get_logger(), "x_m: %4.2f", x_m);
  RCLCPP_DEBUG(this->get_logger(), "y_m: %4.2f", y_m);
  #endif
}

void ControlHandler::on_sub_dual_antenna_1(const novatel_oem7_msgs::msg::DUALANTENNAHEADING::SharedPtr msg)
{
#ifdef CONTROL

if(!driven_once)
{
  RCLCPP_DEBUG(this->get_logger(), "dual_antenna_heading would be: %4.2f", -6.28*(msg->heading-180)/360);
}

#endif

}

void ControlHandler::on_sub_vel_1(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
{
  #ifdef CONTROL
  // update velocity (only available in control)
  if(msg->hor_speed > 2)
  {
    // calculate heading compensation due to mounting of GPS antenna 
    double yaw_rate = model->outputs->out_VehicleDynamicState.dPsi_radps;
    double vx_mps = model->outputs->out_VehicleDynamicState.vx_mps;
    double heading_error = atan(yaw_rate*1.571/fmax(vx_mps, 3));
    model->inputs->in_VehicleSensorData.psi_YawAngleLoc1_rad = -6.28*(msg->trk_gnd)/360 - heading_error;
    driven_once = true;
    }
  else
  {
    if(!driven_once)
    {
      model->inputs->in_VehicleSensorData.psi_YawAngleLoc1_rad = 2.11185;
    }
  }
  #endif
}


void ControlHandler::on_sub_imu_1(const novatel_oem7_msgs::msg::RAWIMU::SharedPtr msg)
{
  // update IMU (only available in control)
  #ifdef CONTROL
  // reset timeout 
  timeout_imu_1 = 0;
  model->inputs->in_VehicleSensorData.ax_CoGIMU1_mps2 = -msg->linear_acceleration.y;
  model->inputs->in_VehicleSensorData.ay_CoGIMU1_mps2 = -msg->linear_acceleration.x;
  model->inputs->in_VehicleSensorData.az_CoGIMU1_mps2 = msg->linear_acceleration.z;
  model->inputs->in_VehicleSensorData.dPhi_RollRateIMU1_radps = msg->angular_velocity.y;
  model->inputs->in_VehicleSensorData.dTheta_PitchRateIMU1_radps = msg->angular_velocity.x;
  model->inputs->in_VehicleSensorData.dPsi_YawRateIMU1_radps = msg->angular_velocity.z;
  model->inputs->in_VehicleSensorData.valid_IMU1_b = 1;
  // RCLCPP_DEBUG(this->get_logger(), "Received IMU data");
  #endif
}

void ControlHandler::on_sub_gps_2(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
{
  // update position (only available in control)
  #ifdef CONTROL
  // reset timeout
  // corrections are done for UTM to local
  double lat = msg->lat;
  double lon = msg->lon;
  double height = msg->hgt;

  std::vector<double> llh = {lat, lon , height};
  std::vector<double> tcs = {0,0,0};

  ControlHandler::llh_to_tcs(llh, tcs);

  double x_m = tcs[0];
  double y_m = tcs[1];

  // transform from antenna to receiver 
  double heading_rad = model->outputs->out_VehicleDynamicState.Pos.psi_rad;
  double x_m_recv = x_m + 0.502*cos(heading_rad) + 0.457*sin(heading_rad);
  double y_m_recv = y_m + 0.502*sin(heading_rad) - 0.457*cos(heading_rad);

  model->inputs->in_VehicleSensorData.x_Loc2_m = x_m_recv;
  model->inputs->in_VehicleSensorData.y_Loc2_m = y_m_recv;
  model->inputs->in_VehicleSensorData.accuracy_Loc2[0] = msg->lat_stdev;
  model->inputs->in_VehicleSensorData.accuracy_Loc2[1] = msg->lon_stdev;
  model->inputs->in_VehicleSensorData.accuracy_Loc2[2] = msg->pos_type.type;
  if(msg->lat_stdev < 0.2 && msg->lon_stdev < 0.2 && msg->pos_type.type == 50)
  {
    model->inputs->in_VehicleSensorData.valid_Loc2_b = 1;
  }
  else
  {
    model->inputs->in_VehicleSensorData.valid_Loc2_b = 0;
  }
  // use internal control time as this is only a trigger for updating the sensor fusion
  // and not actually used for delay compensation
  model->inputs->in_VehicleSensorData.t_EstimateLoc2_s = time; 
  RCLCPP_DEBUG(this->get_logger(), "Received position data: ");
  RCLCPP_DEBUG(this->get_logger(), "x_m: %4.2f", x_m);
  RCLCPP_DEBUG(this->get_logger(), "y_m: %4.2f", y_m);
  #endif
}

void ControlHandler::on_sub_vel_2(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
{
  #ifdef CONTROL
  // update velocity (only available in control)
  if(msg->hor_speed > 2)
  {
    model->inputs->in_VehicleSensorData.psi_YawAngleLoc2_rad = -6.28*(msg->trk_gnd)/360;
  }
  if(!driven_once)
  {
    model->inputs->in_VehicleSensorData.psi_YawAngleLoc2_rad = 2.11185;
  }
  #endif
}


void ControlHandler::on_sub_imu_2(const novatel_oem7_msgs::msg::RAWIMU::SharedPtr msg)
{
  // update IMU (only available in control)
  #ifdef CONTROL
  // reset timeout 
  timeout_imu_2 = 0;
  model->inputs->in_VehicleSensorData.ax_CoGIMU2_mps2 = -msg->linear_acceleration.y;
  model->inputs->in_VehicleSensorData.ay_CoGIMU2_mps2 = -msg->linear_acceleration.x;
  model->inputs->in_VehicleSensorData.az_CoGIMU2_mps2 = msg->linear_acceleration.z;
  model->inputs->in_VehicleSensorData.dPhi_RollRateIMU2_radps = msg->angular_velocity.y;
  model->inputs->in_VehicleSensorData.dTheta_PitchRateIMU2_radps = msg->angular_velocity.x;
  model->inputs->in_VehicleSensorData.dPsi_YawRateIMU2_radps = msg->angular_velocity.z;
  model->inputs->in_VehicleSensorData.valid_IMU2_b = 1;
  // RCLCPP_DEBUG(this->get_logger(), "Received IMU data");
  #endif
}

void ControlHandler::on_sub_reset_hardware(const std_msgs::msg::Bool::SharedPtr msg)
{
  reset_hardware = msg->data;
}

void ControlHandler::telemetry()
{
  // handle telemetry callbacks with 10Hz
  telemetry_cnt++;
  if(telemetry_cnt >= 10)
  {
      // reset counter to limit frequency 
      telemetry_cnt = 0;
      // data frame structure for telemtry to basestation message
      // added 40 dummy variables for load testing
      #ifdef CONTROL
      double data_frame[69] = {track_flag_ack, veh_flag_ack, 
          ct_state, rolling_cnt, 
          model->inputs->in_VehicleSensorData.p_Fuel_kPa, 
          model->inputs->in_VehicleSensorData.T_TransmissionOilTemp_Celsius , 
          model->inputs->in_VehicleSensorData.p_EngineOil_kPa , 
          model->inputs->in_VehicleSensorData.T_CoolantTemp_Celsius,
          model->inputs->in_VehicleSensorData.omega_Engine_radps*60/6.28, 
          model->inputs->in_VehicleSensorData.GearEngaged, 
          battery_voltage, sys_state, 
          status_map_loc, status_objects_pointrcnn, status_objects_clustering, 
          status_objects_radar, status_local_planner, status_prediction, 
          model->outputs->out_TUMModeControl.TUMVehicleState,
          model->inputs->in_TrajectoryPlanning.Strategy_CommsStatus,
          model->inputs->in_TrajectoryPlanning.Trajectories_CommsStatus, 
          timeout_map_loc, timeout_objects_pointrcnn, timeout_objects_clustering, 
          timeout_objects_radar, timeout_local_planner, timeout_prediction,
          timeout_joystick, timeout_imu_1, timeout_imu_2, timeout_race_control, 
          current_lap, status_preprocessing, timeout_preprocessing, 
          model->outputs->out_VehicleDynamicState.Pos.x_m, 
          model->outputs->out_VehicleDynamicState.Pos.y_m, 
          model->outputs->out_VehicleDynamicState.Pos.psi_rad, 
          model->outputs->out_VehicleDynamicState.vx_mps, 
          model->outputs->out_VehicleDynamicState.beta_rad, 
          model->outputs->out_VehicleDynamicState.dPsi_radps, 
          model->outputs->out_VehicleDynamicState.ax_mps2, 
          model->outputs->out_VehicleDynamicState.ay_mps2, 
          model->inputs->in_VehicleSensorData.p_BrakeF_bar, 
          model->inputs->in_VehicleSensorData.p_BrakeR_bar, 
          model->inputs->in_VehicleSensorData.Delta_Wheel_rad, 
          model->inputs->in_VehicleSensorData.T_TireFL_Celsius[0],  
          model->inputs->in_VehicleSensorData.T_TireFL_Celsius[1],  
          model->inputs->in_VehicleSensorData.T_TireFL_Celsius[2],  
          model->inputs->in_VehicleSensorData.T_TireFL_Celsius[3], 
          model->inputs->in_VehicleSensorData.T_TireFR_Celsius[0],  
          model->inputs->in_VehicleSensorData.T_TireFR_Celsius[1],  
          model->inputs->in_VehicleSensorData.T_TireFR_Celsius[2],  
          model->inputs->in_VehicleSensorData.T_TireFR_Celsius[3], 
          model->inputs->in_VehicleSensorData.T_TireRL_Celsius[0],  
          model->inputs->in_VehicleSensorData.T_TireRL_Celsius[1],  
          model->inputs->in_VehicleSensorData.T_TireRL_Celsius[2],  
          model->inputs->in_VehicleSensorData.T_TireRL_Celsius[3], 
          model->inputs->in_VehicleSensorData.T_TireRR_Celsius[0],  
          model->inputs->in_VehicleSensorData.T_TireRR_Celsius[1],  
          model->inputs->in_VehicleSensorData.T_TireRR_Celsius[2],  
          model->inputs->in_VehicleSensorData.T_TireRR_Celsius[3], 
          model->outputs->out_LiveVisualization_Send.PathDeviation_d_m, 
          model->outputs->out_LiveVisualization_Send.PathDeviation_dot_d_mps, 
          model->outputs->out_LiveVisualization_Send.TubeLeft_d_m, 
          model->outputs->out_LiveVisualization_Send.TubeRight_d_m, 
          model->outputs->out_LiveVisualization_Send.LatAcc_FB_Dist_rad, 
          model->outputs->out_LiveVisualization_Send.LatAcc_FB_ay_rad, 
          model->outputs->out_LiveVisualization_Send.LatAcc_FFss_rad,
          model->outputs->out_ThrottlePosition_perc*100
          };
      #else
      double data_frame[69] = {track_flag_ack, veh_flag_ack, 
          ct_state, rolling_cnt, 
          0, 0, 0, 0, 0, 0, // no access to sensor data in sil
          battery_voltage, sys_state, 
          status_map_loc, status_objects_pointrcnn, status_objects_clustering, 
          status_objects_radar, status_local_planner, status_prediction, 
          0, 0, 0, 
          timeout_map_loc, timeout_objects_pointrcnn, timeout_objects_clustering, 
          timeout_objects_radar, timeout_local_planner, timeout_prediction,
          timeout_joystick, timeout_imu_1, timeout_imu_2, timeout_race_control, 
          current_lap, status_preprocessing, timeout_preprocessing, 
          0, 0, 0, 0, 0, 0, 0, 0, 
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      #endif
      boost::system::error_code err;
      send_telemetry_socket.send_to(buffer(data_frame, sizeof(data_frame)), send_telemetry_endpoint, 0, err);
  }
  // receive data from basestation
  boost::array<double, 54> recv_basestation_buffer;
  // read all messages until none is there anymore
  int i = 0;
  boost::system::error_code err_rcv;
  while(recv_basestation_socket.receive_from(boost::asio::buffer(recv_basestation_buffer), recv_basestation_endpoint, 0, err_rcv) != 0)
  {
      i++;
  }
  // only publish when a new topic was received
  // this is required such that the timeouts still work
  if (i > 0)
  {
      // handle joystick
      joystick_gear = recv_basestation_buffer[6];
      joystick_throttle = recv_basestation_buffer[5];
      joystick_brake = recv_basestation_buffer[4];
      joystick_steering = recv_basestation_buffer[3];
      // copy to logging in control mode
      #ifdef CONTROL
      model->inputs->in_VehicleSensorData.Delta_Manual_rad = 6.28*joystick_steering/360;
      model->inputs->in_VehicleSensorData.Throttle_Manual_perc = joystick_throttle/100;
      model->inputs->in_VehicleSensorData.Brake_Manual_bar = joystick_brake/100000;
      model->inputs->in_VehicleSensorData.Gear_Manual = joystick_gear;
      #endif
      // check if joystick data was actually updated 
      if(recv_basestation_buffer[0] != old_joystick_counter || basestation_only)
      {
        // reduce timeout by 50ms. This allows to detect if more than 50% of packages are lost
        // since the message is expected with 25ms update rate
        timeout_joystick -= 0.05;
        if(timeout_joystick < 0)
        {
          timeout_joystick = 0;
        }
        old_joystick_counter = recv_basestation_buffer[0];
      }  
      // check for emergency trigger
      if(recv_basestation_buffer[1] == 1)
      {
        dbw_state_machine.trigger_emergency();
      }

      // only use telemetry race control if we do not use mylaps
      if(!race_control_used)
      {
        // store track condition for acknowledge 
        timeout_race_control = 0;
        track_flag_ack = recv_basestation_buffer[8];
        veh_flag_ack = recv_basestation_buffer[9];
        switch (int(recv_basestation_buffer[8]))
        {
          case 1:
            dbw_state_machine.update_track_condition(TrackCondition::TC1_RED);
            break;
          case 2:
            dbw_state_machine.update_track_condition(TrackCondition::TC2_ORANGE);
            break;
          case 3:
            dbw_state_machine.update_track_condition(TrackCondition::TC3_YELLOW);
            break;
          case 4:
            dbw_state_machine.update_track_condition(TrackCondition::TC4_GREEN);
            break;
          case 255:
            dbw_state_machine.update_track_condition(TrackCondition::TC_DEFAULT);
            break;
        }
        // re-publish race control 
        msg_rc_to_ct.base_to_car_heartbeat = recv_basestation_buffer[7];
        msg_rc_to_ct.track_flag = recv_basestation_buffer[8];
        msg_rc_to_ct.veh_flag = recv_basestation_buffer[9];
        pub_race_control->publish(msg_rc_to_ct);
      }

      // handle TUM request
      // only publish if values are changing
      msg_tum_req.tum_req = recv_basestation_buffer[10];
      msg_tum_req.speed_limit = recv_basestation_buffer[11];
      msg_tum_req.gg_scale = recv_basestation_buffer[12];
      msg_tum_req.rolling_counter = recv_basestation_buffer[13];
      if(msg_tum_req.rolling_counter != tum_req_cnt_old)
      {
        pub_tum_request->publish(msg_tum_req);
        tum_req_cnt_old = msg_tum_req.rolling_counter;
      }
  }
}
