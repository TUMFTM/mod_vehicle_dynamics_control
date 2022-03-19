#include "RaptorEmulation.h"
#include <chrono>
#include <thread>
#include <algorithm>

RaptorEmulation::RaptorEmulation() : Node("RaptorEmulation"), send_cmd_socket(io_service),send_cmd_raptor_system_socket(io_service), recv_sensor_data_socket(io_service), recv_raptor_system_state_socket(io_service)
{
  // set real-time priority
  pid_t pid = getpid();
  std::string cmd = "chrt --rr -p 91 ";
  std::string pid_string = std::to_string(pid);
  std::string cmd_full = cmd + pid_string;
  system(cmd_full.c_str());

  wheel_fl_ = 0.0;
  wheel_fr_ = 0.0;
  wheel_rl_ = 0.0;
  wheel_rr_ = 0.0;
  steering_wheel_angle_ = 0.0;
  steering_counter_ = 0;
  front_brake_pressure_ = 0.0;
  rear_brake_pressure_ = 0.0;
  brake_counter_ = 0;
  pedal_output_ = 0.0;
  throttle_counter_ = 0;
  gear_data_ = 0;
  engine_rpm_ = 0;
  steering_cmd_ = 0.0;
  accelerator_pedal_cmd_ = 0.0;
  gear_cmd_ = 0;
  brake_cmd_ = 0.0;
  force_cmd_ = 0;
  sys_state_ = 255;
  ct_state_ = 255;
  track_cond_ack_ = 255;
  vs_state_ack_ = 1;
  purple_flag_ = false;
  track_flag = 255;
  veh_flag = 0;

  // define parameters
  this->declare_parameter<int>("vehicle_id", 1);
  int vehicle_id;
  this->get_parameter("vehicle_id", vehicle_id);

  this->declare_parameter<float>("steering_ratio", 19.5);
  this->get_parameter("steering_ratio", steering_ratio);

  this->declare_parameter<float>("rear_wheel_radius", 0.3118);
  this->get_parameter("rear_wheel_radius", rear_wheel_radius);

  this->declare_parameter<float>("front_wheel_radius", 0.301);
  this->get_parameter("front_wheel_radius", front_wheel_radius);

  this->declare_parameter<std::string>("dbc_file_path", "");
  dbc_av21_filename_ = "/dev_ws/src/mod_control/control/params/CAN1-INDY-V9.dbc";
  dbc_av21_ = NewEagle::DbcBuilder().NewDbc(dbc_av21_filename_.c_str());

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();


  sub_force = this->create_subscription<std_msgs::msg::Float32>(
      "raptor_dbw_interface/force_cmd", 1, std::bind(&RaptorEmulation::on_sub_force, this, std::placeholders::_1));
  
  sub_rc_to_ct = this->create_subscription<deep_orange_msgs::msg::RcToCt>(
      "simulink/rc_to_ct", qos, std::bind(&RaptorEmulation::on_sub_rc, this, std::placeholders::_1));
  sub_from_simu_can_ = this->create_subscription<can_msgs::msg::Frame>(
      "/to_can_bus", 20, std::bind(&RaptorEmulation::on_sub_from_can, this, std::placeholders::_1));

  in_can_msg_ = std::make_shared<can_msgs::msg::Frame>();

  // create publishers
  pub_to_simu_can_ = this->create_publisher<can_msgs::msg::Frame>("/from_can_bus", 20);
  // create timer to handle cyclic task
  timer_ = this->create_wall_timer(10ms, std::bind(&RaptorEmulation::pub_cmd_udp, this));

  // setup UDP interface
  send_cmd_ip = "10.0.0.20";
  // vehicle ID1 sends at port 10000
  send_cmd_port = 9999 + vehicle_id;
  send_cmd_socket.open(ip::udp::v4());
  send_cmd_endpoint = ip::udp::endpoint(ip::address::from_string(send_cmd_ip), send_cmd_port);
  RCLCPP_INFO(this->get_logger(), "Established connection to HIL with: %s:%u", send_cmd_ip.c_str(), send_cmd_port);

  send_cmd_raptor_system_ip = "10.0.0.20";
  send_cmd_raptor_system_port = 10009 + vehicle_id;
  send_cmd_raptor_system_socket.open(ip::udp::v4());
  send_cmd_raptor_system_endpoint = ip::udp::endpoint(ip::address::from_string(send_cmd_raptor_system_ip), send_cmd_raptor_system_port);
  RCLCPP_INFO(this->get_logger(), "Established connection to HIL with: %s:%u", send_cmd_raptor_system_ip.c_str(), send_cmd_raptor_system_port);

  // IP 10.0.0.11 is for vehicle ID 1
  recv_sensor_data_ip = "10.0.0." + std::to_string(10 + vehicle_id);
  recv_sensor_data_port = 11002;
  recv_sensor_data_socket.open(ip::udp::v4());

  // use IP from which data is sent not local IP
  recv_sensor_data_endpoint = ip::udp::endpoint(ip::address::from_string(recv_sensor_data_ip), recv_sensor_data_port);
  recv_sensor_data_socket.bind(recv_sensor_data_endpoint);
  recv_sensor_data_socket.non_blocking(true);

  // system state from raptor simulink model
  recv_raptor_system_state_ip = "10.0.0." + std::to_string(10 + vehicle_id);
  recv_raptor_system_state_port = 11100;
  recv_raptor_system_state_socket.open(ip::udp::v4());

  // use IP from which data is sent not local IP
  recv_raptor_system_state_endpoint = ip::udp::endpoint(ip::address::from_string(recv_raptor_system_state_ip), recv_raptor_system_state_port);
  recv_raptor_system_state_socket.bind(recv_raptor_system_state_endpoint);
  recv_raptor_system_state_socket.non_blocking(true);

  RCLCPP_INFO(this->get_logger(), "Established connection to HIL with: %s:%u", recv_sensor_data_ip.c_str(), recv_sensor_data_port);
  RCLCPP_INFO(this->get_logger(), "Established connection to Raptor Model with: %s:%u", recv_raptor_system_state_ip.c_str(), recv_raptor_system_state_port);
}

RaptorEmulation::~RaptorEmulation()
{
  send_cmd_socket.close();
}

void RaptorEmulation::on_sub_force(const std_msgs::msg::Float32::SharedPtr msg)
{
  force_cmd_ = msg->data;
}

void RaptorEmulation::on_sub_rc(const deep_orange_msgs::msg::RcToCt::SharedPtr msg)
{
  track_flag = msg->track_flag;
  veh_flag = msg->veh_flag;

}

void RaptorEmulation::pub_cmd_udp()
{
  RCLCPP_DEBUG(this->get_logger(), "Start publishing ...");
  int vehicle_id;
  this->get_parameter("vehicle_id", vehicle_id);

  // publish vehicle commands to HIL
  boost::system::error_code err;

  // data frame structure:
  // SteeringAngle, TractionForce, ThrottlePosition, GearRequest, BrakePressure
  double data_frame[5] = {steering_cmd_, force_cmd_, accelerator_pedal_cmd_, gear_cmd_, brake_cmd_};
  send_cmd_socket.send_to(buffer(data_frame, sizeof(data_frame)), send_cmd_endpoint, 0, err);

  RCLCPP_DEBUG(this->get_logger(), "Sent the following request to the HIL:");
  RCLCPP_DEBUG(this->get_logger(), "Steering: %5.2f | Force: %5.2f | Throttle: %5.2f | Gear: %u | Brake: %5.2f", steering_cmd_, force_cmd_, accelerator_pedal_cmd_, gear_cmd_, brake_cmd_);

  uint8_t raptor_data_frame[4] = {gear_cmd_,purple_flag_,ct_state_,track_cond_ack_};
  send_cmd_raptor_system_socket.send_to(buffer(raptor_data_frame, sizeof(raptor_data_frame)), send_cmd_raptor_system_endpoint, 0 , err);

  RCLCPP_DEBUG(this->get_logger(), "Sent the following request to the Raptor System:");
  RCLCPP_DEBUG(this->get_logger(), "Gear: %u | purple_flag: %d | ct_state: %i | track cond ack: %i", gear_cmd_, purple_flag_, ct_state_, track_cond_ack_);

  // receive data from HIL
  boost::array<double, 14> recv_sensor_data_buffer;
  boost::array<uint8_t, 1> recv_raptor_system_state_buffer;

  // read all messages until none is there anymore
  int i = 0;
  while(recv_sensor_data_socket.receive_from(boost::asio::buffer(recv_sensor_data_buffer), recv_sensor_data_endpoint, 0, err) != 0)
  {
    i++;
  }

    if (i>0)
    {
        wheel_fl_ = recv_sensor_data_buffer[0];
        wheel_fr_ = recv_sensor_data_buffer[1];
        wheel_rl_ = recv_sensor_data_buffer[2];
        wheel_rr_ = recv_sensor_data_buffer[3];
        steering_wheel_angle_ = recv_sensor_data_buffer[7];
        front_brake_pressure_ = recv_sensor_data_buffer[8]*100;
        rear_brake_pressure_ = recv_sensor_data_buffer[9]*100;
        pedal_output_ = recv_sensor_data_buffer[10];
        gear_data_ = recv_sensor_data_buffer[13];
        engine_rpm_ = recv_sensor_data_buffer[12];
    }


  i = 0;
  while(recv_raptor_system_state_socket.receive_from(boost::asio::buffer(recv_raptor_system_state_buffer), recv_raptor_system_state_endpoint,0,err)!=0)
  {
    i++;
  }

  if (i>0)
  {
    sys_state_ = recv_raptor_system_state_buffer[0];
  }

  RCLCPP_DEBUG(this->get_logger(), "Received the following from the HIL:");
  RCLCPP_DEBUG(this->get_logger(), "Wheel FL: %5.2f | Wheel FR: %5.2f | Wheel RL: %5.2f | Wheel RR: %5.2f", wheel_fl_, wheel_fr_, wheel_rl_, wheel_rr_);
  RCLCPP_DEBUG(this->get_logger(), "Received the following from the Raptor Model: ");
  RCLCPP_DEBUG(this->get_logger(), "System State: %i", sys_state_);

  send_misc_report_do();
  send_pt_report();
  send_pt_2_report();
  send_wheel_report();
  send_steering();
  send_throttle();
  send_brake_report();
  send_base_to_car();

  RCLCPP_DEBUG(this->get_logger(), "Published everything in ROS");
}

void RaptorEmulation::send_misc_report_do()
{

  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;
  // SEND SIMULATED MISC REPORT
  can_message = dbc_av21_.GetMessageById(ID_MISC_REPORT_DO);

  can_message->GetSignal("sys_state")->SetResult(sys_state_);
  can_message->GetSignal("safety_switch_state")->SetResult(1);
  can_message->GetSignal("mode_switch_state")->SetResult(1);
  can_message->GetSignal("battery_voltage")->SetResult(13.5);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_pt_report()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_PT_REPORT_1);

  can_message->GetSignal("engine_state")->SetResult(1);
  can_message->GetSignal("engine_run_switch")->SetResult(1);
  can_message->GetSignal("throttle_position")->SetResult(pedal_output_*100);
  can_message->GetSignal("current_gear")->SetResult(gear_data_);
  can_message->GetSignal("engine_speed_rpm")->SetResult(engine_rpm_ * 60/(2*pi));
  can_message->GetSignal("vehicle_speed_kmph")->SetResult(0);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_pt_2_report()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_PT_REPORT_2);

  can_message->GetSignal("fuel_pressure_kPa")->SetResult(0.0);
  can_message->GetSignal("engine_oil_pressure_kPa")->SetResult(0.0);
  can_message->GetSignal("coolant_temperature")->SetResult(0.0);
  can_message->GetSignal("transmission_temperature")->SetResult(0.0);
  can_message->GetSignal("transmission_pressure_kPa")->SetResult(0.0);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_wheel_report()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  // convert wheelspeeds to kph. Radius is taken from powertrain documentation
  // https://docs.google.com/document/d/1gJYag8tVhtnSl3MjWvsVMBEfaR9WXndrybvJtXeLbU8/edit#

  can_message = dbc_av21_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);

  can_message->GetSignal("wheel_speed_FL")->SetResult(std::max(wheel_fl_, 0.0)*front_wheel_radius*3.6);
  can_message->GetSignal("wheel_speed_FR")->SetResult(std::max(wheel_fr_, 0.0)*front_wheel_radius*3.6);
  can_message->GetSignal("wheel_speed_RL")->SetResult(std::max(wheel_rl_, 0.0)*rear_wheel_radius*3.6);
  can_message->GetSignal("wheel_speed_RR")->SetResult(std::max(wheel_rr_, 0.0)*rear_wheel_radius*3.6);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_brake_report()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);

  brake_counter_ = brake_counter_ >= 7 ? 0 : brake_counter_ + 1;

  can_message->GetSignal("brake_pressure_fdbk_front")->SetResult(std::max(front_brake_pressure_, 0.0));
  can_message->GetSignal("brake_pressure_fdbk_rear")->SetResult(std::max(rear_brake_pressure_, 0.0));
  can_message->GetSignal("brk_pressure_fdbk_counter")->SetResult(brake_counter_);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_steering()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_STEERING_REPORT_DO);

  steering_counter_ = steering_counter_ >= 7 ? 0 : steering_counter_ + 1;

  can_message->GetSignal("steering_motor_ang_avg_fdbk")->SetResult((steering_wheel_angle_*180.0/pi)*steering_ratio);
  can_message->GetSignal("steering_motor_fdbk_counter")->SetResult(steering_counter_);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_throttle()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_ACCELERATOR_REPORT_DO);

  throttle_counter_ = throttle_counter_ >= 7 ? 0 : throttle_counter_ + 1;

  can_message->GetSignal("acc_pedal_fdbk")->SetResult(accelerator_pedal_cmd_*100);
  can_message->GetSignal("acc_pedal_fdbk_counter")->SetResult(throttle_counter_);

  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::send_base_to_car()
{
  NewEagle::DbcMessage* can_message;
  can_msgs::msg::Frame out_can_frame;

  can_message = dbc_av21_.GetMessageById(ID_BASE_TO_CAR);

  can_message->GetSignal("base_to_car_heartbeat")->SetResult(0);
  can_message->GetSignal("track_flag")->SetResult(track_flag);
  can_message->GetSignal("veh_flag")->SetResult(veh_flag);
  can_message->GetSignal("veh_rank")->SetResult(0);
  can_message->GetSignal("round_target_speed")->SetResult(0);
  
  out_can_frame = can_message->GetFrame();
  pub_to_simu_can_->publish(out_can_frame);
}

void RaptorEmulation::on_sub_from_can(const can_msgs::msg::Frame::SharedPtr msg)
{
  in_can_msg_ = msg;
  decode_can_msg();
}

void RaptorEmulation::decode_can_msg()
{
  // DECODE CURRENT CAN MSG USING THE IMPORTED DBC
  if (!in_can_msg_->is_rtr && !in_can_msg_->is_error)
  {
    switch (in_can_msg_->id)
    {
      case ID_CT_REPORT:
      {
        // SYS STATE MSG
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_CT_REPORT);
          if (in_can_msg_->dlc >= can_message->GetDlc())
          {
            can_message->SetFrame(in_can_msg_);
            ct_state_ = can_message->GetSignal("ct_state")->GetResult();
            track_cond_ack_ = can_message->GetSignal("track_cond_ack")->GetResult();
            vs_state_ack_ = can_message->GetSignal("veh_sig_ack")->GetResult();
            purple_flag_ = vs_state_ack_ == 8;
          }
        break;
      }
      case ID_BRAKE_CMD:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_BRAKE_CMD);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          brake_cmd_ = can_message->GetSignal("brake_pressure_cmd")->GetResult()/100.0;
        }
        break;
      }
      case ID_ACCELERATOR_PEDAL_CMD:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_ACCELERATOR_PEDAL_CMD);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          accelerator_pedal_cmd_ = can_message->GetSignal("acc_pedal_cmd")->GetResult()/100;
        }
        break;
      }
      case ID_STEERING_CMD:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_STEERING_CMD);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          steering_cmd_ = can_message->GetSignal("steering_motor_ang_cmd")->GetResult()*pi/(180.0*steering_ratio);
        }
        break;
      }
      case ID_GEAR_CMD:
      {
        NewEagle::DbcMessage* can_message = dbc_av21_.GetMessageById(ID_GEAR_CMD);
        if (in_can_msg_->dlc >= can_message->GetDlc()) {
          can_message->SetFrame(in_can_msg_);
          gear_cmd_ = can_message->GetSignal("desired_gear")->GetResult();
        }
        break;
      }
    }
  }
}

int main(int argc, char * argv[])
{
  // initialize ros2 stuff
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RaptorEmulation>());
  rclcpp::shutdown();
  return 0;
}
