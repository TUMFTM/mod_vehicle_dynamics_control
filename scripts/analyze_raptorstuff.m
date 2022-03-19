close all

time_factor = 1;
steering_ratio = 19.5;

figure;

ax1 = subplot(5,1,1);
hold on, grid on
plot(log.raptor_dbw_interface_pt_report_engine_rpm_time*time_factor, log.raptor_dbw_interface_pt_report_engine_rpm);
ylabel('engine speed in rpm')

yyaxis right
plot(log.raptor_dbw_interface_wheel_speed_report_rear_left_time*time_factor, log.raptor_dbw_interface_wheel_speed_report_rear_left, '--')
plot(log.raptor_dbw_interface_wheel_speed_report_rear_right_time*time_factor, log.raptor_dbw_interface_wheel_speed_report_rear_right, '-')
ylabel('wheel speed in ???')

legend('engine speed', 'wheel speed - rear left', 'wheel speed - rear right')

ax2 = subplot(5,1,2);
plot(log.raptor_dbw_interface_pt_report_current_gear_time*time_factor, log.raptor_dbw_interface_pt_report_current_gear);
ylabel('gear')

grid on
ylim([0,6])

ax3 = subplot(5,1,3);
plot(log.raptor_dbw_interface_accelerator_pedal_cmd_pedal_cmd_time*time_factor, log.raptor_dbw_interface_accelerator_pedal_cmd_pedal_cmd);

grid on
ylabel('throttle command in %')

yyaxis right
plot(log.raptor_dbw_interface_brake_cmd_pedal_cmd_time*time_factor, log.raptor_dbw_interface_brake_cmd_pedal_cmd)
ylabel('brake pedal command in kPa')

ax4 = subplot(5,1,4);
hold on, grid on

plot(log.raptor_dbw_interface_brake_2_report_front_brake_pressure_time*time_factor, log.raptor_dbw_interface_brake_2_report_front_brake_pressure)
plot(log.raptor_dbw_interface_brake_2_report_rear_brake_pressure_time*time_factor, log.raptor_dbw_interface_brake_2_report_rear_brake_pressure)
ylabel('brake pressure in ???')
legend('front', 'rear')

ax5 = subplot(5,1,5);
hold on, grid on

plot(log.raptor_dbw_interface_steering_cmd_angle_cmd_time*time_factor, log.raptor_dbw_interface_steering_cmd_angle_cmd/steering_ratio)
plot(log.raptor_dbw_interface_steering_report_steering_wheel_angle_time*time_factor, log.raptor_dbw_interface_steering_report_steering_wheel_angle/steering_ratio)

ylabel('steering angle in deg')
legend('command', 'actual')
linkaxes([ax1,ax2,ax3,ax4,ax5],'x');
