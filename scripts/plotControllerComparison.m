function plotControllerComparison(files, laps, labels)
% Author:       Alexander Wischnewski
% Description:  
%   function used to compare different controllers 
% Inputs/parameters:
%   files:      Cell array with file names of logs
%   laps:       laps to evaluate (matrix with start and end lap for each datafile)
%   labels:     Cell array with abels for the datasets used for legends 
%               (file names are used if not given) 

% plotting parameters
LineWidth = 1; 
DyLim = 1; 
axLim = 25; 
ayLim = 25; 

% load all the relevant data files
for i = 1:1:length(files) 
    data{i} = load(files{i}); 
end

% check if labels are given, if not use file names
if(nargin <= 2) 
    labels = files; 
end

% find start and end indices
for i = 1:1:length(files) 
    idx_start{i} = find((data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(i,1)), 1, 'first') + 50; 
    idx_end{i} = find((data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(i,2)), 1, 'last') - 50; 
end

figure; 
% steering angle request plot
ax1 = subplot(4, 2, 1); hold on; grid on; box on; 

for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_actuator_debug_RequestSteeringAngle_rad.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('Steering Angle in rad'); ylim([-0.1, 0.1]);
legend(labels);

% lateral acceleration
ax2 = subplot(4, 2, 3); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel({'Lateral', 'acceleration in mps2'}); ylim([-1.2*ayLim, 1.2*ayLim]); 
legend(labels); 

% lateral deviation
ax3 = subplot(4, 2, 5); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
yline(DyLim); yline(-DyLim); 
xlabel('s in m'); ylabel('Lateral deviation in m'); ylim([-1.5*DyLim, 1.5*DyLim]); 
legend(labels); 

% longitudinal force request
ax4 = subplot(4, 2, 2); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_dPsi_radps.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel({'yaw rate', 'in rad/s'}); ylim([-0.4, 0.4]); 
legend(labels); 

% longitudinal acceleration
ax5 = subplot(4, 2, 4); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_beta_rad.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel({'beta', 'in rad'}); ylim([-0.1, 0.1]); 
legend(labels); 

% velocity
ax6 = subplot(4, 2, 6); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('Velocity in mps'); ylim([0, 80]); 
legend(labels); 

% velocity
ax7 = subplot(4, 2, 7); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_actuator_debug_RequestLongForce_N.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('Long. Force request in N'); ylim([-8000, 8000]); 
legend(labels); 

% velocity
ax8 = subplot(4, 2, 8); hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('a_x in mps'); ylim([-10, 10]); 
legend(labels); 

% link axes
linkaxes([ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8], 'x'); 

% tire utilization plots
figure; 
ax21 = subplot(2, 1, 1); 
grid on; hold on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_trajectory_driver_perf_TireUtilization.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('Tire utilization actual'); ylim([0, 1.5]); 
legend(labels); 

ax22 = subplot(2, 1, 2); 
grid on; hold on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_tmpc_fast_debug_TireUtilizationTarget.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth); 
end
xlabel('s in m'); ylabel('Tire utilization target'); ylim([0, 1.5]); 
legend(labels); 

% link axes of tire utilization plots
linkaxes([ax21, ax22], 'x');


% scatter plot with acceleration limits
figure; hold on; grid on; box on; 
for i = 1:1:length(files) 
    scatter(data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data(idx_start{i}:idx_end{i}), ...
        10, 'filled'); 
end
plot([0, ayLim], [-axLim, 0], 'k'); 
plot([0, -ayLim], [-axLim, 0], 'k'); 
plot([0, ayLim], [axLim, 0], 'k'); 
plot([0, -ayLim], [axLim, 0], 'k'); 
xlabel('Lat. acceleration in mps2'); ylabel('Long. acceleration in mps2'); 
axis equal; 
legend(labels); 

% scatter plot with raceline
figure; hold on; grid on; box on; 
i=1;
scatter(data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m.Data(idx_start{i}:idx_end{i}), ...
    data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m.Data(idx_start{i}:idx_end{i}), ...
    10, data{i}.debug.debug_mvdc_actuator_debug_RequestSteeringAngle_rad.Data(idx_start{i}:idx_end{i})); 
i=2;
scatter(data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m.Data(idx_start{i}:idx_end{i}), ...
    data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m.Data(idx_start{i}:idx_end{i}), ...
    10, data{i}.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data(idx_start{i}:idx_end{i}),'^'); 

xlabel('x coordinate in m'); ylabel('y coordinate in m'); 
axis equal
legend(labels);
cb = colorbar();
ylabel(cb, 'lateral deviation in m');