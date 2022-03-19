function plotSteeringComparison(files, laps, labels)
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
DyLim = 1.0; 
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
hold on; grid on;
colors = {'r', 'b'};
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_actuator_debug_RequestSteeringAngle_rad.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth, 'Color', colors{i}, 'DisplayName', [labels{i} ' Target']); 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_VehicleSensorData_Delta_Wheel_rad.Data(idx_start{i}:idx_end{i}), '--',...
        'LineWidth', LineWidth, 'Color', colors{i}, 'DisplayName', [labels{i} ' Actual']); 
end
xlabel('s in m'); ylabel({'Steering', 'in rad'}); ylim([-0.02, 0.02]);
legend();
