function benchmarkControllers(logs)
%
% Authors:       Alexander Wischnewski
%
% Description:  
%   function used to compare the control performance with respect to the specified constraints for
%   different datasets
% 
% Inputs: 
%   logs: Cell array with strings of the files to analyze

% load data
for i = 1:1:length(logs) 
    data{i} = load(logs{i}); 
end

% analyze the start and end index of the first flying lap in the dataset 
for i = 1:1:length(data) 
    % find index of last data points in the lap 
    deltaLapCnt = diff(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
    lastPoint_idx_raw = find(deltaLapCnt);
    % take first flying lap and substract some points to mitigate small errors in global coordinate
    idxStart{i} = lastPoint_idx_raw(2) + 5;
    idxEnd{i} = lastPoint_idx_raw(3) - 50; 
end

% plot the velocity profile
figure; grid on; hold on; 
% plot the actual velocity profiles
for i = 1:1:length(data) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idxStart{i}:idxEnd{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(idxStart{i}:idxEnd{i}), ...
        'DisplayName', logs{i}); 
end
% plot target velocity profile only once 
plot(data{1}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idxStart{1}:idxEnd{1}), ...
data{1}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_v_mps.Data(idxStart{1}:idxEnd{1}), ...
    'DisplayName', 'Target Speed'); 
legend(); 
xlabel('Global path coordinate in m'); 
ylabel('Speed in mps'); 
ylim([0 70]); 

% plot the lateral deviation
figure; grid on; hold on;  
for i = 1:1:length(data) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idxStart{i}:idxEnd{i}), ...
        data{i}.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data(idxStart{i}:idxEnd{i}), ...
        'DisplayName', logs{i}); 
end
legend(); 
xlabel('Global path coordinate in m'); 
ylabel('Lateral deviation in m'); 
ylim([-3 3]); 

% plot the combined accelerations
figure; grid on; hold on;  
for i = 1:1:length(data) 
    a_combined = abs(data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data(idxStart{i}:idxEnd{i})) + ...
        abs(data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(idxStart{i}:idxEnd{i})); 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idxStart{i}:idxEnd{i}), ...
        a_combined, ...
        'DisplayName', logs{i}); 
end
legend(); 
xlabel('Global path coordinate in m'); 
ylabel('Acceleration combined in mps2'); 
ylim([-15 15]); 
