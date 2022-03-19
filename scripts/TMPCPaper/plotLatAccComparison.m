function plotLatAccComparison(files, laps, labels)
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
colors = {[0, 0.3961, 0.7412], [0.8902,0.44706,0.1333], [0.6353,0.6784,0]};
sLim_lower = 1700; 
sLim_upper = 2500; 

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
subplot(2, 1, 1); 
hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth, 'Color', colors{i}, 'DisplayName', [labels{i}]); 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth, 'Color', colors{i}, 'LineStyle', '--', 'HandleVisibility', 'off'); 
end
xlabel('s in m'); ylabel({'Lateral Acceleration', 'in mps2'}); ylim([-5, 25]);xlim([sLim_lower, sLim_upper]); legend();


subplot(2, 1, 2); 
hold on; grid on; box on; 
for i = 1:1:length(files) 
    plot(data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}:idx_end{i}), ...
        data{i}.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data(idx_start{i}:idx_end{i}), ...
        'LineWidth', LineWidth, 'Color', colors{i}, 'DisplayName', [labels{i}]); 
end
xlabel('s in m'); ylabel({'Lateral deviation', 'in m'}); ylim([-1.3, 1.3]);xlim([sLim_lower, sLim_upper]); 
plot([data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}),...
    data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_end{i})], ...
    [1, 1], '--k', 'LineWidth', LineWidth);
plot([data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_start{i}),...
    data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(idx_end{i})], ...
    [-1, -1], '--k', 'LineWidth', LineWidth);

matlab2tikz('LateralComparison.tex', 'standalone', true);