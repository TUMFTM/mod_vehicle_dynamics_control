function plotOvertakes(file)
% Author:       Alexander Wischnewski
% Description:  
%   function used to depict overtaking maneuvers and re-optimization
% Inputs/parameters:
%   files:      file name of logs

% plotting parameters
LineWidth = 1; 
colors = {[0, 0.3961, 0.7412], [0.8902,0.44706,0.1333], [0.6353,0.6784,0]};
t_lower = 1385; 
t_upper = 1410; 

% load all the relevant data files
data = load(file); 

% find start and end indices
idx_start = find((data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time == t_lower), 1, 'first') + 50; 
idx_end = find((data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time == t_upper), 1, 'last') - 50; 


figure; 
subplot(3, 1, 1); 
hold on; grid on; box on; 
plot(data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(idx_start:idx_end), ...
    'LineWidth', LineWidth, 'Color', colors{1}, 'DisplayName', 'Actual'); 
plot(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(idx_start:idx_end), ...
    'LineWidth', LineWidth, 'Color', colors{2}, 'DisplayName', 'MPC Target'); 
plot(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_kappa_radpm.Data(idx_start:idx_end).*...
    data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_v_mps.Data(idx_start:idx_end).^2, ...
    'LineWidth', LineWidth, 'Color', colors{3}, 'DisplayName', 'Trajectory Target'); 
xlabel('t in s'); ylabel({'Lateral Acceleration', 'in mps2'}); ylim([-30, 30]);xlim([t_lower, t_upper]); legend('Location', 'southeast');


subplot(3, 1, 2); 
hold on; grid on; box on; 
plot(data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_v_mps.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_v_mps.Data(idx_start:idx_end), ...
    'LineWidth', LineWidth, 'Color', colors{1}, 'DisplayName', 'Actual'); 
plot(data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(idx_start:idx_end), ...
    'LineWidth', LineWidth, 'Color', colors{2}, 'DisplayName', 'Target'); 
xlabel('t in s'); ylabel({'Vehicle speed', 'in mps'}); ylim([60, 75]);xlim([t_lower, t_upper]); legend('Location', 'southeast');

subplot(3, 1, 3); 
hold on; grid on; box on; 
plot(data.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Time(idx_start:idx_end), ...
    data.debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data(idx_start:idx_end), ...
    'LineWidth', LineWidth, 'Color', colors{1}); 
xlabel('t in s'); ylabel({'Lateral deviation', 'in m'}); ylim([-1.3, 1.3]);xlim([t_lower, t_upper]); 
plot([data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Time(idx_start),...
    data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Time(idx_end)], ...
    [1, 1], '--k', 'LineWidth', LineWidth);
plot([data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Time(idx_start),...
    data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Time(idx_end)], ...
    [-1, -1], '--k', 'LineWidth', LineWidth);

matlab2tikz('LateralComparison.tex', 'standalone', true);