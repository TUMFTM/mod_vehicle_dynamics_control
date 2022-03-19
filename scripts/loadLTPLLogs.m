function [s] = loadLTPLLogs(Ts, Ts_slow)

% Author: Thomas Herrmann     Date: 19-01-2021
% 
% Description: This function loads the logged LTPL data, which arrived at
% the control module and prepares these LTPL information for a log data
% replay in Simulink. Using this funcitonality, the controller can be
% developed further.
% 
% Input:    Ts - logging file
%           Ts_slow - slow logging file
%
% Output: s - struct with data from timeseries input oject

% specify signals you want to extract, 'd' must be a fixed string
sig_names = struct(...
    'd1', 'debug_slow_tartraj_x_m', ...
    'd2', 'debug_slow_tartraj_y_m', ...
    'd3', 'debug_slow_tartraj_psi_rad', ...
    'd4', 'debug_slow_tartraj_kappa_radpm', ...
    'd5', 'debug_slow_tartraj_v_mps', ...
    'd6', 'debug_slow_tartraj_ax_mps2', ...
    'd7', 'debug_slow_tartraj_s_loc_m', ...
    'd8', 'debug_slow_tartraj_LapCnt', ...
    'd9', 'debug_slow_tartraj_ax_lim_mps2', ...
    'd10', 'debug_slow_tartraj_ay_lim_mps2', ...
    'd11', 'debug_slow_tartraj_banking_rad');
    
% specify integer of how much faster the fast logging file was recorded
slow_fast_factor = 10;

%% position of vehicle (fast logging file)

% get time of first entry in slow debug file
t_zero = Ts_slow.debug_slow_ax_traj_mps2.Time(1);
% find synchronisation point in fast debug file
t_zero_fast_idx = find(t_zero==Ts.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Time);

% get global s coordinate from fast logging file which matches every entry
% in the slow logging file
s_global_slow_match_m = Ts.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data(t_zero_fast_idx:slow_fast_factor:end);

%%
% initialize empty struct
s = struct();

s.s_global_m = s_global_slow_match_m;

% remove all entries with only zeros at beginning of race
zero_counter = 1;
for k=1:length(Ts_slow.(sig_names.d1).Data)
    if sum(abs(Ts_slow.(sig_names.d1).Data(k, :))) == 0
        zero_counter = zero_counter + 1;
    end
end

s.s_global_m = s.s_global_m(zero_counter:end);
for k=1:length( fields(sig_names) )
    en = ['d', num2str(k)];
    s.(sig_names.(en)) = Ts_slow.(sig_names.(en)).Data(zero_counter:end, :);
end

%% Write to data dictionary
DDObj = ...
    Simulink.data.dictionary.open('TrajectoryPlanningEmulation.sldd');
dataSectObj = getSection(DDObj, 'Design Data');
try addEntry(dataSectObj, 'ltpl_log', s);
% if entry was there already modify it
catch e
    if isa(e, 'MSLException')
        ltplLogObj = getEntry(dataSectObj, 'ltpl_log');
        setValue(ltplLogObj, s);
    else
        print('Something went wrong.')
    end
end

% set switch to enable local log replay
switchObj = getEntry(dataSectObj, 'P_VDC_TrajEmulation_mode');
setValue(switchObj, 1);
% save & close
saveChanges(DDObj);
close(DDObj);

% set start position
Scenario_DD = Simulink.data.dictionary.open('raceline.sldd');
dataSectObj = getSection(Scenario_DD, 'Design Data');
x0_pose = getEntry(dataSectObj, 'x0_vehiclepose_stm');
setValue(x0_pose, ...
    [s.debug_slow_tartraj_x_m(1, 1), ...
     s.debug_slow_tartraj_y_m(1, 1), ...
     s.debug_slow_tartraj_psi_rad(1, 1)]); 
saveChanges(Scenario_DD); 
close(Scenario_DD); 

    
end
