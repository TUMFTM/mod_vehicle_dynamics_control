function generateEvasionTest(lat_dev_m, speed_mps)

% Author: Alexander Wischnewski
% 
% Description: creates a double lane change evasion scenario with the easiest trajectory planning
% possible to create challenging scenarios for the controller. 
% 
% Input: 
%   lat_dev_m: Lateral deviation in m
%   speed_mps: Speed in mps

length = 3000; 
idx_offset_start = 60; 
idx_offset_end = 100; 
n_blend = 10; 
n_foresight = 4; 
step_size = 5; 
num_traj = length/step_size + 1; 

% allocate memory 
s.s_global_m = zeros(num_traj, 1);  
s.debug_slow_tartraj_x_m = zeros(num_traj, 50); 
s.debug_slow_tartraj_y_m = zeros(num_traj, 50); 
s.debug_slow_tartraj_psi_rad = zeros(num_traj, 50) - pi/2; 
s.debug_slow_tartraj_kappa_radpm = zeros(num_traj, 50); 
s.debug_slow_tartraj_v_mps = speed_mps*ones(num_traj, 50); 
s.debug_slow_tartraj_ax_mps2 = zeros(num_traj, 50); 
s.debug_slow_tartraj_s_loc_m = zeros(num_traj, 50); 
s.debug_slow_tartraj_LapCnt = ones(num_traj, 1);  
s.debug_slow_tartraj_ax_lim_mps2 = 18.*ones(num_traj, 50); 
s.debug_slow_tartraj_ay_lim_mps2 = 18.*ones(num_traj, 50); 
s.debug_slow_tartraj_banking_rad = zeros(num_traj, 50); 

% generate arrays with target trajectories in a step size of two meters
% the path is generated along the x axis
% after some time it is offset by lat_dev_m to the left side and back again
temp_traj = linspace(0, step_size*50, 50);
for idx = 1:1:num_traj
    s.s_global_m(idx) = (idx-1)*step_size;
    s.debug_slow_tartraj_x_m(idx, :) = temp_traj + (idx-1)*step_size; 
    s.debug_slow_tartraj_s_loc_m(idx, :) = temp_traj;
    % if the current trajetory is one of the offset ones then adjust the y coordinate
    if idx >= idx_offset_start && idx < idx_offset_end
        % implement a sine wave over ten indices to move over to the new line
        steps_to_go = n_foresight+n_blend-(idx-idx_offset_start);
        current_step = n_foresight+n_blend-steps_to_go;
        if steps_to_go > 0
            if current_step < n_foresight
                s.debug_slow_tartraj_y_m(idx, 1:(n_foresight-current_step)) = 0; 
                s.debug_slow_tartraj_psi_rad(idx, 1:(n_foresight-current_step)) = -pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, 1:(n_foresight-current_step)) = 0; 
                s.debug_slow_tartraj_y_m(idx, (n_foresight-current_step+1):steps_to_go+1) = 0.5*lat_dev_m*(1-cos((0:n_blend)/n_blend*pi)); 
                s.debug_slow_tartraj_y_m(idx, (steps_to_go+2):end) = lat_dev_m;
                cos_dev_1 = 0.5*lat_dev_m*sin((0:n_blend)/n_blend*pi)/(n_blend*step_size/pi);
                cos_dev_2 = 0.5*lat_dev_m*cos((0:n_blend)/n_blend*pi)/(n_blend*step_size/pi)^2;
                s.debug_slow_tartraj_psi_rad(idx, (n_foresight-current_step+1):steps_to_go+1) = atan2(cos_dev_1, 1) - pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, (n_foresight-current_step+1):steps_to_go+1) = cos_dev_2./sqrt((1+cos_dev_1.^2).^3);
            else
                s.debug_slow_tartraj_y_m(idx, 1:steps_to_go+1) = 0.5*lat_dev_m*(1-cos(((current_step-n_foresight):n_blend)/n_blend*pi)); 
                s.debug_slow_tartraj_y_m(idx, (steps_to_go+2):end) = lat_dev_m;
                cos_dev_1 = 0.5*lat_dev_m*sin(((current_step-n_foresight):n_blend)/n_blend*pi)/(n_blend*step_size/pi);
                cos_dev_2 = 0.5*lat_dev_m*cos(((current_step-n_foresight):n_blend)/n_blend*pi)/(n_blend*step_size/pi)^2;
                s.debug_slow_tartraj_psi_rad(idx, 1:steps_to_go+1) = atan2(cos_dev_1, 1) - pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, 1:steps_to_go+1) = cos_dev_2./sqrt((1+cos_dev_1.^2).^3);
            end
        else
            s.debug_slow_tartraj_y_m(idx, :) = lat_dev_m;
        end
    elseif idx >= idx_offset_end
        % implement a sine wave over ten indices to move over to the new line
        steps_to_go = n_foresight+n_blend-(idx-idx_offset_end);
        current_step = n_foresight+n_blend-steps_to_go;
        if steps_to_go > 0
            if current_step < n_foresight
                s.debug_slow_tartraj_y_m(idx, 1:(n_foresight-current_step)) = lat_dev_m; 
                s.debug_slow_tartraj_psi_rad(idx, 1:(n_foresight-current_step)) = -pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, 1:(n_foresight-current_step)) = 0; 
                s.debug_slow_tartraj_y_m(idx, (n_foresight-current_step+1):steps_to_go+1) = lat_dev_m - 0.5*lat_dev_m*(1-cos((0:n_blend)/n_blend*pi)); 
                s.debug_slow_tartraj_y_m(idx, (steps_to_go+2):end) = 0;
                cos_dev_1 = -0.5*lat_dev_m*sin((0:n_blend)/n_blend*pi)/(n_blend*step_size/pi);
                cos_dev_2 = -0.5*lat_dev_m*cos((0:n_blend)/n_blend*pi)/(n_blend*step_size/pi)^2;
                s.debug_slow_tartraj_psi_rad(idx, (n_foresight-current_step+1):steps_to_go+1) = atan2(cos_dev_1, 1) - pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, (n_foresight-current_step+1):steps_to_go+1) = cos_dev_2./sqrt((1+cos_dev_1.^2).^3);
            else
                s.debug_slow_tartraj_y_m(idx, 1:steps_to_go+1) = lat_dev_m - 0.5*lat_dev_m*(1-cos(((current_step-n_foresight):n_blend)/n_blend*pi)); 
                cos_dev_1 = -0.5*lat_dev_m*sin(((current_step-n_foresight):n_blend)/n_blend*pi)/(n_blend*step_size/pi);
                cos_dev_2 = -0.5*lat_dev_m*cos(((current_step-n_foresight):n_blend)/n_blend*pi)/(n_blend*step_size/pi)^2;
                s.debug_slow_tartraj_psi_rad(idx, 1:steps_to_go+1) = atan2(cos_dev_1, 1) - pi/2; 
                s.debug_slow_tartraj_kappa_radpm(idx, 1:steps_to_go+1) = cos_dev_2./sqrt((1+cos_dev_1.^2).^3);
            end
        else
            s.debug_slow_tartraj_y_m(idx, :) = 0;
        end
    end
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
setValue(x0_pose, [0, 0, -pi/2]); 
saveChanges(Scenario_DD); 
close(Scenario_DD); 
