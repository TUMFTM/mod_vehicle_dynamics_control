function checkTrajectoryLogs(data, ID)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% % 
% Description:  used to recalculate the checks from the logs
%         
% Inputs:
%   File                Log file loaded into data struct
%   ID                  Trajectory ID to be checked

% set parameters 
drag_coefficient = 0.725; 
roh_air = 1.22; 
vehiclemass_kg  = 750; 
EmergencyLine_b = 0; 

TargetTrajectory.LapCnt = data.debug_slow.debug_slow_tartraj_LapCnt.Data(ID); 
TargetTrajectory.TrajCnt = data.debug_slow.debug_slow_tartraj_TrajCnt.Data(ID); 
TargetTrajectory.s_loc_m = data.debug_slow.debug_slow_tartraj_s_loc_m.Data(ID, :)'; 
TargetTrajectory.s_glob_m = data.debug_slow.debug_slow_tartraj_s_glob_m.Data(ID, :)'; 
TargetTrajectory.x_m = data.debug_slow.debug_slow_tartraj_x_m.Data(ID, :)'; 
TargetTrajectory.y_m = data.debug_slow.debug_slow_tartraj_y_m.Data(ID, :)'; 
TargetTrajectory.psi_rad = data.debug_slow.debug_slow_tartraj_psi_rad.Data(ID, :)'; 
TargetTrajectory.kappa_radpm = data.debug_slow.debug_slow_tartraj_kappa_radpm.Data(ID, :)'; 
TargetTrajectory.v_mps = data.debug_slow.debug_slow_tartraj_v_mps.Data(ID, :)'; 
TargetTrajectory.ax_mps2 = data.debug_slow.debug_slow_tartraj_ax_mps2.Data(ID, :)'; 
TargetTrajectory.banking_rad = data.debug_slow.debug_slow_tartraj_banking_rad.Data(ID, :)'; 
TargetTrajectory.ax_lim_mps2 = data.debug_slow.debug_slow_tartraj_ax_lim_mps2.Data(ID, :)'; 
TargetTrajectory.ay_lim_mps2 = data.debug_slow.debug_slow_tartraj_ay_lim_mps2.Data(ID, :)'; 

traj_ok = checkTrajectory(TargetTrajectory, drag_coefficient, roh_air, vehiclemass_kg, EmergencyLine_b); 
if traj_ok
    disp('The trajectory was labelled ok'); 
else
    disp('The trajectory was labelled not ok'); 
end