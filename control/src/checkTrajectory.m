function [traj_ok] = checkTrajectory(Trajectory, ax_max_mps2, ay_max_mps2, ...
    a_p, drag_coefficient, roh_air, vehiclemass_kg, EmergencyLine_b)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   01.07.2019
% 
% Description:  checks whether a given trajectory complies with the
%               specified acceleration limits and (optional) if it is a 
%               valid emergency trajectory. 
%         
% Inputs:
%   Trajectory          Trajectory to be checked (see tests for format)
%   ax_max_mps2         Maximum longitudinal acceleration
%   ay_max_mps2         Maximum lateral acceleration 
%   a_p                 Exponent for tire 'ellipse'
%   cDrive_Npmps2       Driving resistance coefficient 
%   EmergencyLine_b     Set to true if it should be verified that this
%                       trajectory comes to a full stop. 
%
% Outputs: 
%   traj_ok             True if trajectory complies with all specs 

% basic checks if a valid trajectory has been send
if(Trajectory.TrajCnt == 0) 
    traj_ok = false; 
    return 
end

% check if trajectory increases strictly monotonic 
if(any(diff(Trajectory.s_loc_m) <= 0))
    traj_ok = false; 
    return 
end

% calculate acceleration requested by a driving force free vehicle to compensate for tire
% force influence 
ax_mps2 = Trajectory.ax_mps2 + (0.5.*drag_coefficient.*roh_air.*Trajectory.v_mps.^2)./vehiclemass_kg;

% calculate required lateral acceleration for every point 
ay_mps2 = Trajectory.kappa_radpm.*Trajectory.v_mps.^2; 
% check if every point complies with the acceleration limits 
acc_ok = abs(ax_mps2./ax_max_mps2).^a_p + abs(ay_mps2./ay_max_mps2).^a_p <= 1; 
if(any(~acc_ok))
    traj_ok = false; 
    return
end
% if it is an emergency line, it must end with very low velocity
if(EmergencyLine_b && Trajectory.v_mps(end) > 0.5)
    traj_ok = false; 
    return
end
% everything is ok
traj_ok = true; 
end

