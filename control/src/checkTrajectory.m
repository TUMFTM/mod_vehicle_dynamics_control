function [traj_ok] = checkTrajectory(Trajectory, drag_coefficient, roh_air, vehiclemass_kg, EmergencyLine_b, P_VDC_FinalSpeedEmergency_mps)
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
%   Trajectory                  Trajectory to be checked (see tests for format)
%   drag_coefficient            Vehicle drag coefficient
%   roh_air                     air density 
%   vehiclemass_kg              vehicle mass 
%   P_VDC_MinVelSlipCalc_mps    Speed above which slip calculation works
%   EmergencyLine_b             Set to true if it should be verified that this
%                               trajectory comes to a full stop. 
%
% Outputs: 
%   traj_ok             True if trajectory complies with all specs 

% Parameters (these are *not* tunable by intentation and therefore set here)
AccCheckTolerance = 1.025;
AccelerationErrorTolerance_mps2 = 2; 

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
acc_ok = abs(ax_mps2./Trajectory.ax_lim_mps2) + abs(ay_mps2./Trajectory.ay_lim_mps2) <= AccCheckTolerance; 
if(any(~acc_ok))
    traj_ok = false; 
    return
end

% if it is an emergency line, it must end with very low velocity
if(EmergencyLine_b && Trajectory.v_mps(end) > P_VDC_FinalSpeedEmergency_mps)
    traj_ok = false; 
    return
end

% verfiy if velocity profile matches velocity where velocity is larger than minimum slip speed
ax_mps2_recalc = calcPathAx(Trajectory.s_loc_m, Trajectory.v_mps); 
ax_mps2_error = ax_mps2_recalc - Trajectory.ax_mps2; 
for i = 1:1:(length(Trajectory.ax_mps2)-1)
    % only consider indices where the next index is still above zero
    if(Trajectory.v_mps(i+1) > 0)
        % perform actual check 
        if(abs(ax_mps2_error) > AccelerationErrorTolerance_mps2)
            traj_ok = false; 
            return
        end
    % if speed is zero only check that acceleration is negative
    else
        if(Trajectory.ax_mps2 > 0)
            traj_ok = false; 
            return
        end
    end
end

% everything is ok
traj_ok = true; 
end

