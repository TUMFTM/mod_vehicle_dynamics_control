function EmergencyTrajectory = calcEmergencyVelocityProfile_veldep(TargetTrajectory, ...
    drag_coefficient, roh_air, vehiclemass_kg, ...
    P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, P_VDC_AccLimits_ay_mps2)

% Authors:       Alexander Wischnewski
%
% Description:  
%   calculates a brake velocity profile to a given target trajectory to generate a reasonable
%   emergency backup

% Inputs:
%   TargetTrajectory    Trajectory to be checked (see tests for format)
%   drag_coefficient    Vehicle drag coefficient
%   roh_air             air density 
%   vehiclemass_kg      vehicle mass 
%   P_VDC_AccLimits_v_mps:  Acceleration limit map - velocity interpolation points
%   P_VDC_AccLimits_ax_mps2: Acceleration limit map - long. acc. limits
%   P_VDC_AccLimits_ay_mps2: Acceleration limit map - lat. acc. limits
%
% Outputs: 
%   EmergencyTrajectory True if trajectory complies with all specs 

% copy target trajectory
EmergencyTrajectory = TargetTrajectory; 
% only if the trajectory is a driving trajectory 
if(EmergencyTrajectory.v_mps(1) > 1) 
    % calculate new velocity profile starting with the fifth element to ensure that the trajectory
    % does not begin prior to switching on it as this might cause serious issues
    for i = 6:1:50
        % distance to next point 
        dS = double(EmergencyTrajectory.s_loc_m(i) - EmergencyTrajectory.s_loc_m(i-1)); 
        % update acceleration limits 
        EmergencyTrajectory.ax_lim_mps2(i-1) = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, EmergencyTrajectory.v_mps(i-1)); 
        EmergencyTrajectory.ay_lim_mps2(i-1) = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ay_mps2, EmergencyTrajectory.v_mps(i-1)); 
        % calculate allowed longitudinal deceleration 
        ax_mps2 = -(1 - abs(double(EmergencyTrajectory.kappa_radpm(i-1))*double(EmergencyTrajectory.v_mps(i-1)).^2/EmergencyTrajectory.ay_lim_mps2(i-1)))*EmergencyTrajectory.ax_lim_mps2(i-1); 
        % add acceleration from drag 
        ax_mps2 = ax_mps2 - 0.5*drag_coefficient*roh_air*EmergencyTrajectory.v_mps(i-1)^2/vehiclemass_kg;
        % calculate time needed to travel there using pq formula 
        p = 2*EmergencyTrajectory.v_mps(i-1)/ax_mps2; 
        q = -2*dS/ax_mps2;
        % check if the solution is still valid
        if((p/2)^2 > q && EmergencyTrajectory.v_mps(i-1) > 0.2)
            v_next = (-p/2 - sqrt((p/2)^2 - q))*ax_mps2 + EmergencyTrajectory.v_mps(i-1); 
        else
            v_next = 0; 
        end
        % set calculated speed and start next step with this speed
        EmergencyTrajectory.v_mps(i) = single(v_next);
        EmergencyTrajectory.ax_mps2(i-1) = ax_mps2; 
    end
    % set final acceleration to value of before
    % use a lot less as this might lead to issues with the safety checks otherwise
    % speed should be zero here anyway
    EmergencyTrajectory.ax_mps2(50) = 0.5*EmergencyTrajectory.ax_mps2(49); 
else
    % set speeds and acceleration to zero if the initial planning speed is too slow for driving
    EmergencyTrajectory.v_mps = zeros(50, 1); 
    EmergencyTrajectory.ax_mps2 = zeros(50, 1); 
end
