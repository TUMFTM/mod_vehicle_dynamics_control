function EmergencyTrajectory = calcEmergencyVelocityProfile(TargetTrajectory, P_TP_ax_max_Emergency_mps2, P_TP_ay_max_Emergency_mps2, P_TP_a_p_Emergency)

% copy target trajectory
EmergencyTrajectory = TargetTrajectory; 
% leave the first point as it is and only modify starting with the second
% point (this decreases velocity control error a lot)
v0 = double(EmergencyTrajectory.v_mps(2)); 
% only if the trajectory is a driving trajectory 
if(v0 > 1) 
    % calculate new velocity profile
    for i = 3:1:50
        % distance to next point 
        dS = double(EmergencyTrajectory.s_loc_m(i) - EmergencyTrajectory.s_loc_m(i-1)); 
        % calculate allowed longitudinal deceleration 
        ax_mps2 = -nthroot(1 - abs(double(EmergencyTrajectory.kappa_radpm(i-1))*double(EmergencyTrajectory.v_mps(i-1)).^2/P_TP_ay_max_Emergency_mps2)^P_TP_a_p_Emergency, P_TP_a_p_Emergency)*P_TP_ax_max_Emergency_mps2; 
        % calculate time needed to travel there using pq formula 
        p = 2*v0/ax_mps2; 
        q = -2*dS/ax_mps2;
        % check if the solution is still valid
        if((p/2)^2 > q && v0 > 0.2)
            v_next = (-p/2 - sqrt((p/2)^2 - q))*ax_mps2 + v0; 
        else
            v_next = 0; 
        end
        % set calculated speed and start next step with this speed
        EmergencyTrajectory.v_mps(i) = single(v_next);
        EmergencyTrajectory.ax_mps2(i-1) = ax_mps2; 
        v0 = v_next;     
    end
    % set final acceleration to value of before
    EmergencyTrajectory.ax_mps2(50) = EmergencyTrajectory.ax_mps2(49); 
end
