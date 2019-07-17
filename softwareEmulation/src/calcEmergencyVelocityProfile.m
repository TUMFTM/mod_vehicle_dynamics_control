function EmergencyTrajectory = calcEmergencyVelocityProfile(TargetTrajectory)

% copy target trajectory
EmergencyTrajectory = TargetTrajectory; 
% set negative acceleration to be constant 
ax_mps2 = -8; 
v0 = double(EmergencyTrajectory.v_mps(1)); 
% only if the trajectory is a driving trajectory 
if(v0 > 1) 
    EmergencyTrajectory.ax_mps2 = single(ones(50, 1)*ax_mps2); 
    % calculate new velocity profile
    for i = 2:1:50
        % distance to next point 
        dS = double(EmergencyTrajectory.s_m(i) - EmergencyTrajectory.s_m(i-1)); 
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
        v0 = v_next;     
    end
end
