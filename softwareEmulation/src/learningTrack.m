function [LapCnt, TrajCnt, s_loc_m, s_glob_m, x_m, y_m, psi_rad, kappa_radpm, v_mps, ax_mps2, ...
    banking_rad, ax_lim_mps2, ay_lim_mps2] =...
  learningTrack(sendMessages, ActualPos, InitialPos, ...
  P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, P_VDC_AccLimits_ay_mps2)
% Authors:     Alexander Wischnewski
%
% Description: 
%   creates a local trajectory which covers all relevant lateral accelerations and velocities

nTrajPoints = 50; 
nTrajHighResPoints = 300; 

% create speed and acceleration array to be tested
TargetArray_v_mps = 10:5:80; 
TargetArray_ay_mps2 = 0:2:30; 
% maximum curvature to prevent crazy accelerations at low speeds
kappa_lim_radpm = 0.12; 
% number of cycles for one operating point
nTestCycles = 150; 
% path length specified as foresight in seconds 
foresight_s = 2.5; 

persistent TrajOld;  
persistent currentIdx_v currentIdx_ay OperatingPoint_cnt done backToStraight ayOld vOld

if(isempty(TrajOld))
    TrajOld.LapCnt = uint32(0); 
    TrajOld.TrajCnt = uint32(0);
    TrajOld.s_loc_m = zeros(nTrajPoints, 1);
    TrajOld.s_glob_m = zeros(nTrajPoints, 1);
    TrajOld.x_m = zeros(nTrajPoints, 1);
    TrajOld.y_m = zeros(nTrajPoints, 1);
    TrajOld.psi_rad = zeros(nTrajPoints, 1);
    TrajOld.kappa_radpm = zeros(nTrajPoints, 1);
    TrajOld.v_mps = zeros(nTrajPoints, 1);
    TrajOld.ax_mps2 = zeros(nTrajPoints, 1);
    TrajOld.banking_rad = zeros(nTrajPoints, 1);
    TrajOld.ax_lim_mps2 = zeros(nTrajPoints, 1);
    TrajOld.ay_lim_mps2 = zeros(nTrajPoints, 1);
    TrajOld.tube_r_m = zeros(nTrajPoints, 1); 
    TrajOld.tube_l_m = zeros(nTrajPoints, 1); 
    currentIdx_v = 1; 
    currentIdx_ay = 1;
    OperatingPoint_cnt = 0; 
    done = 0; 
    backToStraight = 0;
    ayOld = 0; 
    vOld = 0; 
end

if(sendMessages && done == 0)
    % if this is the very first trajectory, start at the initial pose
    if(TrajOld.TrajCnt == 0)
        StartPose = InitialPos; 
    else
        % find current trajectory point
        VehicleDynamicState.Pos.x_m = ActualPos(1);
        VehicleDynamicState.Pos.y_m = ActualPos(2);
        VehicleDynamicState.Pos.psi_rad = ActualPos(3);
        [PathPos, ~] = localTrajectoryMatching(VehicleDynamicState, TrajOld);
        % read out trajectory point
        [ActualTrajectoryPoint] = trajectoryInterpolation(TrajOld, PathPos.s_m); 
        StartPose = [ActualTrajectoryPoint.x_m, ActualTrajectoryPoint.y_m, ActualTrajectoryPoint.psi_rad];
    end
    % create a path with constant radius and speed in front of the vehicle
    if backToStraight
        % decrease radius smoothly over the time window to prevent spinning
        w_old = (nTestCycles - OperatingPoint_cnt)/nTestCycles;
        TargetAy_mps2 = ayOld*w_old; 
        % increase speed smoothly over the time window to prevent spinning
        Target_v_mps_loc = vOld*w_old + TargetArray_v_mps(currentIdx_v)*(1-w_old); 
        TargetKappa_radpm = TargetAy_mps2/Target_v_mps_loc^2;
        TargetSpeed_v_mps = Target_v_mps_loc; 
    else
        TargetKappa_radpm = TargetArray_ay_mps2(currentIdx_ay)/TargetArray_v_mps(currentIdx_v)^2; 
        TargetSpeed_v_mps = TargetArray_v_mps(currentIdx_v); 
    end
    % high detail extrapolation of the path 
    s_loc_high_res = linspace(0, foresight_s*TargetSpeed_v_mps, nTrajHighResPoints)';
    psi_rad_high_res = normalizeAngle(cumsum([StartPose(3); TargetKappa_radpm*diff(s_loc_high_res)])); 
    x_m_high_res = cumsum([StartPose(1); -sin(psi_rad_high_res(1:end-1)).*diff(s_loc_high_res)]); 
    y_m_high_res = cumsum([StartPose(2); cos(psi_rad_high_res(1:end-1)).*diff(s_loc_high_res)]); 
    % map everything to trajectory structure
    TrajOld.s_loc_m = linspace(0, foresight_s*TargetSpeed_v_mps, nTrajPoints)'; 
    TrajOld.x_m = x_m_high_res(1:(nTrajHighResPoints/nTrajPoints):end-1); 
    TrajOld.y_m = y_m_high_res(1:(nTrajHighResPoints/nTrajPoints):end-1); 
    TrajOld.psi_rad = psi_rad_high_res(1:(nTrajHighResPoints/nTrajPoints):end-1); 
    TrajOld.kappa_radpm = TargetKappa_radpm*ones(nTrajPoints, 1); 
    TrajOld.v_mps = TargetSpeed_v_mps*ones(nTrajPoints, 1); 
    % update acceleration limits 
    TrajOld.ax_lim_mps2 = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, TargetSpeed_v_mps).*ones(nTrajPoints, 1); 
    TrajOld.ay_lim_mps2 = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ay_mps2, TargetSpeed_v_mps).*ones(nTrajPoints, 1); 
    % increase trajectory counter
    TrajOld.TrajCnt = TrajOld.TrajCnt + uint32(1); 
    % map this to the outputs
    LapCnt = uint32(1); 
    TrajCnt = TrajOld.TrajCnt;
    s_loc_m = TrajOld.s_loc_m;
    s_glob_m = TrajOld.s_glob_m;
    x_m = TrajOld.x_m;
    y_m = TrajOld.y_m;
    psi_rad = TrajOld.psi_rad; 
    kappa_radpm = TrajOld.kappa_radpm;
    v_mps = TrajOld.v_mps;
    ax_mps2 = TrajOld.ax_mps2;
    banking_rad = TrajOld.banking_rad;
    ax_lim_mps2 = TrajOld.ax_lim_mps2;
    ay_lim_mps2 = TrajOld.ay_lim_mps2;
    % update operating point
    OperatingPoint_cnt = OperatingPoint_cnt + 1; 
    % every six seconds a new operating point is targeted
    if(OperatingPoint_cnt > nTestCycles)
        % reset counter
        OperatingPoint_cnt = 0; 
        % check whether we are in normal operation
        if backToStraight == 0
            % go to next lateral acceleration 
            currentIdx_ay = currentIdx_ay + 1;
            % if last one was reached go back to straight line driving and increase speed
            % do the same if curvature limit or acceleration limit is exceeded
            if(currentIdx_ay > length(TargetArray_ay_mps2) ...
                    || TargetArray_ay_mps2(currentIdx_ay)/TargetArray_v_mps(currentIdx_v)^2 > kappa_lim_radpm ...
                    || TargetArray_ay_mps2(currentIdx_ay) > interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ay_mps2, TargetArray_v_mps(currentIdx_v)))
                % store last ay and last speed
                ayOld = TargetArray_ay_mps2(currentIdx_ay - 1); 
                vOld = TargetArray_v_mps(currentIdx_v); 
                % reset ay to zero 
                currentIdx_ay = 1; 
                % increase speed
                currentIdx_v = currentIdx_v + 1; 
                % go to back to straight mode (smoothly reduce ay to zero)
                backToStraight = 1; 
                % check if all operating points have been tested
                if(currentIdx_v > length(TargetArray_v_mps))
                    done = 1;
                end
            end
        else
            % reset to normal operation after one cycle with back to straight
           backToStraight = 0; 
        end
    end
else
  % output a zero trajectory as long as the component is not considered active
  LapCnt = uint32(0); 
  TrajCnt = uint32(0);
  s_loc_m = zeros(nTrajPoints, 1);
  s_glob_m = zeros(nTrajPoints, 1);
  x_m = zeros(nTrajPoints, 1);
  y_m = zeros(nTrajPoints, 1);
  psi_rad = zeros(nTrajPoints, 1); 
  kappa_radpm = zeros(nTrajPoints, 1);
  v_mps = zeros(nTrajPoints, 1);
  ax_mps2 = zeros(nTrajPoints, 1);
  banking_rad = zeros(nTrajPoints, 1);
  ax_lim_mps2 = zeros(nTrajPoints, 1);
  ay_lim_mps2 = zeros(nTrajPoints, 1);
end