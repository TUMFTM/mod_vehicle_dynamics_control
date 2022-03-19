function [LapCnt, TrajCnt, s_loc_m, s_glob_m, x_m, y_m, psi_rad, kappa_radpm, v_mps, ax_mps2, ...
    banking_rad, ax_lim_mps2, ay_lim_mps2] =...
  cutLocalTrajectoryFromRaceline(sendMessages, ActualPos, ValidPointCnt, ...
  s_m_rl, x_m_rl, y_m_rl, psi_rad_rl, kappa_radpm_rl, v_mps_rl, banking_rad_rl, foresight_s, ...
  P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, P_VDC_AccLimits_ay_mps2, P_VDC_VirtualScaleFactor)
% Authors:     Alexander Wischnewski
%
% Description: 
%   gets local trajectory pieces from a global raceline depending on the actual vehicle
%   position. Can be used to simulate a trajectory planner with a fixed raceline. 
% 
% Inputs: 
%   sendMessages:     actually get the local trajectories, otherwise returns zero trajectories
%   ActualPos:        actual vehicle pose (x_m, y_m, psi_rad) 
%   ValidPointCnt:    Number of valid points in the raceline
%   s_m_rl:           Raceline - Path coordinate vector
%   x_m_rl:           Raceline - East coordinate vector
%   y_m_rl:           Raceline - North coordinate vector
%   psi_rad_rl:       Raceline - Orientation vector
%   kappa_radpm_rl:   Raceline - Curvature vector
%   v_mps_rl:         Raceline - Velocity vector
%   foresight_s:      Prediction time used to calculate prediction distance
%   P_VDC_AccLimits_v_mps:  Acceleration limit map - velocity interpolation points
%   P_VDC_AccLimits_ax_mps2: Acceleration limit map - long. acc. limits
%   P_VDC_AccLimits_ay_mps2: Acceleration limit map - lat. acc. limits
%   P_VDC_VirtualScaleFactor: Scale factor for lower velocity racelines
% 
% Outputs: 
%   LapCnt:           current lap number
%   TrajCnt:          current local trajectory number
%   s_loc_m           Local trajectory - Local path coordinate
%   s_glob_m:         Local trajectory - Global path coordinate
%   x_m:              Local trajectory - East coordinate vector
%   y_m:              Local trajectory - North coordinate vector
%   psi_rad:          Local trajectory - Orientation vector
%   kappa_radpm:      Local trajectory - Curvature vector
%   v_mps:            Local trajectory - Velocity vector
%   ax_mps2:          Local trajectory - Long. acceleration vector
%   banking_rad:      Local trajectory - banking vector
%   ax_lim_mps2:      Local trajectory - long. acc. limit
%   ay_lim_mps2:      Local trajectory - lat. acc. limit
% 
% Parameters: 
idxDiff_Reset = 10;           % if the minimum distance index is decreased by this number of elements,
                              % the algorithm assumes that a new lap has started 
nTrajPoints = 50;             % number of points per trajectory 

persistent TrajCnt_intern ...   % local trajectory counter
  idxmindist_prev ...           % 
  LapCnt_intern;                % lap counter 
 
if(isempty(TrajCnt_intern))     % initialize persistent variables 
  TrajCnt_intern = 0; 
  idxmindist_prev = 1; 
  LapCnt_intern = 1; 
end

% check if the planner shall already send messages 
if(sendMessages)
  %% find nearest point and create vector with 49 points before and 150 points in advance 
  % calculate difference vectors between actual position and raceline points
  diff_x_m = ActualPos(1) - x_m_rl';
  diff_y_m = ActualPos(2) - y_m_rl';
  % calculate squared distance 
  dist_sq_trialpoints = diff_x_m.^2 + diff_y_m.^2; 
  % find minimum distance trial element. It is not necessary to take the 
  % square root as the actual distance is not important and the square root is a 
  % strictly monotonic function and therefore preserves minima
  [~, idx_mindist] = min(dist_sq_trialpoints(1:ValidPointCnt));
  
  %% handle trajectory and lap counter  
  % increase trajectory counter
  TrajCnt_intern = TrajCnt_intern + 1; 
  % if the minimum distance element has moved back by at least idxDiff_Reset elements, 
  % assume that a new lap has started 
  if((idx_mindist - idxmindist_prev) < -idxDiff_Reset)
    LapCnt_intern = LapCnt_intern + 1; 
  end
  % overwrite previous minimum distance index 
  idxmindist_prev = idx_mindist; 
  
  %% generate index vector for points around the minimum distance point
  % move index back by one point to ensure that start point is behind the vehicle
  idx_start_traj = idx_mindist - 1; 
  if(idx_start_traj == 0) 
      % reset to last point if first point was matched
    idx_start_traj = ValidPointCnt; 
  end
  % generate vector with path coordinates 
  s_glob_m = s_m_rl(idx_start_traj) + linspace(0, foresight_s*v_mps_rl(idx_start_traj), nTrajPoints)';  
  % project path coordinates on valid raceline coordinates 
  s_glob_m = mod(s_glob_m, s_m_rl(ValidPointCnt)); 
  
  %% extract the necessary points from the racing line 
  TrajCnt = TrajCnt_intern;
  LapCnt = LapCnt_intern; 
  x_m = interp1(s_m_rl, x_m_rl, s_glob_m, 'linear', 'extrap');
  y_m = interp1(s_m_rl, y_m_rl, s_glob_m, 'linear', 'extrap');
  psi_rad = zeros(nTrajPoints, 1); 
  for i = 1:1:nTrajPoints
    % interpolate heading seperately since function does not support non-increasing s-coordinates
    psi_rad(i) = interp1_angle(s_m_rl, psi_rad_rl, s_glob_m(i), 'linear', 'extrap');  
  end
  kappa_radpm = interp1(s_m_rl, kappa_radpm_rl, s_glob_m, 'linear', 'extrap');
  v_mps = P_VDC_VirtualScaleFactor*interp1(s_m_rl, v_mps_rl, s_glob_m, 'linear', 'extrap');
  s_loc_m = zeros(nTrajPoints, 1); 
  for i = 2:1:nTrajPoints
    % generate the corresponding arc length vector 
    s_loc_m(i) = sqrt((x_m(i) - x_m(i-1)).^2 + (y_m(i) - y_m(i-1)).^2) + s_loc_m(i-1);
  end
  % calculate the accelerations
  ax_mps2 = calcPathAx(s_loc_m, v_mps);
  banking_rad = interp1(s_m_rl, banking_rad_rl, s_glob_m, 'linear', 'extrap');
  % estimate acceleration limits from interpolation. Values for interpolation
  % taken taken from here: 
  % https://gitlab.lrz.de/iac/mod_local_planner/-/blob/develop/inputs/veh_dyn_info/localgg_constloc_varvel.csv
  ax_lim_mps2 = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ax_mps2, v_mps); 
  ay_lim_mps2 = interp1(P_VDC_AccLimits_v_mps, P_VDC_AccLimits_ay_mps2, v_mps); 
else
  % output a zero trajectory as long as the component is not considered active
  LapCnt = 0; 
  TrajCnt = 0;
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
