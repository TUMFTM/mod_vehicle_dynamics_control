function [LapCnt, TrajCnt, s_loc_m, s_glob_m, x_m, y_m, psi_rad, kappa_radpm, v_mps, ax_mps2, banking_rad, ax_lim_mps2, ay_lim_mps2] =...
  cutLocalTrajectoryFromLTPL(sendMessages, ActualPos, ValidPointCnt, s_m_rl, ltpl_s_global_m, ltpl_s_local_m, ltpl_x_m, ltpl_y_m, ltpl_psi_rad, ltpl_kappa_radpm, ltpl_v_mps, ltpl_ax_mps2, ltpl_LapCnt, ltpl_banking_rad, ltpl_ax_lim_mps2, ltpl_ay_lim_mps2)
% Authors:      Alexander Wischnewski,
%               Thomas Herrmann (thomas.herrmann@tum.de)
%
% Description: 
%   gets local trajectory pieces from LTPL log data replay depending on the actual vehicle
%   position. Can be used to re-simulate a local trajectory planner. 
% 
% Inputs: 
% 
% Outputs: 
% 
% Parameters: 
nTrajPoints = 50;             % number of points per trajectory 

persistent ...
    TrajCnt_intern ...   % local trajectory counter
    LapCnt_intern ...
    idx_mindist_sum ...
    TrajCnt_intern_saved
 
if(isempty(TrajCnt_intern))     % initialize persistent variables 
    TrajCnt_intern = 0;
    LapCnt_intern = 1;  % assume to start from first lap
    idx_mindist_sum = 0;
    TrajCnt_intern_saved = 0;
end

% check if the planner shall already send messages 
if(sendMessages)
    
  %% find all indices belonging to current lap and cut logged data accordingly
  traj_idx_current_lap = find(LapCnt_intern == ltpl_LapCnt);
  % cut all ltpl data down to current lap
  ltpl_x_m = ltpl_x_m(traj_idx_current_lap, :);
  ltpl_y_m = ltpl_y_m(traj_idx_current_lap, :);
  ltpl_s_local_m = ltpl_s_local_m(traj_idx_current_lap, :);
  ltpl_s_global_m = ltpl_s_global_m(traj_idx_current_lap);
  
  ltpl_ax_mps2 = ltpl_ax_mps2(traj_idx_current_lap, :);
  ltpl_v_mps = ltpl_v_mps(traj_idx_current_lap, :);
  ltpl_psi_rad = ltpl_psi_rad(traj_idx_current_lap, :);
  ltpl_kappa_radpm = ltpl_kappa_radpm(traj_idx_current_lap, :);
  ltpl_banking_rad = ltpl_banking_rad(traj_idx_current_lap, :);
  ltpl_ax_lim_mps2 = ltpl_ax_lim_mps2(traj_idx_current_lap, :);
  ltpl_ay_lim_mps2 = ltpl_ay_lim_mps2(traj_idx_current_lap, :);
  
  %% NOTE: From now on, all input signals will be lap-specific except ltpl_LapCnt!
  % calculate difference vectors between actual position and raceline points
  diff_x_m = ActualPos(1) - ltpl_x_m(:, 1)';
  diff_y_m = ActualPos(2) - ltpl_y_m(:, 1)';
  % calculate squared distance 
  dist_sq_trialpoints = diff_x_m.^2 + diff_y_m.^2; 
  % find minimum distance trial element. It is not necessary to take the 
  % square root as the actual distance is not important and the square root is a 
  % strictly monotonic function and therefore preserves minima
  [dist_match, ~] = min(dist_sq_trialpoints);
  % find all trajectories, which are exactly the same (no LTPL update)
  dist_match_idxs = find(dist_match == dist_sq_trialpoints);
  % use the trajectory with the highest ID if multiple are available, which
  % are identical
  idx_mindist = max(dist_match_idxs);
  
  % increase lap counter for next trajectory search if logs state new lap
  idx_mindist_global = idx_mindist_sum + idx_mindist;
  if idx_mindist_global + 1 <= length(ltpl_LapCnt)
      % Have we reached the last trajectory ID for a lap?
      if ltpl_LapCnt(idx_mindist_global + 1) > ...
              ltpl_LapCnt(idx_mindist_global) && ...
              TrajCnt_intern > TrajCnt_intern_saved + 10
        LapCnt_intern = LapCnt_intern + 1;
        idx_mindist_sum = idx_mindist_sum + idx_mindist;
        TrajCnt_intern_saved = TrajCnt_intern;
      end
  end
  
  %% handle trajectory counter  
  % increase trajectory counter
  TrajCnt_intern = TrajCnt_intern + 1;
  
  %% generate index vector for points around the minimum distance point

  idx_start_traj = idx_mindist;
  
  %% extract the necessary points from the LTPL logs
  TrajCnt = TrajCnt_intern;
  LapCnt = LapCnt_intern; 

  x_m = ltpl_x_m(idx_start_traj, :)';
  y_m = ltpl_y_m(idx_start_traj, :)';
  s_loc_m = ltpl_s_local_m(idx_start_traj, :)';
  s_glob_m = s_loc_m + ltpl_s_global_m(idx_start_traj);
  % project path coordinates on valid raceline coordinates 
  s_glob_m = mod(s_glob_m, s_m_rl(ValidPointCnt));
  
  ax_mps2 = ltpl_ax_mps2(idx_start_traj, :)';
  v_mps = ltpl_v_mps(idx_start_traj, :)';
  psi_rad = ltpl_psi_rad(idx_start_traj, :)';
  kappa_radpm = ltpl_kappa_radpm(idx_start_traj, :)';
  
  banking_rad = ltpl_banking_rad(idx_start_traj, :)';
  ax_lim_mps2 = ltpl_ax_lim_mps2(idx_start_traj, :)';
  ay_lim_mps2 = ltpl_ay_lim_mps2(idx_start_traj, :)';
  
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
