function [PathPos, actualPathPoint, tDelta_s, PathMatchingStatus, ValidTargetPath, isTargetPathValid, KappaPrediction_radpm] = ...
  local_path_matching(VehicleDynamicState, NewTargetPath, PathEstimationActive, OldValidTargetPath,...
  OldisTargetPathValid, OldPathPos, ExternalReloadTrigger, tS, P_VDC_PathMatchInterpPoints,...
  P_VDC_PowertrainForesight_s, P_VDC_CurvatureForesight_s, P_VDC_LocalFFCalculationActive_b, P_VDC_PathMatchForesight_m)

%% Documentation 
%
% Author: Alexander Wischnewski     Start Date: 14.01.2018
%
% Description:  
%   provides a general matching function to convert cartesian coordinates to
%   path coordinates. It takes the position in cartesian coordinates
%   GlobalPos and converts it to a position PathPos in path coordinates. As
%   this transformation might be globally non-bijective it is necessary to store
%   the old position. If the path is changed, the algorithm detects this 
%   and estimates the position on the new path. Therefore the algorithm can rely on the 
%   local bijectivity properties. It relies on a basic brute force algorithm iterating along the path
%   definition and measuring the eucledian distance to the actual position. 
%
% Inputs:
%   VehicleDynamicState               Structure with all dynamic state information
%   NewTargetPath                     Target path to update the stored path 
%   PathEstimationActive              Signalizes the algorithm that it shall start matching
%   OldValidTargetPath                Stored target path 
%   OldisTargetPathValid              Indicator if stored target path is valid
%   OldPathPos                        Position on the stored target path in frenet coordinates 
%   ExternalReloadTrigger             Force reloading path 
%   tS                                Sample time 
%   P_VDC_PathMatchInterpPoints       Number of points used for interpolation during matching
%   P_VDC_PowertrainForesight_s       Foresight time for longitudinal feedforward signal
%   P_VDC_curvatureForesight_s        Foresight time for lateral feedforward signal 
%   P_VDC_LocalFFCalculationActive_b  Drop external feedforward signals and calculate new
%                                       feedforward values 
%   P_VDC_PathMatchForesight_m        Maximum foresight horizon for path matching search 
% 
% Outputs: 
%   PathPos                           Current estimate for position in frenet coordinates 
%   actualPathPoint                   Nearest path point to current position 
%   tDelta_s                          Path delay from planner to vehicle control
%   PathMatchingStatus                Health status of matching algorithm 
%   ValidTargetPath                   Current target path 
%   isTargetPathValid                 Valid status of current target path
%   KappaPrediction_radpm             Estimate for the kappa time profile of the next second


%% Setup algorithm parameters 
% define bounds for nPoints these MUST match the parameter bounds in the data dictionary
nPoints = P_VDC_PathMatchInterpPoints; 
assert(nPoints <= 1000); 
assert(nPoints >= 100); 
KappaPrediction_radpm = zeros(251, 1); 

%% initialize storage parameters 
% copy old target path in case it is not changed
ValidTargetPath = OldValidTargetPath; 
isTargetPathValid = OldisTargetPathValid;
OldPosEst_s_m = OldPathPos.s_m; 

%% Check if target trajectory should be updated 
% Switch conditions which trigger trajectory update 
%     -> Lap counter changes
%     -> Trajectory counter changes
%     -> external source requires path udpate (for example emergency) 
% Necessary conditions
%     -> Trajectory counter is not zero (there actually exist trajectories)
%     -> Trajectory path increases strictly monotonic
%     -> Path estimation is activated 
if(((ValidTargetPath.LapCnt ~= NewTargetPath.LapCnt) ||...
  (ValidTargetPath.TrajCnt ~= NewTargetPath.TrajCnt) ||...
  ExternalReloadTrigger) &&...
  NewTargetPath.TrajCnt ~= 0 &&...
  ~any(diff(NewTargetPath.s_m) <= 0) &&...
  PathEstimationActive)     
  % copy new target path to current target path 
  ValidTargetPath.s_m = single(NewTargetPath.s_m); 
  ValidTargetPath.x_m = single(NewTargetPath.x_m); 
  ValidTargetPath.y_m = single(NewTargetPath.y_m); 
  ValidTargetPath.v_mps = single(NewTargetPath.v_mps); 
  ValidTargetPath.psi_rad = single(NewTargetPath.psi_rad); 
  ValidTargetPath.Delta_rad = single(NewTargetPath.Delta_rad); 
  ValidTargetPath.DriveForce_N = single(NewTargetPath.DriveForce_N); 
  ValidTargetPath.TrajCnt = NewTargetPath.TrajCnt; 
  ValidTargetPath.LapCnt = NewTargetPath.LapCnt; 
  if(P_VDC_LocalFFCalculationActive_b) 
    % calculate curvature based on position points in the trajectory 
    ValidTargetPath.psi_rad = single(calcPathHeading(double(ValidTargetPath.x_m), double(ValidTargetPath.y_m))); 
    ValidTargetPath.kappa_radpm = single(calcPathCurvature(double(ValidTargetPath.s_m), double(ValidTargetPath.psi_rad))); 
    % calculate longitudinal acceleration based on velocity profile
    ValidTargetPath.ax_mps2 = single(calcPathAx(double(ValidTargetPath.s_m), double(ValidTargetPath.v_mps))); 
  else
    % take curvature from trajectory planner
    ValidTargetPath.kappa_radpm = single(NewTargetPath.kappa_radpm);
    % take acceleration from trajectory planner
    ValidTargetPath.ax_mps2 = single(NewTargetPath.ax_mps2);
  end
  isTargetPathValid = true; 
  % reset position estimate 
  ValidTargetPath.delay_s = NewTargetPath.delay_s;
  % use current velocity and delay estimation from trajectory planner
  % to initialize the position estimate
  OldPosEst_s_m = ValidTargetPath.delay_s*VehicleDynamicState.v_mps; 
end


%% Path matching algorithm 
if(~PathEstimationActive || ~isTargetPathValid)
  % return all zeros in case no valid path is specified or path 
  % estimation is not active
  PathPos.s_m = 0; 
  PathPos.d_m = 0; 
  PathPos.psi_rad = 0; 
  actualPathPoint.LapCnt = ValidTargetPath.LapCnt; 
  actualPathPoint.TrajCnt = ValidTargetPath.TrajCnt; 
  actualPathPoint.s_m = 0; 
  actualPathPoint.x_m = 0; 
  actualPathPoint.y_m = 0; 
  actualPathPoint.psi_rad = 0; 
  actualPathPoint.kappa_radpm = 0; 
  actualPathPoint.kappaFF_radpm = 0; 
  actualPathPoint.v_mps = 0; 
  actualPathPoint.ax_mps2 = 0; 
  actualPathPoint.axFF_mps2 = 0; 
  actualPathPoint.Delta_rad = 0; 
  actualPathPoint.DriveForce_N = 0; 
  tDelta_s = ValidTargetPath.delay_s; 
  PathMatchingStatus = TUMPathMatchingState.PM_NOTVALID; 
else
  %% Estimate position 
  % make a prediction based on the actual velocity and the old 
  % path position
  s_m_est = OldPosEst_s_m + VehicleDynamicState.v_mps*tS; 
  % check if prediction is still on the path 
  if(s_m_est > ValidTargetPath.s_m(end))
    EndOfPathReached = true; 
  else
    EndOfPathReached = false; 
  end
  % generates a vector with trial points for the next position calculation step
  s_m_est_upper = s_m_est + P_VDC_PathMatchForesight_m; 
  s_m_est_lower = s_m_est - P_VDC_PathMatchForesight_m; 
  s_m_trialpoints = linspace(max(ValidTargetPath.s_m(1), s_m_est_lower), min(ValidTargetPath.s_m(end), s_m_est_upper), nPoints); 
  % calculate position interpolants 
  x_m_trialpoints = interp1(ValidTargetPath.s_m, ValidTargetPath.x_m, s_m_trialpoints, 'linear', 'extrap'); 
  y_m_trialpoints = interp1(ValidTargetPath.s_m, ValidTargetPath.y_m, s_m_trialpoints, 'linear', 'extrap'); 
  psi_rad_trialpoints = interp1_angle(ValidTargetPath.s_m, ValidTargetPath.psi_rad, s_m_trialpoints, 'linear', 'extrap'); 
  % update path position 
  [s_m_PathPos, x_m_path, y_m_path, psi_rad_path, d_m_PathPos, psi_rad_PathPos]...
      = findPathPos(s_m_trialpoints, x_m_trialpoints, y_m_trialpoints, psi_rad_trialpoints, VehicleDynamicState.Pos.x_m, VehicleDynamicState.Pos.y_m, VehicleDynamicState.Pos.psi_rad);  
  PathPos.s_m = double(s_m_PathPos); 
  PathPos.d_m = double(d_m_PathPos); 
  PathPos.psi_rad = double(psi_rad_PathPos); 
  % get all information corresponding to the nearest path point 
  actualPathPoint.LapCnt = ValidTargetPath.LapCnt; 
  actualPathPoint.TrajCnt = ValidTargetPath.TrajCnt; 
  actualPathPoint.s_m = PathPos.s_m; 
  actualPathPoint.x_m = double(x_m_path); 
  actualPathPoint.y_m = double(y_m_path); 
  actualPathPoint.psi_rad = double(psi_rad_path); 
  actualPathPoint.kappa_radpm = double(interp1(ValidTargetPath.s_m, ValidTargetPath.kappa_radpm, PathPos.s_m, 'linear', 'extrap')); 
  actualPathPoint.kappaFF_radpm = double(interp1(ValidTargetPath.s_m, ValidTargetPath.kappa_radpm, PathPos.s_m + VehicleDynamicState.v_mps*P_VDC_CurvatureForesight_s, 'linear', 'extrap')); 
  actualPathPoint.v_mps = double(interp1(ValidTargetPath.s_m, ValidTargetPath.v_mps, PathPos.s_m, 'linear', 'extrap')); 
  % find right acceleration point (no interpolation since acc x is valid from P1 to P2) 
  actualPathPoint.ax_mps2 = double(interp1(ValidTargetPath.s_m, ValidTargetPath.ax_mps2, PathPos.s_m , 'linear', 'extrap')); 
  actualPathPoint.axFF_mps2 = double(interp1(ValidTargetPath.s_m, ValidTargetPath.ax_mps2, PathPos.s_m + VehicleDynamicState.v_mps*P_VDC_PowertrainForesight_s, 'linear', 'extrap')); 
  actualPathPoint.Delta_rad = double(interp1(ValidTargetPath.s_m, ValidTargetPath.Delta_rad, PathPos.s_m , 'linear', 'extrap')); 
  actualPathPoint.DriveForce_N = double(interp1(ValidTargetPath.s_m, ValidTargetPath.DriveForce_N, PathPos.s_m , 'linear', 'extrap')); 
  % output timediff
  tDelta_s = ValidTargetPath.delay_s; 
  % set path estimate to valid 
  if(~EndOfPathReached) 
    PathMatchingStatus = TUMPathMatchingState.PM_VALID; 
  else
    PathMatchingStatus = TUMPathMatchingState.PM_ENDREACHED;
  end
  %% calculate kappa prediction profile 
  sPrediction_m = zeros(251, 1); 
  sPrediction_m(1) = PathPos.s_m;
  KappaPrediction_radpm(1) = actualPathPoint.kappa_radpm; 
  % calculate predictions for path variable and kappa
  for i = 2:1:length(KappaPrediction_radpm)
      % calculate next path variable
     sPrediction_m(i) = double(interp1(ValidTargetPath.s_m, ValidTargetPath.v_mps, sPrediction_m(i-1), 'linear', 'extrap'))*tS + sPrediction_m(i-1); 
     % check if prediction is still a valid path point 
     if(sPrediction_m(i) <= max(ValidTargetPath.s_m))
        KappaPrediction_radpm(i) = double(interp1(ValidTargetPath.s_m, ValidTargetPath.kappa_radpm, sPrediction_m(i), 'linear', 'extrap')); 
     else
        KappaPrediction_radpm(i) = 0; 
        % exit loop and leave the rest of the prediction undone 
        break
     end
  end
end
end

