function [PathPos, MatchIdx] = ...
  localTrajectoryMatching(VehicleDynamicState, TargetTrajectory)

%% Documentation 
%
% Author: Alexander Wischnewski     Start Date: 14.01.2018
%                                   Last update: 23.03.2020
%
% Description:  
%   provides a general matching function to convert cartesian coordinates to
%   path coordinates. It takes the position in cartesian coordinates
%   and converts it to a position in path coordinates. 
%
% Inputs:
%   VehicleDynamicState               Structure with all dynamic state information
% 
% Outputs: 
%   PathPos                           Current estimate for position in frenet coordinates 
%   MatchIdx                          Index of found trajectory point (for diagnosis)

%% Initialize structures
PathPos.s_m = 0; 
PathPos.d_m = 0; 
PathPos.psi_rad = 0; 

%% Estimate position
% calculate difference vectors 
diff_x_m = VehicleDynamicState.Pos.x_m - TargetTrajectory.x_m; 
diff_y_m = VehicleDynamicState.Pos.y_m - TargetTrajectory.y_m;
dist_squared_trialpoints = diff_x_m.^2 + diff_y_m.^2; 
% find minimum distance element 
[~, MatchIdx] = min(dist_squared_trialpoints); 
% get the longitudinal match from the actual point plus the difference of
% the rotated vector in x direction. 
PathPos.s_m = TargetTrajectory.s_loc_m(MatchIdx) + ...
  (cos(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_x_m(MatchIdx) - sin(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_y_m(MatchIdx)); 
% limit s coordinate to path 
PathPos.s_m = max(min(PathPos.s_m, TargetTrajectory.s_loc_m(end)), TargetTrajectory.s_loc_m(1)); 
% calculate lateral control error by rotating the path to the north axis
% and evaluating the negative y coordinate of the difference
PathPos.d_m = (sin(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_x_m(MatchIdx) ...
  + cos(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_y_m(MatchIdx)); 
% get path heading by interpolation
PathPos.psi_rad = normalizeAngle(VehicleDynamicState.Pos.psi_rad - ...
  interp1_angle(TargetTrajectory.s_loc_m, TargetTrajectory.psi_rad, PathPos.s_m, 'linear', 'extrap')); 

end

