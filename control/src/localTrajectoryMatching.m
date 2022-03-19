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
% save smallest distance
min_dist = sqrt(dist_squared_trialpoints(MatchIdx));
% find second smallest element 
dist_squared_trialpoints(MatchIdx) = max(dist_squared_trialpoints); 
[~, MatchIdx2] = min(dist_squared_trialpoints); 
% vector along trajectory (always forward facing)
if(MatchIdx2 > MatchIdx)
    e1_x = TargetTrajectory.x_m(MatchIdx2) - TargetTrajectory.x_m(MatchIdx); 
    e1_y = TargetTrajectory.y_m(MatchIdx2) - TargetTrajectory.y_m(MatchIdx); 
    e2_x = VehicleDynamicState.Pos.x_m - TargetTrajectory.x_m(MatchIdx);
    e2_y = VehicleDynamicState.Pos.y_m - TargetTrajectory.y_m(MatchIdx);
else
    e1_x = TargetTrajectory.x_m(MatchIdx) - TargetTrajectory.x_m(MatchIdx2); 
    e1_y = TargetTrajectory.y_m(MatchIdx) - TargetTrajectory.y_m(MatchIdx2); 
    e2_x = VehicleDynamicState.Pos.x_m - TargetTrajectory.x_m(MatchIdx2);
    e2_y = VehicleDynamicState.Pos.y_m - TargetTrajectory.y_m(MatchIdx2);
end
% calculate the lateral distance. This method is more robust than using the similar one from the
% longitudinal approach. 
PathPos.d_m = -(e2_x*e1_y - e1_x*e2_y)/max(0.05, sqrt(e1_x^2+e1_y^2));
% calculate longitudinal distance
PathPos.s_m = TargetTrajectory.s_loc_m(MatchIdx) + ...
  (cos(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_x_m(MatchIdx) - sin(-(TargetTrajectory.psi_rad(MatchIdx)+pi/2))*diff_y_m(MatchIdx)); 
% limit s coordinate to path 
PathPos.s_m = max(min(PathPos.s_m, TargetTrajectory.s_loc_m(end)), TargetTrajectory.s_loc_m(1)); 
% get path heading by interpolation
PathPos.psi_rad = normalizeAngle(VehicleDynamicState.Pos.psi_rad - ...
  interp1_angle(TargetTrajectory.s_loc_m, TargetTrajectory.psi_rad, PathPos.s_m, 'linear', 'extrap')); 

end

