function [s_m_path, x_m_path, y_m_path, psi_rad_path,...
  d_m_veh, psi_rad_veh_p] = findPathPos(s_m_vec, x_m_vec, y_m_vec, psi_rad_vec,...
  x_m_veh, y_m_veh, psi_rad_veh_g) 
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   01.02.2018
% 
% Description:  Outputs the closest match for the actual position in the 
%               given path vector. It is based on a brute force search. 
%         
% Inputs: 
%   s_m_vec:        Vector with arc length parameter in meter
%   x_m_vec:        Vector with path x position in meter
%   y_m_vec:        Vector with path y position in meter
%   psi_rad_vec:    Vector with path orientation in radians
%   x_m_veh:        Actual vehicle position in x coordinates 
%   y_m_veh:        Actual vehicle position in y coordinates
% Outputs:
%   s_m_path:       Arc length in meter of closest point 
%   x_m_path:       x position in meter of closest point
%   y_m_path:       y position in meter of closest point
%   psi_rad_path:   Orientation in radians of closest point
%   d_m_veh:        Vehicle lateral deviation in meters from closest point 
%   psi_rad_veh:    Vehicle heading deviation from closest point 
    
% calculate difference vectors 
diff_x_m = x_m_veh - x_m_vec; 
diff_y_m = y_m_veh - y_m_vec;
dist_squared_trialpoints = diff_x_m.^2 + diff_y_m.^2; 
% find minimum distance element 
[mindist_squared_m, idx_mindist] = min(dist_squared_trialpoints);
% retrieve path points 
s_m_path = s_m_vec(idx_mindist); 
x_m_path = x_m_vec(idx_mindist); 
y_m_path = y_m_vec(idx_mindist); 
psi_rad_path = psi_rad_vec(idx_mindist); 
% calculate sign of control error
diffvec_rot = [cos(-psi_rad_vec(idx_mindist)), -sin(-psi_rad_vec(idx_mindist))]*...
  [diff_x_m(idx_mindist);diff_y_m(idx_mindist)]; 
% calculate path orientation
psi_rad_veh_p = normalizeAngle(psi_rad_veh_g - psi_rad_path);
% project difference vector onto path
d_m_veh = -sign(diffvec_rot)*sqrt(mindist_squared_m)*cos(psi_rad_veh_p); 


