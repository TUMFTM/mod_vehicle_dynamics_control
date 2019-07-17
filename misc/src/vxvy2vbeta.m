function [v_mps, beta_rad] = vxvy2vbeta(vx_mps, vy_mps, vx_calc_lim_mps)
%___________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               
% Start Date:   24.01.2018
% 
% Description:  Calculates the side slip angle and absolute
%               velocity based on the lateral and longitudinal velocities 
%               in vehicle coordinate frame. 
% 
% Inputs:       
%   vx_mps              Longitudinal velocity in mps 
%   vy_mps              Lateral velocity in mps 
%   vx_calc_lim_mps     Minimum speed for which a slip angle is calculated
% 
% Outputs: 
%   v_mps       Absolute velocity in mps
%   beta_rad    Side slip angle in rad
% 
%__________________________________________________________________________

% if speed is smaller than the minimum speed, do not calculate beta 
if(vx_mps < vx_calc_lim_mps) 
    beta_rad = 0;
    v_mps = vx_mps;
else
    beta_rad = atan2(vy_mps, vx_mps); 
    v_mps = sqrt(vx_mps^2+vy_mps^2); 
end