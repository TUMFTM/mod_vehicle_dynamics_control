function [psi_rad] = calcPathHeading(x_m, y_m)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   01.02.2018
% 
% Description:  calculates the heading profile corresponding to a 
%               a given path based on cartesian point coordinates. 
%         
% Inputs:
%   x_m             Vector with x positions in global, cartesian
%                   coordinates
%   y_m             Vector with y positions in global, cartesian
%                   coordinates
%
% Outputs: 
%   psi_rad         Vector with heading values 

% initialize variables
psi_rad = zeros(length(x_m), 1); 
for i = 2:1:(length(x_m)-1)
    % calculate position difference between points 
    dx = x_m(i+1) - x_m(i-1); 
    dy = y_m(i+1) - y_m(i-1); 
    % output angle is zero in north direction and limited to +pi/-pi
    psi_rad(i) = normalizeAngle(atan2(dy, dx) - pi/2); 
end
% first value is assumed to be equal to the point after as no better
% information is available 
psi_rad(1) = psi_rad(2); 
% last value is assumed to be equal to the point before as no better
% information is available 
psi_rad(end) = psi_rad(end-1); 

end
