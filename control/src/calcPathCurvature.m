function [kappa_radpm] = calcPathCurvature(s_m, psi_rad)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   01.02.2018
% 
% Description:  calculates the curvature profile corresponding to a 
%               a given path based on heading information. 
%         
% Inputs:
%   s_m             Vector with path length values
%   psi_rad         Vector with heading of the path at the discretization 
%                   point in global coordinates
%
% Outputs: 
%   kappa_radpm     Vector with constant curvature valid between two 
%                   discretization points

% initialize variables
kappa_radpm = zeros(length(psi_rad), 1); 
for i = 2:1:length(psi_rad)-1
    % calculate two sided difference
   dPsi = normalizeAngle(psi_rad(i+1) - psi_rad(i-1)); 
   dS = s_m(i+1) - s_m(i-1); 
   kappa_radpm(i) = dPsi/dS; 
end
% first value is assumed to be equal to the point after as no better
% information is available 
kappa_radpm(1) = kappa_radpm(2); 
% assume last point to have same curvature as point before, as no better
% information is available
kappa_radpm(end) = kappa_radpm(end-1); 

end
