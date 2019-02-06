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
% calculate heading differen between the points 
dPsi = normalizeAngle(diff(psi_rad));
% calculate path distance between the points
dS = diff(s_m);
% calculate path curvature 
kappa_radpm(1:(end-1)) = dPsi./dS; 
% assume last point to have same curvature as point before, as no better
% information is available
kappa_radpm(end) = kappa_radpm(end-1); 

end
