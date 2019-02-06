function [ax_mps2] = calcPathAx(s_m, v_mps)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   08.02.2018
% 
% Description:  calculates the acceleration profile corresponding to a 
%               velocity profile for a given path length. Constant
%               acceleration is assumed between the discretization points. 
%         
% Inputs:
%   s_m         Vector with path length values
%   v_mps       Vector with velocity at these path points 
%
% Outputs: 
%   ax_mps2     Vector with accelerations valid from the current to the
%               next point. 

% initialize variables
ax_mps2 = zeros(length(v_mps), 1); 
% calculate acceleration based on the assumption that it is constant
% between two discretization points. Last point can't be calculated as no
% further velocity information is available for the point behind. 
dS = diff(s_m); 
dV = diff(v_mps); 
ax_mps2(1:(end-1)) = (dV.^2 + 2.*dV.*v_mps(1:(end-1)))./(2.*dS); 
% copy last point which could be calculated, as no better information is
% available 
ax_mps2(end) = ax_mps2(end-1); 