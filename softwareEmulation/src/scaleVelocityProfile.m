function [v_scaled_mps, ax_scaled_mps2] = scaleVelocityProfile(s_m, v_orig_mps, scale_factor, v_max_mps)
%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   23.11.2018
% 
% Description:  takes a velocity profile and scales it according to the given parameters. 
%               Finally, the resulting acceleration profile is calculated. 
%               It is assumed, that the velocity profile is not closed. 
% 
% Inputs: 
%   s_m             Distance vector along path
%   v_orig_mps      Original velocity profile vector
%   scale_Factor    Scales the velocity profile 
%   v_max_mps       Limits the maximum speed of the velocity profile 
% 
% Outputs: 
%   v_scaled_mps    Resulting velocity profile after modification 
%   ax_scaled_mps2  Resulting acceleration profile after modification 
%_________________________________________________________________________
%% Function

ValidPointCnt = length(v_orig_mps);
% initialize outputs
v_scaled_mps = zeros(ValidPointCnt, 1); 
ax_scaled_mps2 = zeros(ValidPointCnt, 1); 

% modify velocity profile according to parameters
v_scaled_mps = min(scale_factor.*v_orig_mps, v_max_mps);
% obtain length of each point
delta_s = diff(s_m);
% assume points are equally spaced for last point 
delta_s = [delta_s; 0];
delta_s(ValidPointCnt) = delta_s(1); 
% calculate acceleration profile (last point is set to zero as it is not 
% used anyway) 
for i = 1:1:(ValidPointCnt-1)
    % check if driven distance is zero and set ax to zero in that case
    if(delta_s(i) == 0)
        ax_scaled_mps2(i) = 0;
    else
        ax_scaled_mps2(i) = (v_scaled_mps(i+1).^2 - v_scaled_mps(i).^2)./...
                        (2*delta_s(i)); 
    end
end

