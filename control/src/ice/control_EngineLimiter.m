function cutoff = control_EngineLimiter(drive_request, n_limiter_rpm, n_engine_rpm)
%__________________________________________________________________________
%% Documentation
%
% Author:       Dominik St�rk (dominik.staerk@tum.de)
% 
% Start Date:   21.12.2020
% 
% Description: 
% This function cuts off the torque deployment if the engine rpm is above 
% limit. To avoid jumping around the rpm-limit a sigmoid function is used.
% 
% Inputs: 
%   drive_request   - drive request [-]
%   n_limiter_rpm   - upper engine rpm limit [rpm]
%   n_engine_rpm    - engine rpm [rpm]
%   
% Outputs:
%   cutoff          - engine drive cutoff [-]   


%% cut off the torque request if engine rpm limit is reached
if drive_request > 0
    D = -1;
    x = n_limiter_rpm - n_engine_rpm;
    cutoff = (1/(exp(D*x)+1)) + ((-1)/(exp(-D*x)+1));
    cutoff = max(0, min(cutoff, 1));
else
    cutoff = 1;
end
