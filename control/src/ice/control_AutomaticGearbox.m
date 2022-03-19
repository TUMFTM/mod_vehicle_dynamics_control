function [gear, shift_request] = control_AutomaticGearbox(drive_request, gear_in, ...
                            w_engine_radps, n_ub_rpm, i_gearset, shift_delay_s)
%__________________________________________________________________________
%% Documentation
%
% Author:       Dominik St�rk (dominik.staerk@tum.de)
% 
% Start Date:   21.12.2020
% 
% Description: 
% This function serves as a gear shift logic. It upshifts if the engine rpm
% is above the rpm limit and downshifts if the engine rpm is below the rpm
% limit of the current gear.
%
% Inputs: 
%   gear_in             - currently selected gear
%   drive_request       - drive request [-]
%   w_engine_radps      - engine angular velocity [radps]
%   n_ub_rpm            - engine upper rpm limit [rpm]
%   i_gearset           - available gearset 
%   shift_delay_s       - time since last gear shift [s]
%   
% Outputs:
%   gear                - output gear
%   shift_request       - shift request [bool]

%% gear shift logic
% init rpm limits
radps_lb = i_gearset(3, gear_in+1) * ((2*pi)/60);
radps_ub = n_ub_rpm * ((2*pi)/60);

persistent current_request 

if(isempty(current_request))
    current_request = int16(-1);
end

% initialize with current gear as soon as it is running 
if(w_engine_radps > 50 && current_request == -1)
    current_request = gear_in;
end

% only start shifting after it has been initialized
if current_request ~= -1
    % prevent gear jumping by waiting some time until gear shift is tried again
    if shift_delay_s >= 1.2
        % upshift
        if w_engine_radps>=radps_ub && i_gearset(1, gear_in+1) < i_gearset(1, end)
           current_request = gear_in + 1;
           shift_request = 1;
        % downshift
        elseif w_engine_radps<=radps_lb && i_gearset(1, gear_in+1) > 1
           current_request = gear_in - 1;
           shift_request = 1;
        else
           shift_request = 0;
        end
    else
        % 0.6 seconds until shift is required to be completed 
        % if not completed, reset current request and try again after full second has expired
        if (shift_delay_s >= 0.7 && current_request ~= gear_in)
            current_request = gear_in;
        end
        shift_request = 0;
    end
    gear = current_request;
else 
    gear = gear_in;
    shift_request = 0;
end
