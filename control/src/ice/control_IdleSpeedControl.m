function idle_control = control_IdleSpeedControl(n_idle_rpm, n_engine_rpm)
%__________________________________________________________________________
%% Documentation
%
% Author:       Dominik St�rk (dominik.staerk@tum.de)
% 
% Start Date:   21.12.2020
% 
% Description: 
% This function controls the torque request while the engine is in neutral.
% 
% Inputs: 
%   n_idle_rpm      - idle speed [rpm] 
%   w_engine_radps  - engine speed [radps]
%   
% Outputs:
%   idle_control    - P-controlled output [-]


%% output torque
n_delta_rpm = n_idle_rpm - n_engine_rpm;

% p-controller
if n_delta_rpm >= 0
   idle_control = n_delta_rpm;
else
   idle_control = 0;
end