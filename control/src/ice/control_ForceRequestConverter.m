function [T_request_Nm, OverallRatio] = control_ForceRequestConverter(F_x_request_N, ...
                        tire_radius_m, i_c, i_f, i_r, i_gearset, ...
                        gear_in)
%__________________________________________________________________________
%% Documentation
%
% Author:       Dominik St�rk (dominik.staerk@tum.de)
% 
% Start Date:   21.12.2020
% 
% Description: 
% This function converts the requested overall longitudinal force 
% in a torque request for the engine. The effects of gear-ratios and 
% -efficiencies as well as differentials and acceleration-losses are taken
% into account.
% 
% Inputs: 
%   F_x_request_N   - requested overall longitudinal force [N] 
%   tire_radius_m   - tire`s radius [m]
%   i_c [2x1]       - central differential ratio and efficiency [-, -]
%   i_f [2x1]       - front differential ratio and efficiency [-, -]
%   i_r [2x1]       - rear differential ratio and efficiency [-, -]
%   i_gearset [4x7] - gear information 
%   gear_in         - currently selected gear
%
% Outputs:
%   T_request_Nm    - requested engine torque [Nm]

p_torque_rear = 1;

%% request engine torque
T_Nm = F_x_request_N * tire_radius_m;

if gear_in ~= 0
    T_request_Nm = T_Nm / (i_c(1)*i_c(2) * ...
      ((1-p_torque_rear)*i_f(1)*i_f(2) + p_torque_rear*i_r(1)*i_r(2)) * ...
      i_gearset(2, gear_in+1)*i_gearset(4, gear_in+1));
    OverallRatio = (i_c(1)*i_c(2) * ...
      ((1-p_torque_rear)*i_f(1)*i_f(2) + p_torque_rear*i_r(1)*i_r(2)) * ...
      i_gearset(2, gear_in+1)*i_gearset(4, gear_in+1))/tire_radius_m;
else
    % use first gear even if in neutral
    T_request_Nm = T_Nm / (i_c(1)*i_c(2) * ...
      ((1-p_torque_rear)*i_f(1)*i_f(2) + p_torque_rear*i_r(1)*i_r(2)) * ...
      i_gearset(2, gear_in+2)*i_gearset(4, gear_in+1));
    % no brake torque when in neutral
    OverallRatio = 0;
end
