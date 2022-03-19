function plotLatFFLearning(struct, idx_recover)

% Authors: 
%       Alexander Wischnewski
% Description
%   visualizes the learned FF model
% Inputs:
%   struct:      Log file struct
%   idx_recover: ID of the optimization problem which is plotted (debug_slow.tmpc_cnt)

sysDict = Simulink.data.dictionary.open('il_mvdc_mpc.sldd');
dDataSectObj2 = getSection(sysDict,'Design Data');
% load controller parameters from data dictionary
acc_control_learning = getValue(getEntry(dDataSectObj2, 'acc_control_learning'));

% reshape results 
[VX, AY] = meshgrid(acc_control_learning.spread_vx_mps, acc_control_learning.spread_ay_req_mps2); 
Z_mean = reshape(struct.debug_slow.debug_slow_LatAcc_mean_debug.Data(idx_recover, :), ...
    length(acc_control_learning.spread_ay_req_mps2), length(acc_control_learning.spread_vx_mps)); 
Z_cov = reshape(struct.debug_slow.debug_slow_LatAcc_cov_debug.Data(idx_recover, :), ...
    length(acc_control_learning.spread_ay_req_mps2), length(acc_control_learning.spread_vx_mps)); 

figure; 
subplot(1, 2, 1); 
surface(VX, AY, Z_mean); 
xlabel('Vehicle speed in mps'); 
ylabel('Lateral acceleration in mps2'); 
zlabel('Corrective steering in rad'); 
subplot(1, 2, 2); 
surface(VX, AY, Z_cov); 
xlabel('Vehicle speed in mps'); 
ylabel('Lateral acceleration in mps2'); 
zlabel('Corrective steering uncertainty in rad'); 
