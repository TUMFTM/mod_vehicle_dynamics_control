function visualizeAccControlLearning(data, idx_recover)

% Authors:       
%       Alexander Wischnewski
% Description:  
%   visualizes the data used for learning of the acceleration controllers
% Inputs/parameters:
%   data:           Log file struct
%   idx_recover:    Recovery index

idx_skip = 10; 
l_wheelbase_m = 2.971;

% load learning basis functions
sysDict = Simulink.data.dictionary.open('il_mvdc_mpc.sldd');
dDataSectObj2 = getSection(sysDict,'Design Data');
acc_control_learning = getValue(getEntry(dDataSectObj2, 'acc_control_learning'));
    
%% basic data visualization
figure; 
scatter3(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data(1:idx_skip:end)); 
xlabel('Lateral acceleration request in mps2'); 
ylabel('Vehicle velocity in mps'); 
zlabel('Lateral acceleration in mps2'); 
xlim([-40 40]); 

figure; 
scatter3(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_dPsi_radps.Data(1:idx_skip:end)); 
xlabel('Lateral acceleration request in mps2'); 
ylabel('Vehicle velocity in mps'); 
zlabel('Yaw rate in radps'); 
xlim([-40 40]); 

figure; 
scatter3(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_beta_rad.Data(1:idx_skip:end)); 
xlabel('Lateral acceleration request in mps2'); 
ylabel('Vehicle velocity in mps'); 
zlabel('Chassis side slip angle in rad'); 
xlim([-40 40]); 

%% visualize lateral acceleration learning 
% remove neutral steer from ay 
ay_residual = data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data - ...
    data.debug.debug_mvdc_curvvel_tracking_debug_Req_Delta_rad.Data/l_wheelbase_m.*...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data.^2; 
figure; 
scatter3(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(1:idx_skip:end), ...
    ay_residual(1:idx_skip:end)); 
xlabel('Lateral acceleration request in mps2'); 
ylabel('Vehicle velocity in mps'); 
zlabel('Lateral acceleration residual in mps2'); 
xlim([-40 40]); 
zlim([-5 5]); 

% remove neutral steer from delta 
delta_diff_neutral = data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_FB_Dist_rad.Data + ...
    data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_FB_ay_rad.Data + ...
    data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_FB_Delta_rad.Data + ...
    data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_FFdyn_rad.Data + ...
    data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_FFLearned_rad.Data;     

% visualize residual model at idx_recover
weights = data.debug_slow.debug_slow_LatAcc_FFweights.Data(idx_recover, :); 
acc_control_learning.w0 = weights'; 
visz_vx_mps = 0:5:100;
visz_ay_req_mps2 = -40:0.5:40; 
[Visz_X, Visz_Y] = meshgrid(visz_vx_mps, visz_ay_req_mps2); 
Visz_Z = zeros(length(visz_ay_req_mps2), length(visz_vx_mps)); 
for i = 1:1:length(visz_vx_mps)
    for j = 1:1:length(visz_ay_req_mps2)
        % apply basis functions to each sample point
        dist = ((acc_control_learning.bf_vx_mps - visz_vx_mps(i))./acc_control_learning.vx_width_mps).^2 + ...
                       ((acc_control_learning.bf_ay_req_mps2 - visz_ay_req_mps2(j))./acc_control_learning.ay_width_mps2).^2;
        Visz_Z(j, i) = weights*[exp(-dist); visz_ay_req_mps2(j)]; 
    end
end
figure; 
% surface(Visz_Y, Visz_X, Visz_Z); hold on; grid on; 
scatter3(data.debug.debug_mvdc_tmpc_fast_debug_LatAcc_Target_mps2.Data(1:idx_skip:end), ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data(1:idx_skip:end), ...
    delta_diff_neutral(1:idx_skip:end)); 
xlabel('Lateral acceleration request in mps2'); 
ylabel('Vehicle velocity in mps'); 
zlabel('Diff to neutral steer in rad'); 
xlim([-40 40]); 
zlim([-0.1 0.1]); 