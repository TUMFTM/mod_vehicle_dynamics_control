function [v_target, d_target, dot_d_target, v_pred, tire_util_1, tire_util_2, tire_util_pred_1, tire_util_pred_2, ...
    tPred, p_t_2, p_t_3, M_s, be_u_abs, be_l_abs, ax_lim_mps2, ay_lim_mps2, d_lim_ub_m, d_lim_lb_m, vx_lin_mps] = ...
    prepareTMPCPlotting(debug, debug_slow, tmpc_cnt, sys, P_VDC_VirtualController, P_VDC_PositiveAxLimScale)
% function prepareTMPCPlotting
% Authors:      Martin Euler         
%               Salih Gümüs
%               Alexander Wischnewski
%
% Description:  
%   helper function used to initialize everything required for plotting of the TMPC results
% Inputs/parameters:
%   mode:   determines how the parameters for the controller are calculated
%   data:   log data structure
% Outputs:
%   sys:                            struct with parameters
%   P_VDC_DyLim                     maximum lateral deviation constraint
%   P_VDC_AxLim                     maximum longitudinal acceleration constraint
%   P_VDC_AyLim                     maximum lateral acceleration constraint
%   dist                            disturbance vector with two elements (longitudinal and lateral)
%   errorState_1                    error state velocity in mps (actual)
%   errorState_2                    error state lateral deviation in m (actual) 
%   errorState_3                    error state lateral deviation derivative in mps (actual) 
%   tPred                           time vector with prediction times of TMPC
%   dist_mean                       vector with predicted disturbances (longitudinal and lateral)
%   p_t_1                           error state velocity in mps (prediction)
%   p_t_2                           error state lateral deviation in m (prediction)
%   p_t_3                           error state lateral deviation derivative in mps (prediction)
%   M_s                             shape matrices of uncertainty tube
%   be_u                            terminal set, upper bounds
%   be_l                            terminal set, lower bounds
%   ax_lim_mps2                     acceleration limits in long. direction
%   ay_lim_mps2                     acceleration limits in lat. direction 
%   d_lim_ub_m                      lateral constraint upper bound
%   d_lim_lb_m                      lateral constraint lower bound
%   vx_lin_mps                      Linearization speed

%% ------------------ import stored data from simulation ----------------%%

% calculate time vector for prediction
tPred_start = debug_slow.debug_slow_tmpc_cnt.Time(tmpc_cnt); 
tPred = tPred_start:sys.Ts:(tPred_start+sys.Ts*sys.N_hor); 

% extract predictions for the error states
p_t_2 = debug_slow.debug_slow_d_pred_m.Data(tmpc_cnt, :);
p_t_3 = debug_slow.debug_slow_dot_d_pred_mps.Data(tmpc_cnt, :);
v_pred = debug_slow.debug_slow_vx_pred_mps.Data(tmpc_cnt, :);

% extract constraints 
ax_lim_mps2 = debug_slow.debug_slow_ax_lim_mps2.Data(tmpc_cnt, :); 
% modify ax constraintes where prediciton is positive (RWD factor)
idx_pos = debug_slow.debug_slow_ax_tire_pred_mps2.Data(tmpc_cnt, :) > 0; 
ax_lim_mps2(idx_pos) = P_VDC_PositiveAxLimScale*ax_lim_mps2(idx_pos); 
ay_lim_mps2 = debug_slow.debug_slow_ay_lim_mps2.Data(tmpc_cnt, :); 
d_lim_ub_m = debug_slow.debug_slow_d_lim_ub_m.Data(tmpc_cnt, :); 
d_lim_lb_m = debug_slow.debug_slow_d_lim_lb_m.Data(tmpc_cnt, :); 

% extract linearization 
vx_lin_mps = debug_slow.debug_slow_vx_lin_mps.Data(tmpc_cnt, :); 

% calculate velocity prediction
v_target = debug_slow.debug_slow_v_traj_mps.Data(tmpc_cnt, :)'; 
d_target = debug_slow.debug_slow_d_Target_m.Data(tmpc_cnt, :)'; 
dot_d_target = debug_slow.debug_slow_dot_d_Target_mps.Data(tmpc_cnt, :)'; 

% calculate tire utilization 
tire_util_1 = debug.debug_mvdc_trajectory_driver_perf_TireUtilization_c1; 
tire_util_2 = debug.debug_mvdc_trajectory_driver_perf_TireUtilization_c2; 
tire_util_pred_1 = debug_slow.debug_slow_ax_tire_pred_mps2.Data(tmpc_cnt, :)./ax_lim_mps2 + ...
                   debug_slow.debug_slow_ay_pred_mps2.Data(tmpc_cnt, :)./ay_lim_mps2; 
tire_util_pred_2 = debug_slow.debug_slow_ax_tire_pred_mps2.Data(tmpc_cnt, :)./ax_lim_mps2 - ...
                   debug_slow.debug_slow_ay_pred_mps2.Data(tmpc_cnt, :)./ay_lim_mps2; 
               
ax_dist_mps2 = debug_slow.debug_slow_ax_dist_mps2.Data(tmpc_cnt, :)'; 
ay_dist_mps2 = debug_slow.debug_slow_ay_dist_mps2.Data(tmpc_cnt, :)'; 

%% ---------------------- calc tube shape matrices --------------------- %%
M_s = calcTubeShapeMatrices(sys.A_d, sys.B_d, P_VDC_VirtualController, sys.M_1, [ax_dist_mps2'; ay_dist_mps2'], sys.N_hor+1);

% retrieve terminal set bounds from the log
be_u_abs = debug_slow.debug_slow_be_u_abs.Data(tmpc_cnt, :); 
be_l_abs = debug_slow.debug_slow_be_l_abs.Data(tmpc_cnt, :); 
