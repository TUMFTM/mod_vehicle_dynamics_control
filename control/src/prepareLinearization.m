function [flag_s_request, PathPos, x_traj, y_traj, psi_traj, v_traj, kappa_traj, ...
    ax_diff_traj, ax_traj, ay_traj, ax_lim_mps2_tartraj, ay_lim_mps2_tartraj, ...
    tube_r_m, tube_l_m, d_Target_m, dot_d_Target_mps, ...
    d_lim_ub_m, d_lim_lb_m, ax_dist_mps2, ay_dist_mps2, ...
    vx_lin, s_dot_lin, kappa_lin, UncertaintyTube, v_terminal_mps] = ...
    prepareLinearization(VehicleDynamicState, TargetTrajectory, M_tilde, LatAcc_P_cov, ...
    s_dot_pred_old, vx_pred_old, s_dot_lin_old, vx_lin_old, sys, ...
    P_VDC_MinVelSlipCalc_mps, P_VDC_RTISQP_alpha_old, P_VDC_RTISQP_alpha_target, ...
    P_VDC_IncreaseUncertaintyPerStep_perc, P_VDC_VirtualController, ...
    P_VDC_ControlMargin_ax_mps2, P_VDC_ControlMargin_ay_mps2, acc_control_learning, ...
    P_VDC_TerminalSetBrakeTime_s, drag_coefficient, roh_air, vehiclemass_kg, ...
    P_VDC_TuneTerminalSet_mps)

% Authors:       Alexander Wischnewski

% Description:  
%   function used to determine the linearization trajectory for the optimization problem

%% ------- define optimization problem variables ----------------------- %%
% these variables are repeated due to codegen restrictions. They must be adjusted manually! 
N = 40; % optimization horizon length should equal sys.N_hor

%% --------- call path matching function: project current position to target path -------------------------- %%
% first point is determined by matching to handle occuring uncertainties in the prediciton. 
[PathPos, ~] = localTrajectoryMatching(VehicleDynamicState, TargetTrajectory); 
% calculate points for readout of curvature 
s_dot_lin = P_VDC_RTISQP_alpha_old*s_dot_lin_old + (1-P_VDC_RTISQP_alpha_old)*s_dot_pred_old; 
% prediction is not shifted since discretization time is much larger than sample time
% limit from below to prevent zero predictions which can cause problems with interpolation
s_dot_lin = max(P_VDC_MinVelSlipCalc_mps, s_dot_lin); 
s_request_m = [PathPos.s_m; PathPos.s_m + cumsum(s_dot_lin*sys.Ts)]; 
% Check if requested arc length is within range of TargetTrajectory
if max(s_request_m) <= max(TargetTrajectory.s_loc_m) ...
        && min(s_request_m) >= min(TargetTrajectory.s_loc_m)
    flag_s_request = 0;
else
    flag_s_request = 1;
end
[TrajectoryPoints] = trajectoryInterpolation(TargetTrajectory, s_request_m);
% interpolated target trajectory
% last point can be skipped as it is only needed for derivative calculation
x_traj = TrajectoryPoints.x_m(1:end-1); 
y_traj = TrajectoryPoints.y_m(1:end-1); 
psi_traj = TrajectoryPoints.psi_rad(1:end-1); 
v_traj = TrajectoryPoints.v_mps(1:end-1); 
kappa_traj = TrajectoryPoints.kappa_radpm(1:end-1); 
ax_traj = TrajectoryPoints.ax_mps2(1:end-1); 
ax_lim_mps2_tartraj = TrajectoryPoints.ax_lim_mps2(1:end-1) + P_VDC_ControlMargin_ax_mps2; 
ay_lim_mps2_tartraj = TrajectoryPoints.ay_lim_mps2(1:end-1) + P_VDC_ControlMargin_ay_mps2; 
tube_r_m = TrajectoryPoints.tube_r_m(1:end-1); 
tube_l_m = TrajectoryPoints.tube_l_m(1:end-1);
ay_traj = kappa_traj.*v_traj.^2;
ax_diff_traj = diff(TrajectoryPoints.v_mps)./diff(s_request_m); 

%% calculate disturbance vectors as percantage of constraints 
ax_dist_mps2 = (M_tilde(1) + (0:1:N)'*P_VDC_IncreaseUncertaintyPerStep_perc).*ax_lim_mps2_tartraj/100; 
ay_dist_mps2 = (M_tilde(2) + (0:1:N)'*P_VDC_IncreaseUncertaintyPerStep_perc).*ay_lim_mps2_tartraj/100; 

%% -- calculate shape matrices for the tube within the prediction horizon -- %%
UncertaintyTube = calcTubeShapeMatrices(sys.A_d, sys.B_d, P_VDC_VirtualController, sys.M_1, [ax_dist_mps2'; ay_dist_mps2'], N+1);

%% determine terminal set
% get shape matrix of terminal state
input_shape_matrix = P_VDC_VirtualController*UncertaintyTube(:, end-2:end)*P_VDC_VirtualController'; 
% tire constraint uncertainty
tire_const_unc = P_VDC_TuneTerminalSet_mps*sqrt([1/ax_lim_mps2_tartraj(end), 1/ay_lim_mps2_tartraj(end)] * ...
    input_shape_matrix * ...
    [1/ax_lim_mps2_tartraj(end); 1/ay_lim_mps2_tartraj(end)]);
% compare planned and max. accelerations 
P_VDC_AccLimitsPlanner_norm = ...
    max((ax_lim_mps2_tartraj(end)-P_VDC_ControlMargin_ax_mps2)/ax_lim_mps2_tartraj(end), ...
        (ay_lim_mps2_tartraj(end)-P_VDC_ControlMargin_ay_mps2)/ay_lim_mps2_tartraj(end)); 
% limit tire_const_uncertainty factor as it can become larger than one in case that the input 
% uncertainty is larger than the maximum limits 
terminal_set_scale_factor = sqrt((1 - min(tire_const_unc, 1))/P_VDC_AccLimitsPlanner_norm); 
v_terminal_mps = min(terminal_set_scale_factor, 1)*v_traj(end);

%% calculate linearization trajectory 
% the new linearization trajectory is a weighted combination of the target trajectory, the previous
% linearization and the previous solution. This ensures smooth transition between the optimization
% problems. It is not shifted since the discretization time is much larger than the sample rate. 

% calculate a guess from the final solution from the actual target velocity and the terminal set velocity
% the square ensures that at the beginning the target is more pronounced than the terminal set
vx_sol_guess = (1-((0:1:N)/N).^2)'.*v_traj + (((0:1:N)/N).^2)'*v_terminal_mps;
vx_lin_guess = (P_VDC_RTISQP_alpha_old*vx_lin_old + P_VDC_RTISQP_alpha_target*vx_sol_guess)/...
    (P_VDC_RTISQP_alpha_old + P_VDC_RTISQP_alpha_target);
% calculate the linearization 
vx_lin = (((0:1:N)/N).^2)'.*vx_lin_guess + (1-((0:1:N)/N).^2)'.*vx_pred_old; 
% ensure that linearization is not zero 
vx_lin = max(vx_lin, P_VDC_MinVelSlipCalc_mps);
% use target trajectory curvature for linearization
kappa_lin = kappa_traj; 
ay_lin = kappa_lin.*vx_lin.^2; 

%% calculate features for long. and lat. acc feedforward models along target trajectory
ax_feat = zeros(N, 2); 
ay_feat = zeros(N, length(acc_control_learning.bf_vx_mps)+1);
% for i = 1:1:N
%     dist_ay_feat = ((acc_control_learning.bf_vx_mps' - vx_lin(i))/acc_control_learning.vx_width_mps).^2 + ...
%         ((acc_control_learning.bf_ay_req_mps2' - ay_lin(i))/acc_control_learning.ay_width_mps2).^2; 
%     ay_feat(i, :) = [exp(-dist_ay_feat), ay_traj(i)]; 
% end
% extract uncertainty for all feature points
ax_uncertainty_mps2 = zeros(N+1, 1); 
ay_uncertainty_mps2 = zeros(N+1, 1); 
% for i = 1:1:N
%    ay_uncertainty_mps2(i) = sqrt(ay_feat(i, :)*LatAcc_P_cov*ay_feat(i, :)')*vx_lin(i)^2;  
% end

% update uncertainties 
% ay_dist_mps2 = ay_dist_mps2 + ay_uncertainty_mps2; 

%% calculate details for path model
d_Target_m = zeros(N+1, 1); 
dot_d_Target_mps = zeros(N+1, 1); 
d_lim_ub_m = d_Target_m + tube_l_m; 
d_lim_lb_m = d_Target_m - tube_r_m; 
