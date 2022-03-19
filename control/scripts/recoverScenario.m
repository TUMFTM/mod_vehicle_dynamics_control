function recoverScenario(struct, idx_recover, mode, file_name)

% Authors:       
%       Salih Guemues   
%       Alexander Wischnewski
% Description:  
%   function used to recalculate one simulation step to recover the
%   specific QP definition of the (soft) TMPC
% Inputs/parameters:
%   struct:      Log file struct
%   idx_recover: ID of the optimization problem which is plotted (debug_slow.tmpc_cnt)
%   mode:        Choose 'DD' for data dictionary parameters or 'script' for setupTMPCstruct to
%                   obtain the parameters
%   file_name:   if given, store results in seperate .mat file for dedicated visualization purposes

%% -------------------- HOW TO USE ---------------------------- %%
% Run Simulink model controller_dev -> 
% execute convertSimLogs(logsout,'SimTest.mat'); ->
% load data via data = load('SimTest.mat');  
% call this script via recoverScenario(data, t_recover, mode) and specify t_recover and mode

%% -------------------- DEFINE PARAMETERS ---------------------------- %%
% maximum number of iterations (high to check whether online solution may not have converged)
max_iter = 1000;

%% ---------------------- IMPORT CONTROLLER AND VEHICLE DATA ------------------------- %%
% get access to dictionary with controller parameters
sysDict = Simulink.data.dictionary.open('il_mvdc_mpc.sldd');
dDataSectObj2 = getSection(sysDict,'Design Data');
if(strcmp(mode, 'DD'))
    % load controller parameters from data dictionary
    sys = getValue(getEntry(dDataSectObj2,'sys'));
elseif(strcmp(mode, 'script'))
    % load controller parameters from initialize script
    sys = setupTMPCStruct(false); 
else
    disp('No valid mode has been specified. Use DD or script');
end
help = getValue(getEntry(dDataSectObj2,'P_VDC_VirtualController'));
P_VDC_VirtualController = help.Value;
help = getValue(getEntry(dDataSectObj2,'P_VDC_PositiveAxLimScale'));
P_VDC_PositiveAxLimScale = help.Value;

% get access to dictionary with controller parameters
vehDict = Simulink.data.dictionary.open('il_vehicleparameter.sldd');
dDataSectObj3 = getSection(vehDict,'Design Data');
% get some more vehicle parameters
drag_coefficient = getValue(getEntry(dDataSectObj3, 'drag_coefficient'));
help = getValue(getEntry(dDataSectObj3,'roh_air'));
roh_air = help.Value;
vehiclemass_kg = getValue(getEntry(dDataSectObj3, 'vehiclemass_kg'));

%% ------------ IMPORT TRAJECTORY, VEHICLE, PREDICTION DATA AND RECONSTRUCT SCENARIO --------------- %%
% create TargetTrajectory struct and fill with logged data
TargetTrajectory.LapCnt         = struct.debug_slow.debug_slow_tartraj_LapCnt.Data(idx_recover,:);
TargetTrajectory.TrajCnt        = struct.debug_slow.debug_slow_tartraj_TrajCnt.Data(idx_recover,:);
TargetTrajectory.PointIdx       = uint16(0); 
TargetTrajectory.s_loc_m        = struct.debug_slow.debug_slow_tartraj_s_loc_m.Data(idx_recover,:)'; 
TargetTrajectory.s_glob_m       = struct.debug_slow.debug_slow_tartraj_s_glob_m.Data(idx_recover,:)'; 
TargetTrajectory.x_m            = struct.debug_slow.debug_slow_tartraj_x_m.Data(idx_recover,:)'; 
TargetTrajectory.y_m            = struct.debug_slow.debug_slow_tartraj_y_m.Data(idx_recover,:)';
TargetTrajectory.psi_rad        = struct.debug_slow.debug_slow_tartraj_psi_rad.Data(idx_recover,:)';
TargetTrajectory.kappa_radpm    = struct.debug_slow.debug_slow_tartraj_kappa_radpm.Data(idx_recover,:)';
TargetTrajectory.v_mps          = struct.debug_slow.debug_slow_tartraj_v_mps.Data(idx_recover,:)';
TargetTrajectory.ax_mps2        = struct.debug_slow.debug_slow_tartraj_ax_mps2.Data(idx_recover,:)';
TargetTrajectory.banking_rad    = struct.debug_slow.debug_slow_tartraj_banking_rad.Data(idx_recover,:)';
TargetTrajectory.ax_lim_mps2    = struct.debug_slow.debug_slow_tartraj_ax_lim_mps2.Data(idx_recover,:)';
TargetTrajectory.ay_lim_mps2    = struct.debug_slow.debug_slow_tartraj_ay_lim_mps2.Data(idx_recover,:)';

% create VehicleDynamicState struct and fill with logged data
VehicleDynamicState.Pos.x_m             = struct.debug_slow.debug_slow_x_real_m.Data(idx_recover,:);
VehicleDynamicState.Pos.y_m             = struct.debug_slow.debug_slow_y_real_m.Data(idx_recover,:);
VehicleDynamicState.Pos.psi_rad         = struct.debug_slow.debug_slow_psi_real_rad.Data(idx_recover,:);
VehicleDynamicState.beta_rad            = struct.debug_slow.debug_slow_beta_real_rad.Data(idx_recover,:);
VehicleDynamicState.v_mps               = struct.debug_slow.debug_slow_vx_real_mps.Data(idx_recover,:);
          
ax_dist_mps2 = struct.debug_slow.debug_slow_ax_dist_mps2.Data(idx_recover, :)'; 
ay_dist_mps2 = struct.debug_slow.debug_slow_ay_dist_mps2.Data(idx_recover, :)'; 
% recalculate uncertainty matrices
UncertaintyTube = calcTubeShapeMatrices(sys.A_d, sys.B_d, P_VDC_VirtualController, sys.M_1, [ax_dist_mps2'; ay_dist_mps2'], sys.N_hor+1);
[PathPos, ~] = localTrajectoryMatching(VehicleDynamicState, TargetTrajectory); 
% extract terminal set speed
v_terminal_mps = struct.debug_slow.debug_slow_be_l_abs.Data(idx_recover, 1); 
dot_d_numerical = struct.debug_slow.debug_slow_dot_d_numerical_mps.Data(idx_recover, 1); 

% get predictions vx_pred, d_psi_pred, s_dot_pred
x_pred_log       = struct.debug_slow.debug_slow_x_pred_m.Data(idx_recover, :)'; 
y_pred_log       = struct.debug_slow.debug_slow_y_pred_m.Data(idx_recover, :)'; 
vx_pred_log      = struct.debug_slow.debug_slow_vx_pred_mps.Data(idx_recover, :)'; 
d_pred_log       = struct.debug_slow.debug_slow_d_pred_m.Data(idx_recover, :)'; 
dot_d_pred_log   = struct.debug_slow.debug_slow_dot_d_pred_mps.Data(idx_recover, :)'; 
ax_pred_log      = struct.debug_slow.debug_slow_ax_pred_mps2.Data(idx_recover, :)'; 
ax_tire_pred_log = struct.debug_slow.debug_slow_ax_tire_pred_mps2.Data(idx_recover, :)'; 
ay_pred_log      = struct.debug_slow.debug_slow_ay_pred_mps2.Data(idx_recover, :)'; 

% get linearization and target trajectory
x_traj              = struct.debug_slow.debug_slow_x_traj_m.Data(idx_recover, :)'; 
y_traj              = struct.debug_slow.debug_slow_y_traj_m.Data(idx_recover, :)'; 
psi_traj            = struct.debug_slow.debug_slow_psi_traj_rad.Data(idx_recover, :)'; 
v_traj              = struct.debug_slow.debug_slow_v_traj_mps.Data(idx_recover, :)'; 
kappa_traj          = struct.debug_slow.debug_slow_kappa_traj_radpm.Data(idx_recover, :)'; 
ax_diff_traj        = struct.debug_slow.debug_slow_ax_diff_traj_mps2m.Data(idx_recover, :)'; 
ax_traj             = struct.debug_slow.debug_slow_ax_traj_mps2.Data(idx_recover, :)'; 
ay_traj             = struct.debug_slow.debug_slow_ay_traj_mps2.Data(idx_recover, :)'; 
ax_lim_mps2         = struct.debug_slow.debug_slow_ax_lim_mps2.Data(idx_recover, :)'; 
ay_lim_mps2         = struct.debug_slow.debug_slow_ay_lim_mps2.Data(idx_recover, :)'; 
d_lim_ub_m          = struct.debug_slow.debug_slow_d_lim_ub_m.Data(idx_recover, :)'; 
d_lim_lb_m          = struct.debug_slow.debug_slow_d_lim_lb_m.Data(idx_recover, :)'; 
d_Target_m          = struct.debug_slow.debug_slow_d_Target_m.Data(idx_recover, :)'; 
dot_d_Target_mps    = struct.debug_slow.debug_slow_dot_d_Target_mps.Data(idx_recover, :)'; 
ax_traj_old         = struct.debug_slow.debug_slow_ax_traj_mps2.Data(idx_recover-1, :)'; 
ay_traj_old         = struct.debug_slow.debug_slow_ay_traj_mps2.Data(idx_recover-1, :)'; 
vx_lin_mps          = struct.debug_slow.debug_slow_vx_lin_mps.Data(idx_recover, :)'; 
kappa_lin_radpm     = struct.debug_slow.debug_slow_kappa_lin_radpm.Data(idx_recover, :)'; 
u_opt_total_sim     = struct.debug_slow.debug_slow_u_opt_total.Data(idx_recover, :)';
u_opt_old_log       = struct.debug_slow.debug_slow_u_opt_total.Data(idx_recover-1, :)'; 

%% --------------------- SETUP INITIAL QP  ------------------------------ %% 
% Define problem data by reconstructing it using the sparsity pattern
% init empty matrices
P_help = zeros(sys.osqp_n, sys.osqp_n);
A_help = zeros(sys.osqp_m, sys.osqp_n);
% fill values according to sparsity pattern
P_help(sys.P_i_lin) = sys.P_x_par;
% NOTE THAT P IS STORED AS UPPER TRIANGULAR, however calculations require
% FULL P MATRIX --> add transpose and substract diagonal elements
P_help = P_help + P_help' - diag(diag(P_help));
A_help(sys.A_i_lin) = sys.A_x_par;
q = sys.osqp_qpar;
l = sys.l_par;
u = sys.u_par;
P = sparse(P_help);
A = sparse(A_help);
% Create an OSQP object
prob = osqp;
% change settings
settings = prob.default_settings();
settings.alpha = 1.0;
settings.verbose = true;
settings.scaling = 0;  % number of scaling iterations
settings.max_iter = max_iter;
settings.warm_start = true; 
settings.eps_abs = 1e-5;
settings.eps_rel = 1e-5;
% Setup workspace, change alpha parameter and disable prints
prob.setup(P, q, A, l, u, settings);

%% ----------- CALL prepareOptimizationProblem TO RECOVER QP ----------------- %%
[f, lb, ub, A_x, be_u_abs, be_l_abs, ~, error_state] =...
    prepareOptimizationProblem(VehicleDynamicState, dot_d_numerical, PathPos, ...
    v_traj, ax_diff_traj, ax_traj, ay_traj, ax_lim_mps2, ay_lim_mps2, ...
    d_lim_ub_m, d_lim_lb_m, d_Target_m, dot_d_Target_mps, ...
    vx_lin_mps, kappa_lin_radpm, UncertaintyTube, v_terminal_mps, ...
    ax_traj_old, ay_traj_old, u_opt_old_log, ...
    sys, P_VDC_VirtualController, drag_coefficient, roh_air, vehiclemass_kg, ...
    P_VDC_PositiveAxLimScale, true, 0.9);

%% --------------------- UPDATE AND SOLVE QP  ------------------------------ %% 
% update QP vectors
prob.update('q', f, 'l', lb, 'u', ub);
prob.update('Ax', A_x);
% solve problem and get solution. two runs necessary to replicate warm start behavior of online
% solution while running the car.
res = prob.solve();
u_opt_total = res.x;
% meaning of status as in https://osqp.org/docs/interfaces/status_values.html
status = res.info.status_val; 

% debug output
disp(['The problem has been solved during reoptimization with status: ' num2str(status)]); 
disp(['It took ' num2str(res.info.iter) ' iterations and ' num2str(res.info.solve_time) ' seconds']); 

%% ----------- CALL transformMPCResult TO RECOVER OPTIMIZED TRAJ. AND PREDICTIONS ----------------- %%
[u_opt_total, x_pred, y_pred, x_pred_left, y_pred_left, x_pred_right, y_pred_right, ...
    vx_pred, d_pred, dot_d_pred, ~, ax_pred, ax_tire_pred, ay_pred, ~, ~]  ...
    = transformMPCResult(status, u_opt_total, error_state,...
    x_traj, y_traj, psi_traj, v_traj, ax_diff_traj, d_Target_m, ax_lim_mps2, ay_lim_mps2, ...
    vx_lin_mps, kappa_lin_radpm, UncertaintyTube, ...
    sys, drag_coefficient, roh_air, vehiclemass_kg, P_VDC_PositiveAxLimScale);

%% plotting of optimization problem result

% recalculate tubes
[lb_v, ub_v] = calcBoundsPlot(zeros(sys.N_hor, 1), UncertaintyTube, false, P_VDC_VirtualController, sys.N_hor, repmat([1; 0; 0], 1, sys.N_hor));
[lb_d, ub_d] = calcBoundsPlot(zeros(sys.N_hor, 1), UncertaintyTube, false, P_VDC_VirtualController, sys.N_hor, repmat([0; 1; 0], 1, sys.N_hor));
[lb_dot_d, ub_dot_d] = calcBoundsPlot(zeros(sys.N_hor, 1), UncertaintyTube, false, P_VDC_VirtualController, sys.N_hor, repmat([0; 0; 1], 1, sys.N_hor));
[~, ub_tire1] = calcBoundsPlot(zeros(sys.N_hor+1, 1), UncertaintyTube, true, ...
    P_VDC_VirtualController, sys.N_hor+1, [1./(P_VDC_PositiveAxLimScale*ax_lim_mps2), 1./ay_lim_mps2]');
[~, ub_tire2] = calcBoundsPlot(zeros(sys.N_hor+1, 1), UncertaintyTube, true, ...
    P_VDC_VirtualController, sys.N_hor+1, [1./ax_lim_mps2, 1./ay_lim_mps2]');
[~, ub_tire3] = calcBoundsPlot(zeros(sys.N_hor+1, 1), UncertaintyTube, true, ...
    P_VDC_VirtualController, sys.N_hor+1, [-1./ax_lim_mps2, 1./ay_lim_mps2]');
[~, ub_tire4] = calcBoundsPlot(zeros(sys.N_hor+1, 1), UncertaintyTube, true, ...
    P_VDC_VirtualController, sys.N_hor+1, [-1./(P_VDC_PositiveAxLimScale*ax_lim_mps2), 1./ay_lim_mps2]');

% calculate limits for ax depending on positive accelerations 
idx_pos = ax_traj > 0;
ax_lim_mps2_traj = ax_lim_mps2; 
ax_lim_mps2_traj(idx_pos) = P_VDC_PositiveAxLimScale*ax_lim_mps2(idx_pos)'; 
idx_pos = ax_tire_pred > 0; 
ax_lim_mps2_pred = ax_lim_mps2; 
ax_lim_mps2_pred(idx_pos) = P_VDC_PositiveAxLimScale*ax_lim_mps2(idx_pos)'; 
idx_pos = ax_tire_pred_log > 0; 
ax_lim_mps2_pred_log = ax_lim_mps2; 
ax_lim_mps2_pred_log(idx_pos) = P_VDC_PositiveAxLimScale*ax_lim_mps2(idx_pos)' ;

figure; 
grid on; hold on; 
plot(x_traj, y_traj); 
plot(x_pred, y_pred);
plot(x_pred_log, y_pred_log, '*'); 
plot(x_pred_left, y_pred_left, 'k--'); 
plot(x_pred_right, y_pred_right, 'k--', 'HandleVisibility','off'); 
grid on; axis equal; 
legend('Target', 'Optimized', 'Logged', 'Tube'); 
xlabel('x East in m'); 
ylabel('y North in m'); 

% compare target and reoptimized trajectory
figure; 
ax11 = subplot(2, 2, 1); 
grid on; hold on; 
plot(v_traj); 
plot(vx_pred); 
plot(vx_pred_log, '*'); 
plot(vx_pred(1:sys.N_hor) + ub_v', 'k--'); 
plot(vx_pred(1:sys.N_hor) + lb_v', 'k--', 'HandleVisibility','off'); 
plot([sys.N_hor+1, sys.N_hor+3], [be_u_abs(1), be_u_abs(1)], 'k'); 
plot([sys.N_hor+1, sys.N_hor+3], [be_l_abs(1), be_l_abs(1)], 'k', 'HandleVisibility','off'); 
plot(vx_lin_mps); 
xlabel('Discretization points'); 
ylabel('Velocity in mps'); 
legend('Target', 'Optimized', 'Logged', 'Tube', 'Terminal Set', 'Linearization'); 

ax12 = subplot(2, 2, 2); 
grid on; hold on; 
plot(ax_traj./ax_lim_mps2_traj + kappa_traj.*v_traj.^2./ay_lim_mps2); 
plot(ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2); 
plot(ax_tire_pred_log./ax_lim_mps2_pred_log + ay_pred_log./ay_lim_mps2, '*'); 
plot(ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2 + ub_tire1', 'k--'); 
plot(ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2 - ub_tire2', 'k--', 'HandleVisibility','off'); 
plot([0, sys.N_hor], [1, 1], 'k'); 
plot([0, sys.N_hor], [-1, -1], 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Acceleration constraint normalized'); 
legend('Target', 'Optimized', 'Logged', 'Tube', 'Limit'); 

ax13 = subplot(2, 2, 3); 
grid on; hold on; 
plot(d_Target_m)
plot(d_pred);  
plot(d_pred_log, '*'); 
plot(d_pred(1:sys.N_hor) + ub_d', 'k--'); 
plot(d_pred(1:sys.N_hor) + lb_d', 'k--', 'HandleVisibility','off'); 
plot(d_lim_ub_m, 'k'); 
plot(d_lim_lb_m, 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Lateral deviation in m'); 
legend('Target', 'Optimized', 'Logged', 'Tube', 'Limit'); 

ax14 = subplot(2, 2, 4); 
grid on; hold on; 
plot(-ax_traj./ax_lim_mps2 + kappa_traj.*v_traj.^2./ay_lim_mps2); 
plot(-ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2); 
plot(-ax_tire_pred_log./ax_lim_mps2_pred_log + ay_pred_log./ay_lim_mps2, '*'); 
plot(-ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2 - ub_tire3', 'k--'); 
plot(-ax_tire_pred./ax_lim_mps2_pred + ay_pred./ay_lim_mps2 + ub_tire4', 'k--', 'HandleVisibility','off'); 
plot([0, sys.N_hor], [1, 1], 'k'); 
plot([0, sys.N_hor], [-1, -1], 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Acceleration constraint normalized');
legend('Target', 'Optimized', 'Logged', 'Tube', 'Limit'); 

linkaxes([ax11, ax12, ax13, ax14], 'x'); 

figure; 
ax21 = subplot(2, 3, 1); 
grid on; hold on; 
plot(ax_traj); 
plot(ax_pred); 
plot(ax_pred_log, '*'); 
plot(-u_opt_total(1:2:2*(sys.N_hor+1))); 
plot(ax_lim_mps2, 'k'); 
plot(-ax_lim_mps2, 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Acceleration x in mps2'); 
legend('Target', 'Optimized', 'Logged', 'delta ax', 'Limit'); 

ax22 = subplot(2, 3, 2); 
grid on; hold on; 
plot(kappa_traj.*v_traj.^2); 
plot(ay_pred); 
plot(ay_pred_log, '*'); 
plot(u_opt_total(2:2:2*(sys.N_hor+1))); 
plot(ay_lim_mps2, 'k'); 
plot(-ay_lim_mps2, 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Acceleration y in mps2'); 
legend('Target', 'Optimized', 'Logged', 'delta ay', 'Limit'); 

ax23 = subplot(2, 3, 3); 
grid on; hold on; 
plot(dot_d_Target_mps)
plot(dot_d_pred);  
plot(dot_d_pred_log, '*'); 
plot(dot_d_pred_log(1:sys.N_hor) + ub_dot_d', 'k--'); 
plot(dot_d_pred_log(1:sys.N_hor) + lb_dot_d', 'k--', 'HandleVisibility','off'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Lateral deviation derivative in mps'); 
legend('Target', 'Optimized', 'Logged', 'Tube'); 

ax26 = subplot(2, 3, 4); 
grid on; hold on; 
plot(u_opt_total(2*(sys.N_hor+1)+3:sys.n_slacks:end), 'b'); 
plot(u_opt_total_sim(2*(sys.N_hor+1)+3:sys.n_slacks:end), 'b*'); 
plot(u_opt_total(2*(sys.N_hor+1)+4:sys.n_slacks:end), 'c'); 
plot(u_opt_total_sim(2*(sys.N_hor+1)+4:sys.n_slacks:end), 'c*'); 
plot([0, sys.N_hor], [0, 0], 'k'); 
plot([0, sys.N_hor], [1.5, 1.5]/2, 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Slacks lateral'); 
legend('Optimized upper', 'Logged upper', 'Optimized lower', 'Optimized lower', 'Limit'); 

ax27 = subplot(2, 3, 5); 
grid on; hold on; 
plot(u_opt_total(2*(sys.N_hor+1)+1:sys.n_slacks:end)); 
plot(u_opt_total_sim(2*(sys.N_hor+1)+1:sys.n_slacks:end), '*'); 
plot([0, sys.N_hor], [0, 0], 'k'); 
plot([0, sys.N_hor], [sys.slack_lim_rel, sys.slack_lim_rel], 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Slacks tire 1 lower'); 
legend('Optimized upper', 'Logged', 'Limit'); 

ax28 = subplot(2, 3, 6); 
grid on; hold on; 
plot(u_opt_total(2*(sys.N_hor+1)+2:sys.n_slacks:end)); 
plot(u_opt_total_sim(2*(sys.N_hor+1)+2:sys.n_slacks:end), '*'); 
plot([0, sys.N_hor], [0, 0], 'k'); 
plot([0, sys.N_hor], [sys.slack_lim_rel, sys.slack_lim_rel], 'k'); 
grid on; 
xlabel('Discretization points'); 
ylabel('Slacks tire 2 lower'); 
legend('Optimized', 'Logged', 'Limit'); 

linkaxes([ax21, ax22, ax23, ax26, ax27, ax28], 'x'); 

%% store data in external file for further visualization
if nargin > 3
    save(file_name); 
end

end
