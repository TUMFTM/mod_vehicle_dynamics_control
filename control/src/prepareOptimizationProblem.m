function [f, lb, ub, A_x, be_u_abs, be_l_abs, s_current_m, error_state] = ...
    prepareOptimizationProblem(VehicleDynamicState, dot_d_numerical, PathPos, v_traj, ...
    ax_diff_traj, ax_traj, ay_traj, ax_lim_mps2, ay_lim_mps2, ...
    d_lim_ub_m, d_lim_lb_m, d_Target_m, dot_d_Target_mps, ...
    vx_lin, kappa_lin, UncertaintyTube, v_terminal_mps, ax_traj_old, ay_traj_old, solution_old, ...
    sys, P_VDC_VirtualController, drag_coefficient, roh_air, vehiclemass_kg, ...
    P_VDC_PositiveAxLimScale, P_VDC_EnableNumLatErrorDer_b, P_VDC_MaxTightening, ...
    P_VDC_MinVelSlipCalc_mps)

% function prepareOptimizationProblem
% Authors:       Martin Euler
%                Salih Guemues
%                Alexander Wischnewski
%
% Description:  
%   function used to generate the constraint matrices for the TMPC-Controller 
% Inputs/parameters:
%   VehicleDynamicState:    Dynamic state of the racecar
%   PathPos:                Vehicle position on the target path
%   v_traj:                 Target trajectory for the PH (prediction horizon)
%   ax_diff_traj:           Derivative of the target trajectory velocity in the PH
%   ax_traj:                Longitudinal acceleration for the target trajectory in the PH
%   ay_traj:                Lateral acceleration for the target trajectory in the PH
%   ax_lim_mps2             Longitudinal acceleration limits of the target trajectory in the PH
%   ay_lim_mps2             Lateral acceleration limits of the target trajectory in the PH
%   d_lim_ub_m              Size of the driving tube to the right side
%   d_lim_lb_m              Size of the driving tube to the left side
%   d_Target_m              Target trajectory projected into the curvilinear coordinate frame 
%   dot_d_Target_mps        Derivative of the target trajectory projected into the c.l. coordinates
%   vx_lin                  Linearization velocity profile 
%   kappa_lin               Linearization curvature profile 
%   UncertaintyTube         Uncertainy ellipsoids for predicted uncertainty tube
%   v_terminal_set          Terminal set speed
%   ax_traj_old             Previous iteration target trajectory long. acceleration 
%   ay_traj_old             Previous iteration target trajectory lat. acceleration 
%   solution_old            Previous iteration solution 
%   P_VDC_VirtualController Virtual feedback controller used for uncertainty tube calculation 
%   drag_coefficient        Vehicle drag coefficient
%   roh_air                 Air density 
%   vehiclemass_kg          Vehicle mass
%   P_VDC_PositiveAxLimScale Scaling factor for positive accelerations to consider RWD
%
% Outputs:
%
%   f:                      linear weight matrix for QP
%   lb:                     lower bound vector for inequality constraints
%   ub:                     upper bound vector for inequality constraints
%   A_x:                    nonzero-value-vector of inequality constraint coefficient matrix A_ineq
%   be_u_abs:               upper terminal set bounds in absolute coordinates
%   be_l_abs:               lower terminal set bounds in absolute coordinates
%   s_current:              Vehicle position in raceline coordinates
%   errorState:             current errorState of the system

%% ------- define optimization problem variables ----------------------- %%
% get UNSCALED (in terms of preconditioning) constraint matrices/vectors
A_ineq = zeros(sys.osqp_m, sys.osqp_n);
A_ineq(sys.A_i_lin) = sys.A_x_par;
lb = sys.l_par;
ub = sys.u_par;

%% ------------------------- calculate current error state ------------------------------- %%
% set s_0 to current distance on racetrack
s_current_m = PathPos.s_m; 
% calculate assumed velocity. limit to lower value to prevent linearization issues. 
v_mps = max(VehicleDynamicState.v_mps, 0.5*P_VDC_MinVelSlipCalc_mps); 
% velocity difference: delta_vx = v_path - v_car;
delta_vx = v_traj(1) - v_mps;
% lateral error delta_y as output from local_path_matching with added initial offset
d = PathPos.d_m;
% Delta between vehicle and path heading by interpolation
d_psi = PathPos.psi_rad + VehicleDynamicState.beta_rad;    
% lateral error dynamics (either numerical or analytical)
if(P_VDC_EnableNumLatErrorDer_b)
    d_dot = dot_d_numerical;
else
    d_dot = v_mps * sin(d_psi);
end
% current state vector for controller (state vector of error diff. equat.)
error_state = [delta_vx; d; d_dot];
% calculate system response to initial state
x0_resp = sys.Ax0_MPC*error_state; 

%% ----------- calc linear weight matrix for QP-Solver ------------%
f = sys.osqp_qpar + 2*(x0_resp'*sys.f_x0)' - 2*(d_Target_m'*sys.f_d_m)' ...
    - 2*(dot_d_Target_mps'*sys.f_dot_d_mps)' ...
    - 2*([ax_traj_old(1); ax_traj]'*sys.f_Dax)' ...
    + 2*([ay_traj_old(1); ay_traj]'*sys.f_Day)' ...
    + 2*(solution_old(1)*sys.f_D_deltaax)' + 2*(solution_old(2)*sys.f_D_deltaay)';
% its minus the ax term since the definitions of delta ax and delta ay are changed

%% - calculation of input and state constraints within prediction horizon - %%
for j = 1:1:sys.N_hor
    % get uncertainty matrix for the inputs
    M_states_current = UncertaintyTube(:, (j-1)*3+1:(j-1)*3+3); 
    M_inputs_current = P_VDC_VirtualController*M_states_current*P_VDC_VirtualController';
    % calculate tightenings for all state variables and inputs 
    tight_delta_v_mps = sqrt(M_states_current(1, 1)); 
    tight_d_m = sqrt(M_states_current(2, 2)); 
    tight_dot_d_mps = sqrt(M_states_current(3, 3)); 
    % the factors here are swapped with respect to the inputs following this scheme
    % such that only upper bounds are given as the ax_lim has to be modified for the upper bounds
    % ay_lim*ax_util + P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    tight_tire1 = sqrt([ay_lim_mps2(j), P_VDC_PositiveAxLimScale*ax_lim_mps2(j)]*M_inputs_current*[ay_lim_mps2(j); P_VDC_PositiveAxLimScale*ax_lim_mps2(j)]); 
    % -ay_lim*ax_util + -ax_lim*ay_util <= ax_lim*ay_lim
    tight_tire2 = sqrt([-ay_lim_mps2(j), -ax_lim_mps2(j)]*M_inputs_current*[-ay_lim_mps2(j); -ax_lim_mps2(j)]); 
    % -ay_lim*ax_util + ax_lim*ay_util <= ax_lim*ay_lim
    tight_tire3 = sqrt([-ay_lim_mps2(j), ax_lim_mps2(j)]*M_inputs_current*[-ay_lim_mps2(j); ax_lim_mps2(j)]); 
    % ay_lim*ax_util + -P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    tight_tire4 = sqrt([ay_lim_mps2(j), -P_VDC_PositiveAxLimScale*ax_lim_mps2(j)]*M_inputs_current*[ay_lim_mps2(j); -P_VDC_PositiveAxLimScale*ax_lim_mps2(j)]); 
    % calculate normalization factor for acceleration constraints
    norm_acc_con_upper = 1/(P_VDC_PositiveAxLimScale*ax_lim_mps2(j)*ay_lim_mps2(j));
    norm_acc_con_lower = 1/(ax_lim_mps2(j)*ay_lim_mps2(j));
    
    % limit tigthening to given factor
    tight_d_m = min(d_lim_ub_m(j)*P_VDC_MaxTightening, tight_d_m);
    % get tightened admissible lateral error for current step
    ub(sys.n_constr*j) = d_lim_ub_m(j) - tight_d_m - x0_resp(2+(j-1)*sys.n_sys);
    lb(sys.n_constr*j) = d_lim_lb_m(j) + tight_d_m - x0_resp(2+(j-1)*sys.n_sys);
    
    % prepare linearization around d = 0, dot_d = 0 and vx = vx_pred for longitudinal acceleration
    ax_grad_d = ax_diff_traj(j)*kappa_lin(j)*vx_lin(j);
    ax_grad_v = ax_diff_traj(j) + roh_air*drag_coefficient*vx_lin(j)/vehiclemass_kg; 
    % prepare full gradient for easier specification of constraints 
    ax_grad_states = ay_lim_mps2(j)*[ax_grad_v, ax_grad_d, 0]; 
    ax_grad_inputs = ay_lim_mps2(j)*[-1, 0]; % this is minus one since delta_ax is different sign
    ax_op = ax_diff_traj(j)*(2*vx_lin(j) - v_traj(j)) + 0.5*roh_air*drag_coefficient*vx_lin(j)^2/vehiclemass_kg;
    
    % prepare linearization around d = 0, dot_d = 0 and vx = vx_pred for lateral acceleration 
    ay_grad_d = kappa_lin(j)^2*vx_lin(j)^2; 
    ay_grad_dot_d = -ax_diff_traj(j); 
    ay_grad_v = -2*kappa_lin(j)*vx_lin(j);
    % prepare full gradient for easier specification of constraints 
    ay_grad_states_upper = P_VDC_PositiveAxLimScale*ax_lim_mps2(j)*[ay_grad_v, ay_grad_d, ay_grad_dot_d]; 
    ay_grad_inputs_upper = P_VDC_PositiveAxLimScale*ax_lim_mps2(j)*[0, 1];
    ay_grad_states_lower = ax_lim_mps2(j)*[ay_grad_v, ay_grad_d, ay_grad_dot_d]; 
    ay_grad_inputs_lower = ax_lim_mps2(j)*[0, 1];
    ay_op = kappa_lin(j)*vx_lin(j)^2 + 2*kappa_lin(j)*vx_lin(j)*(v_traj(j) - vx_lin(j));
      
    % update gradients for states and inputs in constraint matrix 
    % ay_lim*ax_util + P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    A_ineq(sys.n_constr*(j-1)+1, 1:sys.N_hor*sys.m_sys) = ...
        (ax_grad_states+ay_grad_states_upper)*sys.ABK_MPC(1+(j-1)*sys.n_sys:j*sys.n_sys, 1:sys.N_hor*sys.m_sys); 
    A_ineq(sys.n_constr*(j-1)+1, sys.m_sys*(j-1)+1:sys.m_sys*j) = ...
        (A_ineq(sys.n_constr*(j-1)+1, sys.m_sys*(j-1)+1:sys.m_sys*j) + ax_grad_inputs + ay_grad_inputs_upper); 
    % -ay_lim*ax_util + -ax_lim*ay_util <= ax_lim*ay_lim
    A_ineq(sys.n_constr*(j-1)+2, 1:sys.N_hor*sys.m_sys) = ...
        -(ax_grad_states+ay_grad_states_lower)*sys.ABK_MPC(1+(j-1)*sys.n_sys:j*sys.n_sys, 1:sys.N_hor*sys.m_sys); 
    A_ineq(sys.n_constr*(j-1)+2, sys.m_sys*(j-1)+1:sys.m_sys*j) = ...
        (A_ineq(sys.n_constr*(j-1)+2, sys.m_sys*(j-1)+1:sys.m_sys*j) - ax_grad_inputs - ay_grad_inputs_lower); 
    % -ay_lim*ax_util + ax_lim*ay_util <= ax_lim*ay_lim
    A_ineq(sys.n_constr*(j-1)+3, 1:sys.N_hor*sys.m_sys) = ...
        (-ax_grad_states+ay_grad_states_lower)*sys.ABK_MPC(1+(j-1)*sys.n_sys:j*sys.n_sys, 1:sys.N_hor*sys.m_sys); 
    A_ineq(sys.n_constr*(j-1)+3, sys.m_sys*(j-1)+1:sys.m_sys*j) = ...
        (A_ineq(sys.n_constr*(j-1)+3, sys.m_sys*(j-1)+1:sys.m_sys*j) - ax_grad_inputs + ay_grad_inputs_lower); 
    % ay_lim*ax_util + -P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    A_ineq(sys.n_constr*(j-1)+4, 1:sys.N_hor*sys.m_sys) = ...
        -(-ax_grad_states+ay_grad_states_upper)*sys.ABK_MPC(1+(j-1)*sys.n_sys:j*sys.n_sys, 1:sys.N_hor*sys.m_sys); 
    A_ineq(sys.n_constr*(j-1)+4, sys.m_sys*(j-1)+1:sys.m_sys*j) = ...
        (A_ineq(sys.n_constr*(j-1)+4, sys.m_sys*(j-1)+1:sys.m_sys*j) + ax_grad_inputs - ay_grad_inputs_upper); 
   
    % scale matrices appropriately 
    A_ineq(sys.n_constr*(j-1)+1, 1:sys.N_hor*sys.m_sys) = ...
        norm_acc_con_upper*A_ineq(sys.n_constr*(j-1)+1, 1:sys.N_hor*sys.m_sys);
    A_ineq(sys.n_constr*(j-1)+2, 1:sys.N_hor*sys.m_sys) = ...
        norm_acc_con_lower*A_ineq(sys.n_constr*(j-1)+2, 1:sys.N_hor*sys.m_sys);
    A_ineq(sys.n_constr*(j-1)+3, 1:sys.N_hor*sys.m_sys) = ...
        norm_acc_con_lower*A_ineq(sys.n_constr*(j-1)+3, 1:sys.N_hor*sys.m_sys);
    A_ineq(sys.n_constr*(j-1)+4, 1:sys.N_hor*sys.m_sys) = ...
        norm_acc_con_upper*A_ineq(sys.n_constr*(j-1)+4, 1:sys.N_hor*sys.m_sys);
    
    % limit tire tightening to resonable values 
    tight_tire1 = min(P_VDC_PositiveAxLimScale*ax_lim_mps2(j)*ay_lim_mps2(j)*P_VDC_MaxTightening, ... 
                        tight_tire1); 
    tight_tire2 = min(ax_lim_mps2(j)*ay_lim_mps2(j)*P_VDC_MaxTightening, ...
                        tight_tire2); 
    tight_tire3 = min(ax_lim_mps2(j)*ay_lim_mps2(j)*P_VDC_MaxTightening, ...
                        tight_tire3); 
    tight_tire4 = min(P_VDC_PositiveAxLimScale*ax_lim_mps2(j)*ay_lim_mps2(j)*P_VDC_MaxTightening, ...
                        tight_tire4); 
    
    % ay_lim*ax_util + P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    ub(sys.n_constr*(j-1)+1) = norm_acc_con_upper*(-ax_op*ay_lim_mps2(j) ...
        - ay_op*P_VDC_PositiveAxLimScale*ax_lim_mps2(j) ...
        - ax_grad_states*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - ay_grad_states_upper*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - tight_tire1 + P_VDC_PositiveAxLimScale*ax_lim_mps2(j) * ay_lim_mps2(j));
    % -ay_lim*ax_util + -ax_lim*ay_util <= ax_lim*ay_lim
    ub(sys.n_constr*(j-1)+2) = -norm_acc_con_lower*(-ax_op*ay_lim_mps2(j) ...
        - ay_op*ax_lim_mps2(j) ...
        - ax_grad_states*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - ay_grad_states_lower*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        + tight_tire2 - ax_lim_mps2(j) * ay_lim_mps2(j));
    % -ay_lim*ax_util + ax_lim*ay_util <= ax_lim*ay_lim
    ub(sys.n_constr*(j-1)+3) = norm_acc_con_lower*(ax_op*ay_lim_mps2(j) ...
        - ay_op*ax_lim_mps2(j) ...
        + ax_grad_states*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - ay_grad_states_lower*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - tight_tire3 + ax_lim_mps2(j) * ay_lim_mps2(j));
    % ay_lim*ax_util + -P_VDC_PositiveAxLimScale*ax_lim*ay_util <= P_VDC_PositiveAxLimScale*ax_lim*ay_lim
    ub(sys.n_constr*(j-1)+4) = -norm_acc_con_upper*(ax_op*ay_lim_mps2(j) ...
        - ay_op*P_VDC_PositiveAxLimScale*ax_lim_mps2(j) ...
        + ax_grad_states*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        - ay_grad_states_upper*x0_resp(1+(j-1)*sys.n_sys:j*sys.n_sys) ...
        + tight_tire4 - P_VDC_PositiveAxLimScale*ax_lim_mps2(j) * ay_lim_mps2(j));
end

%% -------------------- terminal state constraints ----------------------- %%
be_u = zeros(3, 1); 
be_l = zeros(3, 1); 
be_u(1) = inf; 
be_l(1) = v_traj(end) - v_terminal_mps; 

% lateral error is constrained to be 
tight_terminal_d_m = sqrt(UncertaintyTube(2, end-1));
tight_terminal_d_m = min(d_lim_ub_m(end)*P_VDC_MaxTightening, tight_terminal_d_m);
be_u(2) = d_lim_ub_m(end) - tight_terminal_d_m;
be_l(2) = d_lim_lb_m(end) + tight_terminal_d_m;
% calculate lateral erorr speed as average of the last five values as they might be noisy
be_u(3) = mean(dot_d_Target_mps(end-5:end));
be_l(3) = mean(dot_d_Target_mps(end-5:end)); 
% write terminal constraints to logs
be_u_abs = be_u; 
be_l_abs = be_l; 
% add offset velocity to make logging meaningful
be_u_abs(1) = v_traj(end) - be_u_abs(1); 
be_l_abs(1) = v_traj(end) - be_l_abs(1); 
% calculate ub for terminal constraint on states
ub(sys.n_constr*sys.N_hor+1:sys.n_constr*sys.N_hor+sys.n_sys) = be_u - ...
    x0_resp(sys.N_hor*sys.n_sys+1:(sys.N_hor+1)*sys.n_sys);
% calculate lb for terminal constraint on states
lb(sys.n_constr*sys.N_hor+1:sys.n_constr*sys.N_hor+sys.n_sys) = be_l - ...
    x0_resp(sys.N_hor*sys.n_sys+1:sys.n_sys*(sys.N_hor+1));
% no terminal constraints on longitudinal acceleration
ub(sys.n_constr*sys.N_hor+sys.n_sys+1) = inf;
lb(sys.n_constr*sys.N_hor+sys.n_sys+1) = -inf;
% lateral acceleration is constrained to be close to target 
ub(sys.n_constr*sys.N_hor+sys.n_sys+2) = 0;
lb(sys.n_constr*sys.N_hor+sys.n_sys+2) = 0;

%% --------------------- setup slack constraints ------------------------------------- %%
% limit slacks to be strictly positive 
lb(sys.N_hor*sys.n_constr+sys.n_sys+sys.m_sys+1:end) = 0;
% upper limit of tire slacks set to certain percentage of maximum available accelerations
ub(sys.N_hor*sys.n_constr+sys.n_sys+sys.m_sys+1:sys.n_slacks:end) = sys.slack_lim_rel; 
ub(sys.N_hor*sys.n_constr+sys.n_sys+sys.m_sys+2:sys.n_slacks:end) = sys.slack_lim_rel; 
% upper limit of lateral error slacks set to equal amount of allowed deviation
% which allows to go for the double amount if required
ub(sys.N_hor*sys.n_constr+sys.n_sys+sys.m_sys+3:sys.n_slacks:end) = 0.5*(d_lim_ub_m(1)-d_lim_lb_m(1)); 
ub(sys.N_hor*sys.n_constr+sys.n_sys+sys.m_sys+4:sys.n_slacks:end) = 0.5*(d_lim_ub_m(1)-d_lim_lb_m(1)); 

%% ----------- transform A_ineq and P to CSC compatible format using sparsity pattern ----------- %%
A_x = A_ineq(sys.A_i_lin);

end

