function [sys] = setupTMPCStruct(verbose)
%
% Authors:       Martin Euler
%                Salih Guemues
%                Alexander Wischnewski
%
% Description:  
%   function used to initialize matrices and parameters for the
%   TMPC-Controller. Run this script and copy results to parameters/control/xx_mvdc_mpc.sldd.
% 
% Inputs: 
%   verbose:    Set to true to enable debug outputs
% Outputs:
%   sys:        Struct with all control parameters

%% -------------------- General ------------------------------------------- %% 
sys.dummy = 'ForceInline';      % add dummy  to force parameter inline during code generation  

%% -------------------- TMPC tuning parameters ---------------------------- %%
sys.Q = diag([0.02, 0.1, 40]);  % state weight matrix - tuned such that lateral tracking is nearly unconsidered
sys.R = diag([0.01, 0.005]);    % input weight matrix
sys.roh_u = 1;                  % weight for input slacks (allows to prioritize violation of input or state constraints)
sys.roh_x = 1;                  % weight for state slacks (allows to prioritize violation of input or state constraints)
sys.roh_lin = 1000;             % weight for linear slack penalty --> sufficiently high for exact penalty
sys.roh_quad = 100;             % weight for quadratic slack penalty
sys.rdiff_ax = 0.1;             % weight for ax differences
sys.rdiff_ay = 200;             % weight for ay differences
sys.r_ax = 0.00;                % weight for absolute ax
sys.slack_lim_rel = 5/100;      % define desired upper relative slack limits, e.g. 5% as here

%% -------------------- OSQP solver parameters ---------------------------- %% 
sys.trigger_print = 0;          % trigger for debugging: 1 = ssPrintf executed; 0 = ssPrintf commented out
sys.trigger_Pupdate = 0;        % trigger for P-update: 1 = update P; 0 = P is constant, update unnecessary
sys.use_scale_units = 0;        % boolean for performing scaling of slack units: 1: active, 0: inactive
sys.use_constr_precision = 0;   % boolean for performing increase of constraints precision: 1: active, 0: inactive

%% -------------------- Point mass model ---------------------------------- %%
sys.N_hor = 40;                 % optimization horizon 
sys.Ts = 0.06;                  % sample time of TMPC
A = [0, 0, 0; 0, 0, 1; 0, 0, 0];% dynamic matrix of continous time representation of error diff. equat. 
B = [1, 0; 0, 0; 0, 1];         % input matrix of -"- 
sys.A_d = eye(3) + A*sys.Ts;    % dynamic matrix of discrete time representation of error diff. equat.
sys.B_d = B * sys.Ts;           % input matrix of discrete time representation of error diff. equat.
sys.n_sys = size(A, 1);         % system dimension = number of state variables, here =3
sys.m_sys = size(B, 2);         % input dimension = number of inputs, here = 2
% calculate LQR and corresponding terminal weight matrix from Riccati equation
[K_LQR, S_LQR, ~] = dlqr(sys.A_d, sys.B_d, sys.Q, sys.R);
sys.S_LQR = S_LQR; 
sys.K_LQR = -K_LQR;
% initial shape matrix for ellipsoids
sys.M_1 = diag([0.0001^2, 0.0001^2, 0.0001^2]);

if verbose
    disp('The LQR controller for the terminal set has been designed to be: '); 
    disp(sys.K_LQR); 
    disp('The terminal set weights have been calculated to be: '); 
    disp(sys.S_LQR); 
end

%% -------------- soft constraint preparation via slack introduction ----------------- %%
nc_u = 4;       % number of input constraints in each calculation step
nc_x = 1;       % number of state constraints in each calculation step
ns_u = 2;       % number of slack variables for input constraints in each calculation step
ns_x = 2;       % number of slack variables for state constraints in each calculation step
  
% number of constraints per calculation step
sys.n_constr = nc_u + nc_x;        
% number of slacks per calculation step
sys.n_slacks = ns_u + ns_x; 
% calculate total number of slack variables
sys.ns_total = (ns_u + ns_x)*sys.N_hor;   
% weight column vector for linear slack penalty (Dimension: (ns_u+ns_x)-by-1)
sys.S_lin_slack = sys.roh_lin * [sys.roh_u*ones(ns_u, 1); sys.roh_x*ones(ns_x, 1);];
% weight matrix for quadratic slack penalty --> allows for constraint violation tuning (Dimension: (ns_u+ns_x)-by-(ns_u+ns_x))
sys.S_quad_slack = sys.roh_quad * [sys.roh_u*eye(ns_u), zeros(ns_u, ns_x); ...
                                    zeros(ns_x, ns_u), sys.roh_x*eye(ns_x)];
        
%% ------------------ Calculate matrices for MPC controller ---------------------- %%
% optimization variables are ordered as follows: 
% delta_ax0             - corrective long. acceleration for stage 0
% delta_ay0             - corrective lat. acceleration for stage 0
% delta_ax1
% delta_ay1
% ...
% delta_ax_terminal     - corrective long. acceleration for stage N+1 (terminal stage)
% delta_ay_terminal     - corrective lat. acceleration for stage N+1 (terminal stage)
% slacks                - box constraints for slack variables

[sys.ABK_MPC, sys.Ax0_MPC, H, f, sys.H_states, sys.H_inputs, sys.H_reg, sys.H_slacks, ...
    sys.f_Dax, sys.f_Day, sys.f_x0, sys.f_D_deltaax, sys.f_D_deltaay, sys.f_d_m, sys.f_dot_d_mps] = ...
    calcQPTransformationMatrices(sys.A_d, sys.B_d, sys.Q, sys.R, sys.S_LQR, sys.N_hor, sys.n_sys, ...
    sys.m_sys, ns_u, ns_x, sys.S_lin_slack, sys.S_quad_slack, sys.rdiff_ax, sys.rdiff_ay, sys.r_ax);

%% ------------------ Buildup constraint matrices -------------------------------- %%
% constraints are ordered as follows: 
%
% tire1_au                  - first tire constraint for stage 1 (upper)
% tire1_al                  - first tire constraint for stage 1 (lower)
% tire1_bl                  - second tire constraint for stage 1 (lower)
% tire1_bu                  - second tire constraint for stage 1 (upper)
% d_m1                      - lateral error constraint for stage 1
% ...                       - similar up to stage 50
% d_m50                     - lateral error terminal state constraint (stage 51)
% dot_d_m50                 - lateral error derivative terminal state constraint (stage 51)
% delta_vx_mps50            - velocity error terminal state constraint (stage 51)
% up_slack_tire1_a          - upper slack on tire1_a constraint
% low_slack_tire1_a         - lower slack on tire1_a constraint
% up_slack_tire1_b          - upper slack on tire1_b constraint
% low_slack_tire1_b         - lower slack on tire1_b constraint
% up_slack_dm_1             - upper slack on d_m1 constraint
% low_slack_dm_1            - lower slack on d_m1 constraint
% ...                       - similar up to stage 50

% total number of constraints
sys.n_constr_total = sys.n_constr*sys.N_hor + sys.n_sys + sys.m_sys + sys.ns_total; 
% total number of optimization variables 
sys.n_opt_total = sys.m_sys*(sys.N_hor+1) + sys.ns_total; 
% generate equality constraint matrix
A_ineq = zeros(sys.n_constr_total, sys.n_opt_total); 

% prepare sparsity pattern of constraint matrix for tire constraints
% true matrix values will be set during online update method in prepareOptimizationProblem.m
for i=1:1:sys.N_hor
    % - depend on all input variables up to this stage:
    A_ineq(1+(i-1)*sys.n_constr, 1:i*sys.m_sys) = ones(1, i*sys.m_sys); 
    A_ineq(2+(i-1)*sys.n_constr, 1:i*sys.m_sys) = ones(1, i*sys.m_sys); 
    A_ineq(3+(i-1)*sys.n_constr, 1:i*sys.m_sys) = ones(1, i*sys.m_sys); 
    A_ineq(4+(i-1)*sys.n_constr, 1:i*sys.m_sys) = ones(1, i*sys.m_sys); 
    % - and add the slack variables to the braking constraints
    A_ineq(2+(i-1)*sys.n_constr, ...
        (sys.N_hor+1)*sys.m_sys+1+(i-1)*sys.n_slacks:(sys.N_hor+1)*sys.m_sys+i*sys.n_slacks) = ...
        [-1, 0, 0, 0;]; 
    A_ineq(3+(i-1)*sys.n_constr, ...
        (sys.N_hor+1)*sys.m_sys+1+(i-1)*sys.n_slacks:(sys.N_hor+1)*sys.m_sys+i*sys.n_slacks) = ...
        [0, -1, 0, 0;]; 
end

% lateral error constraints 
for i = 1:1:sys.N_hor
    % - depends on all input variables up to this stage
    A_ineq(5+(i-1)*sys.n_constr, :) = sys.ABK_MPC(2+(i-1)*sys.n_sys, :); 
    % - and the slack variables 
    A_ineq(5+(i-1)*sys.n_constr, ...
        (sys.N_hor+1)*sys.m_sys+1+(i-1)*sys.n_slacks:(sys.N_hor+1)*sys.m_sys+i*sys.n_slacks) = ...
        [0, 0, 1, -1;]; 
end

% prepare constraint matrices for terminal constraints
A_ineq(sys.n_constr*sys.N_hor+1:sys.n_constr*sys.N_hor+sys.n_sys, :) = ...
    sys.ABK_MPC(end-sys.n_sys+1:end, :);
% prepare constraint matrices for terminal input constraints 
A_ineq(sys.n_constr*sys.N_hor+sys.n_sys+1:sys.n_constr*sys.N_hor+sys.n_sys+sys.m_sys, ...
    sys.N_hor*sys.m_sys+1:(sys.N_hor+1)*sys.m_sys) = eye(sys.m_sys);

% slack constraints 
A_ineq(1+sys.n_constr*sys.N_hor+sys.n_sys+sys.m_sys:end, 1+sys.m_sys*(sys.N_hor+1):end) = ...
    eye(sys.n_slacks*sys.N_hor); 



%% -------------------- prepare sparse matrices for online QP ------------------------- %%
% tolerenace used to determine sparsity pattern
tol = 1e-5; 
% initialize some stuff 
P_fullmatrix = 0;                   % trigger for representation of P as full upper triangular; 0 = exploit sparsity, 1 = full triangular
A_fullmatrix = 0;                   % trigger for representation of A as full matrix; 0 = exploit sparsity, 1 = full matrix
P_nnz = 0;                          % P_nnz = 1-D array width of P_x & number of elements in upper triangular of P with dimension nxn

sys.osqp_m = size(A_ineq, 1);   % number of rows/constraints (1st matrix dimension)
sys.osqp_n = size(H, 1);        % number of columns/optimization-variables (2nd matrix dimension)

l_par = -inf*ones(sys.osqp_m, 1);
u_par = inf*ones(sys.osqp_m, 1);

% store unscaled A_ineq (in terms of the condition number), since some
% A_ineq-entries are replaced during simulation. Entries corresponding to 
% the slack variables, which are scaled by L, are NOT affected. Thus, store A_ineq which is prescaled by L, but not by E & D.
sys.A_ineq = A_ineq;
sys.l_par = l_par;
sys.u_par = u_par;
sys.osqp_qpar = f;   % multiplier of linear term in QP
% transform matrices into csc-format for the sfunction of the QP-solver
P_par = H; % Hessian matrix for QP. attention!! here "H" is the Hessian BEFORE the cholesky decomp., i.e. NOT sys.H!

% initialize P using the CSC-format to exploit sparsity pattern
P_par_upper = triu(P_par); % extract upper triangular of Hessian
P_idx_help = (find((abs(P_par_upper) >= tol)))'; % returns linear indices of entries >= defined tolerance
P_nnz = size(P_idx_help,2);
sys.P_nnz = P_nnz;
P_x_par(1, 1:P_nnz) = P_par_upper(P_idx_help);
sys.P_i_lin(1 ,1:P_nnz) = P_idx_help; % linear indices (for P-Update)
[P_row, P_col, ~] = find((abs(P_par_upper) >= tol));    % returns column indices of entries >= tol
sys.P_i_par(1, 1:P_nnz) = (P_row - 1)'; % row indices (for P-INIT): -1 because indexing in C begins with 0
k3 = 0;
for i = 1:1:sys.osqp_n        % number of columns/variables = sys.osqp_n 
    k3 = k3 + sum(sum(P_col==(i)));
    sys.P_p_par(1,i+1) = k3;
end
    
P = zeros(sys.osqp_n, sys.osqp_n);
P(sys.P_i_lin) = P_x_par;  % fill values according to sparsity pattern
% make matrix symmetric as P is only upper triangular before
P = P + P' - diag(diag(P));

A_idx_help = (find((abs(sys.A_ineq) >= tol)))'; % returns linear indices of entries >= defined tolerance
A_nnz = size(A_idx_help,2);
sys.A_nnz = A_nnz;
sys.A_i_lin(1 ,1:A_nnz) = A_idx_help; % linear indices (for A-Update)
sys.A_x_par(1, 1:A_nnz) = sys.A_ineq(A_idx_help);
[A_row, A_col, ~] = find((abs(sys.A_ineq) >= tol));    % returns column indices of entries >= tol
sys.A_i_par(1, 1:A_nnz) = (A_row - 1)'; % row indices (for A-INIT): -1 because indexing in C begins with 0
k3 = 0;
for i = 1:1:sys.osqp_n        % number of columns/variables = sys.osqp_n 
    k3 = k3 + sum(sum(A_col==(i)));
    sys.A_p_par(1,i+1) = k3;
end

% since Hessian matrix is constant and not updated, osqp uses the
% information sys.P_x_par, sys.P_i_par, sys.P_p_par. 
sys.P_par = P;
sys.P_x_par = P(sys.P_i_lin);

