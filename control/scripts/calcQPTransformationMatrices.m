function [ABK_MPC, Ax0_MPC, H, f, H_states, H_inputs, H_reg, H_slacks, f_Dax, f_Day, f_x0, ...
    f_D_deltaax, f_D_deltaay, f_d_m, f_dot_d_mps] ...
    = calcQPTransformationMatrices(A_d, B_d, Q, R, S_LQR, N, nx, nu, ns_u, ns_x, S_lin_slack, ...
    S_quad_slack, rdiff_ax, rdiff_ay, r_ax)

% Authors:      Martin Euler
%               Salih G�m�s
%               Alexander Wischnewski
% Description:  
%   function used to calc the QP transformation matrices
%
% Inputs/parameters:
%   A_d: discrete system dynamic matrix
%   B_d: discrete system input matrix
%   Q: MPC state weight matrix
%   R: MPC input weight matrix
%   S_LQR: solution of riccati equation
%   N: length of the control horizon
%   nx: number of states
%   nu: number of inputs
%   ns_u: number of slack variables for input constraints in each calculation step
%   ns_x: number of slack variables for state constraints in each calculation step
%   S_lin_slack: weight column vector for linear slack penalty (Dimension: (ns_u+ns_x)-by-1)
%   S_quad_slack: weight matrix for quadratic slack penalty (Dimension: (ns_u+ns_x)-by-(ns_u+ns_x))
%   rdiff_ax: weight for ax difference penalties
%   rdiff_ay: weight for ay difference penalties

% Outputs:
%   ABK_MPC: state transformation matrix from optimization variables 
%   Ax0_MPC: state transformation matrix from initial state
%   H: hessian matrix for QP Dimension: ((nu+ns_u+ns_x)*N+nx)-by-(nu+ns_u+ns_x)*N+nx))
%   f: multiplier of linear term in QP (column vector, Dimension: ((nu+ns_u+ns_x)*N+nx)-by-1)
%   H_states: quadratic cost for states
%   H_inputs: quadratic cost for inputs
%   H_reg: quadratic cost for regularization terms
%   H_slacks: quadratic cost for slacks
%   f_Dax: linear cost for online calculation of weighted differences for ax
%   f_Day: linear cost for online calculation of weighted differences for ay
%   f_x0: linear cost for online calculation of initial state response
%   f_D_deltaax: linear cost for online calculation of last step difference of ax
%   f_D_deltaay: linear cost for online calculation of last step difference of ay
%   f_d_m: linear cost for tracking lateral target trajectory - lateral error
%   f_dot_d_mps: linear cost for tracking lateral target trajectory - lateral error derivative

%% construct matrices mapping to states
% matrix mapping such that
% [x0, ..., xN]^T = ABK_MPC*opt + Ax0_MPC*x0
% with optimization variables o = [control inputs, slacks] 
% to state sequence x0, ..., xN
ABK_MPC = zeros(nx*(N+1), nu*(N+1)+(ns_u+ns_x)*N);
% construct the toeplitz matrix 
for i = 1:N
   % iterate via predicted states
   for j = 0:(i-1)
       % iterate via external input signal  
       ABK_MPC(nx*i+1:nx*(i+1), 1+nu*j:nu*(j+1)) = A_d^(i-j-1)*B_d;             
   end   
end
% matrix mapping from initial state to state sequence x0, ..., xN
Ax0_MPC = zeros(nx*(N+1), nx); 
for i = 0:N
    Ax0_MPC(nx*i+1:nx*(i+1), :) = A_d^i; 
end

%% construct cost matrices
% matrix with weight matrix Q on diagonal entries, zero else. 
Q_MPC = zeros(nx*(N+1), nx*(N+1)); 
% matrix with weight matrix R on diagonal entries, zero else. 
R_MPC_noslack = zeros(nu*(N+1), nu*(N+1));
R_MPC_slack = zeros((ns_u+ns_x)*N, (ns_u+ns_x)*N);
% column vectors for the augmentation to f. 
f_slack = zeros((ns_u+ns_x)*N, 1);% part of f corresponding to the linear slack penalty
% initialize and construct differentiation matrices and scale with weight
D_mat_full = zeros(N+1, N+2); 
D_select_ax = zeros(N+1, nu*(N+1)); 
D_select_ay = zeros(N+1, nu*(N+1)); 
for i = 1:(N+1)
    D_mat_full(i, i:i+1) = [1, -1]; 
end
% calculate quadratic matrix of differentiatiors
D_mat_quad = D_mat_full'*D_mat_full; 
for i = 1:(N+1)
    D_select_ax(i, 2*i-1) = 1; 
    D_select_ay(i, 2*i) = 1; 
end
% precompute terms used online for f
f_Dax = [rdiff_ax*D_mat_quad(1, 2:end)*D_select_ax, zeros(1, (ns_u+ns_x)*N);...
    rdiff_ax*D_mat_quad(2:end, 2:end)*D_select_ax + r_ax * D_select_ax, zeros(N+1, (ns_u+ns_x)*N)]; 
f_Day = [rdiff_ay*D_mat_quad(:, 2:end)*D_select_ay, zeros(N+2, (ns_u+ns_x)*N)]; 
f_D_deltaax = [rdiff_ax*D_mat_quad(1, 2:end)*D_select_ax, zeros(1, (ns_u+ns_x)*N)];
f_D_deltaay = [rdiff_ay*D_mat_quad(1, 2:end)*D_select_ay, zeros(1, (ns_u+ns_x)*N)];
% compute quadratic terms
PReg_ax = rdiff_ax*D_select_ax'*D_mat_quad(2:end, 2:end)*D_select_ax + D_select_ax' * r_ax * D_select_ax;
PReg_ay = rdiff_ay*D_select_ay'*D_mat_quad(2:end, 2:end)*D_select_ay; 

% build state cost matrix
for i = 0:(N-1)  
   Q_MPC(1+nx*i:nx+nx*i,1+nx*i:nx+nx*i) = Q;
end
% add terminal cost 
Q_MPC(nx*N+1:nx*(N+1), nx*N+1:nx*(N+1)) = S_LQR; 

% precompute terms used online for f (initial state response
f_x0 = Q_MPC*ABK_MPC; 

% build cost terms for lateral target tracking 
d_m_select = diag(repmat([0, 1, 0], 1, N+1)); 
f_d_m = d_m_select(2:nx:end, :)*d_m_select'*Q_MPC*d_m_select*ABK_MPC; 
dot_d_mps_select = diag(repmat([0, 0, 1], 1, N+1)); 
f_dot_d_mps = dot_d_mps_select(3:nx:end, :)*dot_d_mps_select'*Q_MPC*dot_d_mps_select*ABK_MPC; 

 % build input cost matrix 
for i = 0:(N-1)
    R_MPC_noslack(1+nu*i:nu+nu*i,1+nu*i:nu+nu*i) = R;
end

% build slack cost matrices
for i = 0:(N-1)
    % quadratic cost
    R_MPC_slack(1+(ns_u+ns_x)*i:(ns_u+ns_x)*(i+1), 1+(ns_u+ns_x)*i:(ns_u+ns_x)*(i+1)) = S_quad_slack;
    % linear cost 
    f_slack(1+(ns_u+ns_x)*i:(ns_u+ns_x)*(i+1), 1) = S_lin_slack;
end

% construct quadratic cost matrices and sum them up to form the hessian of the optimization problem
% multiplication with 2 is to compensate for 1/2 of OSQP formulation
H_states = 2*ABK_MPC' * Q_MPC * ABK_MPC; 
H_inputs = 2*[R_MPC_noslack, zeros(nu*(N+1), (ns_u+ns_x)*N); ...
    zeros((ns_u+ns_x)*N, nu*(N+1)), zeros((ns_u+ns_x)*N, (ns_u+ns_x)*N)]; 
H_reg = 2*[PReg_ax + PReg_ay, zeros(nu*(N+1), (ns_u+ns_x)*N); ...
    zeros((ns_u+ns_x)*N, nu*(N+1)), zeros((ns_u+ns_x)*N, (ns_u+ns_x)*N)]; 
H_slacks = 2*[zeros(nu*(N+1), nu*(N+1)), zeros(nu*(N+1), (ns_u+ns_x)*N); ...
    zeros((ns_u+ns_x)*N, nu*(N+1)), R_MPC_slack];
H = H_states + H_inputs + H_reg + H_slacks; 

% construct linear cost matrix to form the gradient of the optimization problem 
f = [zeros(nu*(N+1), 1); f_slack];

end

