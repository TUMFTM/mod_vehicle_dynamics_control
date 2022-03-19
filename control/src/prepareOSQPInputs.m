function [q, l, u, P_x, A_x, A_n] = prepareOSQPInputs(P_upd, f, A_ineq, lb, ub, iA0, P_nnz, n, m, A_nnz, P_i_lin, A_i_lin)
%PREPAREOSQPINPUTS Summary of this function goes here
%   Detailed explanation goes here
% Define the dimensions of the output port using the sys-structure to avoid
% error messages due to unknown or mismatching dimensions.
% n_osqp is used to define matrix dimensions in "prepareOSQPInputs", even
% if sys is defined as a nontunable parameter, errors occur. Using the
% detour through the variable "n_osqp" solves this problem. Therefore
% include following 4 codelines before calling the "prepareOSQPInputs"
% function via the user-defined matlab function:
% n_osqp = sys.osqp_n;    
% m_osqp = sys.osqp_m;  
% P_nnz_osqp = sys.P_nnz;
% A_nnz_osqp = sys.A_nnz;

l = -inf*ones(m,1); % initialize the lower bounds with -inf, i.e. initially no lower bound active
u = inf*ones(m,1); % initialize the upper bounds with +inf, i.e. initially no upper bound active
q = zeros(n,1); % lin. cost function vector
q = f;  % no need for processing the lin. cost function vector
P_x = zeros(1,P_nnz);   % to enable "variable sized vector" define P_nnz as a non-tunable parameter (via 'Edit Data')
P_x(1, 1:P_nnz) = P_upd(P_i_lin);
A_x = zeros(A_nnz, 1);
A_idx_help = A_i_lin';  % A_i_lin is initially a row-vector
A_n = size(A_idx_help,1);
A_x(1:A_n, 1) = A_ineq(A_idx_help);
l = lb;
u = ub;
end

