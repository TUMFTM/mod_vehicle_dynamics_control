function [flag_setup, flag_solve, flag_q_Upd, flag_P_Upd, flag_A_Upd,...
    flag_bound_Upd, solver_sfunT, solver_runT, solver_updateT, solver_solveT, solver_iter, v_traj,...
    psi_traj, y_traj, x_traj, solver_state, u_opt_total, s_current, errorState, OptimizedTrajectory] = calcDummyTrajectory(LinearizationTrajectory,N)
% function calcDummyTrajectory
% Author:       Martin Euler         last update: 18-03-2020
% Description:  
%   function used to create a dummy trajectory
% Inputs/parameters:
%   LinearizationTrajectory: current raceline section

% Outputs:
%   flag_setup: dummy solver exitflag of setup of solver-settings/data etc.
%   flag_solve: dummy solver exitflag of solving QP
%   flag_q_Upd: dummy solver exitflag of q update
%   flag_P_Upd: dummy solver exitflag of P update
%   flag_A_Upd: dummy solver exitflag of A update
%   flag_bound_Upd: dummy solver exitflag of bound update (l, u)
%   solver_sfunT: dummy solver s-Fuction runtime in s
%   solver_runT: dummy solver runtime in s
%   solver_updateT: dummy solver update time in s
%   solver_solveT: dummy solver solve time in s
%   solver_iter: dummy solver number of iterations
%   psi_traj: trajectory heading
%   y_traj: y-coordinates of trajectory
%   x_traj: x-coordinates of trajectory
%   solver_state: dummy solver status
%   u_opt_total: dummy QP solution
%   s_current: current s-coordinate dummy
%   errorState: errorState dummy
%   OptimizedTrajectory: new vehicle trajectory
%% ------- define optimization problem variables ----------------------- %%
N = 50; % number of prediction steps should equal sys.contr_horizon
nu = 2;
nx = 3;
%% ---------------- generate dummy outputs ----------------------------- %% 
flag_setup = -1;
flag_solve = -1;
flag_q_Upd = -1;
flag_P_Upd = -1;
flag_A_Upd = -1;
flag_bound_Upd = -1;
solver_sfunT = 0;
solver_runT = 0;
solver_updateT = 0;
solver_solveT = 0;
solver_iter = 0;
v_traj = zeros(N+1,1);
psi_traj = zeros(N+1,1);
y_traj = zeros(N+1,1);
x_traj = zeros(N+1,1);
solver_state = 1;
errorState = [0;0;0];
u_opt_total = zeros(N*nu+nx,1);
s_current = 0;
s_pred = zeros(N,1);
s_pred(1) = s_current;
s_pred(2:end) = s_current+1e-6:1e-6:s_current+1e-6*(N-1);
[OptimizedTrajectory] = fillTrajectoryStruct(LinearizationTrajectory.LapCnt, LinearizationTrajectory.TrajCnt, ...
                             s_pred, zeros(N,1), zeros(N,1), zeros(N,1), zeros(N,1), zeros(N,1),...
                             zeros(N,1));
end