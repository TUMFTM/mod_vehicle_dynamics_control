function [x_low, y_low, x_up, y_up, x_low_max, y_low_max, x_up_max, y_up_max] = genTubeData(errorState, x_traj, y_traj, psi_traj, u_opt_total, status, M_s, N,P_VDC_DyLim, P_VDC_VariableTubeSize, sys)
% function genTubeData
% Author:       Martin Euler         last update: 15-04-2020
% Description:  
%   function used to generate the tube data for the unity sim
% Inputs/parameters:
%   x_traj: x-Coordinates of target trajectory
%   y_traj: x-Coordinates of target trajectory
%   psi_traj: Heading of target trajectory
%   u_opt_total: Optimized inputs for dynamic system
%   status: Solver status / > 0 -> optimal solution / <= 0 suboptimal or
%   infeasable solution
%   sys: struct with necessary parameters and matrices

% Outputs:
%   x_low: x-coordinates of lower tube bound
%   y_low: y-coordinates of lower tube bound
%   x_up: x-coordinates of upper tube bound
%   x_up: x-coordinates of upper tube bound
%   x_low_max: x-coordinates of lower dy bound
%   y_low_max: y-coordinates of lower dy bound
%   x_up_max: x-coordinates of upper dy bound
%   x_up_max: x-coordinates of upper dy bound

%% ------- define optimization problem variables ----------------------- %%
nx = 3; % number of state variables should equal sys.n_sys
nu = 2; % number of input variables should equal sys.m_sys
% check if solver solution is valid
if status > 0   
%% -------------------------- init matrices ---------------------------- %%
    if ~P_VDC_VariableTubeSize  
        % TODO: Do we actual need this equation? This should still be done by the underlying
        % controller? 
        u_opt_total(2:2:6,1) = u_opt_total(2:2:6,1) + sys.K_LQR(2,:)*(errorState - u_opt_total((nu*N+nx)-2:(nu*N+nx)));
    end    
    % calculate errorState prediction    
    error_pred = sys.ABK_MPC * u_opt_total;
    
%% ------------------ generate trajectory for the vehicle ----------------- %%  
    % define vehicle width starting from the COG
    vehWidthCOG = 1; 
    % calc tube position predictions
    [lb_Tube, ub_Tube] = calcBounds(error_pred(2:nx:3*N), M_s, N, 2, vehWidthCOG);
    lb_lim = -(P_VDC_DyLim+vehWidthCOG)*ones(N,1);
    ub_lim = (P_VDC_DyLim+vehWidthCOG)*ones(N,1);
    x_low = x_traj(1:N) - sin(psi_traj(1:N) + pi/2) .* lb_Tube;
    y_low = y_traj(1:N) + cos(psi_traj(1:N) + pi/2) .* lb_Tube;
    x_up = x_traj(1:N) - sin(psi_traj(1:N) + pi/2) .* ub_Tube;
    y_up = y_traj(1:N) + cos(psi_traj(1:N) + pi/2) .* ub_Tube;
    % get dy_lim position predictions
    x_low_max = x_traj(1:N) - sin(psi_traj(1:N) + pi/2) .* lb_lim;
    y_low_max = y_traj(1:N) + cos(psi_traj(1:N) + pi/2) .* lb_lim;
    x_up_max = x_traj(1:N) - sin(psi_traj(1:N) + pi/2) .* ub_lim;
    y_up_max = y_traj(1:N) + cos(psi_traj(1:N) + pi/2) .* ub_lim;
    
else
    % gen dummy arrays
    x_low = zeros(N,1);
    y_low = zeros(N,1);
    x_up = zeros(N,1);
    y_up = zeros(N,1);
    x_low_max = zeros(N,1);
    y_low_max = zeros(N,1);
    x_up_max = zeros(N,1);
    y_up_max = zeros(N,1);
   
end
end
function [lb, ub] = calcBounds(data, tubeShapeMatrix, N, pos,vehWidthCOG)
    lb = zeros(N,1);
    ub = zeros(N,1);
    for iter = 1:1:N
        % get tube bounds
        shapeMatrix = (tubeShapeMatrix(:,(iter-1)*3+1:(iter-1)*3+3));
        % use 1.2*size to cope with 'tighter' tube because of line width in
        % unity plot
        lb(iter,1) = data(iter,1)-vehWidthCOG-sqrt(shapeMatrix(pos,pos));
        ub(iter,1) = data(iter,1)+vehWidthCOG+sqrt(shapeMatrix(pos,pos));
    end
end


