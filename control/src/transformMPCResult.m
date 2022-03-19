function [u_opt_total, x_pred, y_pred, x_pred_left, y_pred_left, x_pred_right, y_pred_right, ...
    vx_pred, d_pred, dot_d_pred, s_dot_pred, ...
    ax_pred, ax_tire_pred, ay_pred, TireUtilizationTarget, cost_values]  = ...
    transformMPCResult(status, u_opt_total, errorState, x_traj, y_traj, ...
    psi_traj, v_traj, ax_diff_traj, d_Target_m, ax_lim_mps2, ay_lim_mps2, ...
    vx_lin_mps, kappa_lin_radpm, UncertaintyTube, ...
    sys, drag_coefficient, roh_air, vehiclemass_kg, P_VDC_PositiveAxLimScale)
% function transformMPCResult
% Author:       Martin Euler
%               Alexander Wischnewski
% Description:  
%   function used to process the output of the mpc solver
% Inputs/parameters:
%   status:             Solver status / > 0 -> ok / 0 -> not ok
%   u_opt_total:        optimal solution
%   error_state:        vehicle state in error coordinates
%   UncertaintyTube:    Matrix with all uncertainty matrices for the prediction horizon
%   x_traj:             x-East of target trajectory
%   y_traj:             y-North of target trajectory
%   psi_traj:           Heading of target trajectory
%   v_traj:             Velocity of target trajectory
%   ax_diff_traj:       Velocity derivative with respect to path coordinate of target trajectory 
%   d_Target_m:         Target trajectory projected into curvilinear coordinate system
%   ax_lim_mps2:        Limit for longitudinal acceleration
%   ay_lim_mps2:        Limit for lateral acceleration
%   ax_dist_mps2:       Disturbance in longitudinal acceleration
%   ay_dist_mps2:       Disturbance in lateral acceleration
%   vx_lin_mps:         Linearization velocity
%   kappa_lin_radpm:    Linearization curvature
%   sys:                struct with necessary parameters and matrices
%   drag_coefficient:   Coefficient for air drag according to F = 1/2*roh_air*drag_coefficient
%   roh_air:            Air density
%   vehiclemass_kg:     Overall vehicle mass
%   P_VDC_PositiveAxLimScale Scaling factor for positive accelerations to consider RWD

% Outputs:
%   u_opt_total:        optimal solution
%   x_pred:             Predicted x-East positions (mean of uncertainty tube)
%   y_pred:             Predicted y-North positions (mean of uncertainty tube)
%   x_pred_left:        Predicted x-East positions (left side of uncertainty tube)
%   y_pred_left:        Predicted y-North positions (left side of uncertainty tube)
%   x_pred_right:       Predicted x-East positions (right side of uncertainty tube)
%   y_pred_right:       Predicted y-North positions (right side of uncertainty tube)
%   vx_pred:            Predicted velocity
%   d_pred:             Predicted lateral deviation
%   dot_d_pred:         Predicted lateral deviation derivative
%   s_dot_pred:         Predicted path progress
%   ax_pred:            Predicted longitudinal acceleration
%   ax_tire_pred:       Predicted longitudinal acceleration "at tire" (with driving resistances)
%   ay_pred:            Predicted lateral acceleration 
%   TireUtilization:    Tire utilization (target)
%   cost_values:        debug information on separate cost terms

%% ------- define optimization problem variables ----------------------- %%
nx = 3; % number of state variables should equal sys.n_sys

% initialize output variables 
cost_values = zeros(4, 1); 

% check if solver solution is valid
if status > 0
    %% -------------------------- obtain predictions for nominal system ---------------------------- %%
    % calculate errorState prediction and improve computational efficiency by exploiting
    % knowledge that slacks do not contribute to this
    error_pred = sys.ABK_MPC(:, 1:sys.m_sys*(sys.N_hor+1))*u_opt_total(1:sys.m_sys*(sys.N_hor+1)) ...
        + sys.Ax0_MPC*errorState;
    % update velocity predictions and limit to ensure robust transformations
    vx_pred = v_traj - error_pred(1:nx:end); 
    % update lateral error predictions
    d_pred = error_pred(2:nx:end); 
    % calculate lateral uncertainties
    tight_d_m = sqrt(UncertaintyTube(2, 2:nx:end))';    
    % update lateral error derivative predictions
    dot_d_pred = error_pred(3:nx:end); 
    % update predictions for path coordinate 
    d_psi_pred = asin(max(min(dot_d_pred./vx_pred, 0.999), -0.999)); 
    s_dot_pred = vx_pred.*cos(d_psi_pred)./(1-d_pred.*kappa_lin_radpm); 
    % update position predictions (compensate for linearization error with respect to target path)
    x_pred = x_traj - sin(psi_traj+pi/2).*(d_pred-d_Target_m);
    y_pred = y_traj + cos(psi_traj+pi/2).*(d_pred-d_Target_m);         
    % update uncertainty tube predictions 
    % +1 for vehicle width
    x_pred_left = x_pred - sin(psi_traj+pi/2).*(tight_d_m+1);
    y_pred_left = y_pred + cos(psi_traj+pi/2).*(tight_d_m+1);     
    x_pred_right = x_pred + sin(psi_traj+pi/2).*(tight_d_m+1);
    y_pred_right = y_pred - cos(psi_traj+pi/2).*(tight_d_m+1);        
    % update longitudinal acceleration predictions  
    ax_pred = ax_diff_traj.*(kappa_lin_radpm.*vx_lin_mps.*d_pred + ...
        2*vx_lin_mps-vx_pred) + ...
        roh_air*drag_coefficient.*vx_lin_mps.*error_pred(1:nx:end)./vehiclemass_kg +...
        - u_opt_total(1:2:2*(sys.N_hor+1)); 
    % include driving resistances
    ax_tire_pred = ax_pred + 0.5.*roh_air.*drag_coefficient.*vx_lin_mps.^2./vehiclemass_kg; 
    % update lateral acceleration predictions
    ay_pred = kappa_lin_radpm.^2.*vx_lin_mps.^2.*d_pred - ax_diff_traj.*dot_d_pred ...
        - 2*kappa_lin_radpm.*vx_lin_mps.*error_pred(1:nx:end) ...
        + 2*kappa_lin_radpm.*vx_lin_mps.*(v_traj - vx_lin_mps) ...
        + kappa_lin_radpm.*vx_lin_mps.^2 + u_opt_total(2:2:2*(sys.N_hor+1));
    
    % tire utilization target
    if(ax_tire_pred(1) > 0)
        % consider RWD factor for positive accelerations
        TireUtilizationTarget = abs(ax_tire_pred(1)/(P_VDC_PositiveAxLimScale*ax_lim_mps2(1))) ...
            + abs(ay_pred(1)/ay_lim_mps2(1)); 
    else
        TireUtilizationTarget = abs(ax_tire_pred(1)/(ax_lim_mps2(1))) ...
            + abs(ay_pred(1)/ay_lim_mps2(1)); 
    end        
    
    %% recalculate costs 
    % calculations removed due to computation time restrictions
    cost_values(1) = 0; 
    cost_values(2) = 0;
    cost_values(3) = 0;
    cost_values(4) = 0;     
    
else
    % initialize all predictions 
    s_dot_pred = 0.01*ones(41, 1); 
    x_pred = zeros(41, 1); 
    y_pred = zeros(41, 1); 
    x_pred_left = zeros(41, 1); 
    y_pred_left = zeros(41, 1); 
    x_pred_right = zeros(41, 1); 
    y_pred_right = zeros(41, 1); 
    vx_pred = zeros(41, 1);
    d_pred = zeros(41, 1);
    dot_d_pred = zeros(41, 1);
    ax_pred = zeros(41, 1); 
    ax_tire_pred = zeros(41, 1); 
    ay_pred = zeros(41, 1);
    TireUtilizationTarget = 0; 
end

