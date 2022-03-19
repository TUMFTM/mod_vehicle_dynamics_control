function [OptimizedTrajectory] = fillTrajectoryStruct(LapCnt, TrajCnt, ...
    s_loc_pred, x_pred, y_pred, psi_pred, kappa_pred, vx_pred, ax_pred)
% function fillTrajectoryStruct
% Author:       Martin Euler         last update: 05-03-2020
% Description:  
%   helper function used to initialize the trajectory struct
% Inputs/parameters:
%   s_loc_pred: vector for predicted raceline distance in path coordinates
%   x_pred: vector for predicted x-coordinates
%   y_pred: vector for predicted y-coordinates
%   psi_pred: vector for predicted psi
%   kappa_pred: vector for predicted kappa
%   vx_pred: vector for predicted velocity
%   ax_pred: vector for predicted acceleration
% Outputs:
%   OptimizedTrajectory: struct of new trajectory

%% ------------------------- fill struct ------------------------------- %%
OptimizedTrajectory.LapCnt = LapCnt;
OptimizedTrajectory.TrajCnt = TrajCnt;
OptimizedTrajectory.s_loc_m = s_loc_pred;
OptimizedTrajectory.s_glob_m = s_loc_pred;
OptimizedTrajectory.x_m = x_pred;
OptimizedTrajectory.y_m = y_pred;
OptimizedTrajectory.psi_rad = psi_pred;
OptimizedTrajectory.kappa_radpm = kappa_pred;
OptimizedTrajectory.v_mps = vx_pred;
OptimizedTrajectory.ax_mps2 = ax_pred;
end

