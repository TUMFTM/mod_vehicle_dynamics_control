function [ax_pred_mps2, ay_pred_mps2, ax_quantile_mps2, ay_quantile_mps2, ...
    alg_quantile_ax_upd, alg_quantile_ay_upd, alg_mean_ax_upd, alg_mean_ay_upd]...
    = learnDisturbanceRepresentation(LatDisturbanceEstimation, LongDisturbanceEstimation, ...
    ax_Target_mps2, dax_Target_mps3, ay_Target_mps2, v_Target_mps, ...
    alg_quantile_ax, alg_quantile_ay, alg_mean_ax, alg_mean_ay)
%% Documentation 
%
% Author: Alexander Wischnewski     Start Date: 13.12.2019
%
% Description:  
%   calculates a disturbance prediction model based upon the currently observed disturbances in
%   longitudinal and lateral control. It combines a recursive mean value regression with a recursive
%   quantile estimation algorithm to learn the model. The model is then used for predicting the
%   mean and spread bounds on the disturbances along the predicted feature trajectory. 
%
% Inputs:
%   LatDisturbanceEstimation        Data structure of lateral disturbance estimation algorithm
%   LongDisturbanceEstimation       Data structure of longitudinal disturbance estimation algorithm
%   ax_Target_mps2                  Predicted feature vector for ax_Target [30x1]
%   dax_Target_mps3                 Predicted feature vector for dax_Target [30x1]
%   ay_Target_mps2                  Predicted feature vector for ay_Target [30x1]
%   v_Target_mps2                   Predicted feature vector for v_Target [30x1]
%   alg_quantile_ax                 Data structure of longitudinal quantile estimation algorithm
%   alg_quantile_ay                 Data structure of lateral quantile estimation algorithm
%   alg_mean_ax                     Data structure of longitudinal mean value regression algorithm
%   alg_mean_ay                     Data structure of lateral mean value regression algorithm
% 
% Outputs: 
%   ax_pred_mps2                    Predicted longitudinal disturbance mean [30x1]
%   ay_pred_mps2                    Predicted longitudinal disturbance mean [30x1]
%   ax_quantile_mps2                Longitudinal disturbance prediction error quantile
%   ay_quantile_mps2                Lateral disturbance prediction error quantile
%   alg_quantile_ax                 Data structure of longitudinal quantile estimation algorithm
%   alg_quantile_ay                 Data structure of lateral quantile estimation algorithm
%   alg_mean_ax                     Data structure of longitudinal mean value regression algorithm
%   alg_mean_ay                     Data structure of lateral mean value regression algorithm
%___________________________________________________________________________________________________
%% Algorithm


% initialize outputs
ax_pred_mps2 = zeros(30, 1); 
ay_pred_mps2 = zeros(30, 1); 

% update mean prediction for longitudinal and lateral disturbance model
[ax_pre_update_mps2, alg_mean_ax_upd] = learnDisturbanceMean(...
        [LongDisturbanceEstimation.ax_Target_mps2; ...
         LongDisturbanceEstimation.dax_Target_mps3],... 
         LongDisturbanceEstimation.ax_DistEstimate_mps2, true, alg_mean_ax); 
[ay_pre_update_mps2, alg_mean_ay_upd] = learnDisturbanceMean(...
        [LatDisturbanceEstimation.ay_Target_mps2; ...
         LatDisturbanceEstimation.v_Target_mps],... 
         LatDisturbanceEstimation.ay_DistEstimate_mps2, true, alg_mean_ay); 
     
% update quantile estimate for model error 
alg_quantile_ax_upd = learnDisturbanceQuantile(...
    (LongDisturbanceEstimation.ax_DistEstimate_mps2 - ax_pre_update_mps2), alg_quantile_ax); 
alg_quantile_ay_upd = learnDisturbanceQuantile(...
    (LatDisturbanceEstimation.ay_DistEstimate_mps2 - ay_pre_update_mps2), alg_quantile_ay); 

% generate predictions for predicted trajectory
for i = 1:1:30
    [ax_pred_mps2(i), ~] = learnDisturbanceMean(...
            [ax_Target_mps2(i); dax_Target_mps3(i)], 0, false, alg_mean_ax); 
    [ay_pred_mps2(i), ~] = learnDisturbanceMean(...
            [ay_Target_mps2(i); v_Target_mps(i)], 0, false, alg_mean_ay); 
end

ax_quantile_mps2 = alg_quantile_ax.q_est; 
ay_quantile_mps2 = alg_quantile_ay.q_est;     