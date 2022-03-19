function [w_mean, M_tilde] = processPrediction(ax_pred, ay_pred, ax_quantile_mps2_upper,...
    ax_quantile_mps2_lower, ay_quantile_mps2_upper, ay_quantile_mps2_lower, P_VDC_UseLearnedPrediction, N, P_VDC_Dist_ax_mps2, P_VDC_Dist_ay_mps2)
% function processPrediction
% Author:       Martin Euler         last update: 18-03-2020
% Description:  
%   function used to process the disturbance prediciton for the TMPC-Controller 
% Inputs/parameters:
%   ax_pred: predicted mean for ax
%   ay_pred: predicted mean for ay
%   ax_quantile_mps2_upper: quantile of upper bound for ax
%   ax_quantile_mps2_lower: quantile of lower bound for ax
%   ay_quantile_mps2_upper: quantile of upper bound for ay
%   ay_quantile_mps2_lower: quantile of lower bound for ay
%   P_VDC_UseLearnedPrediction: flag whether prediction should be used or not
%   N: length of control horizon
%   P_VDC_Dist_ax_mps2: Disturbance bounds for ax used when learning is disabled
%   P_VDC_Dist_ay_mps2: Disturbance bounds for ax used when learning is disabled
% Outputs:
%   w_mean: vector with mean prediction values
%   M_tilde: vector with semi-axis for disturbance ellipsoids

w_mean = zeros(2*N,1);
M_tilde = zeros(2,1);
% check if prediction should be used or not
if P_VDC_UseLearnedPrediction
    % calculate the mean value of the prediction
    for i = 1:N
        w_mean(2*i-1) = ax_pred(i);
        w_mean(2*i) = ay_pred(i);
    end
    % calculate vector with semi-axis for disturbance ellipsoids
    M_tilde(1) = (ax_quantile_mps2_upper(1)-ax_quantile_mps2_lower(1))/2.0;
    M_tilde(2) = (ay_quantile_mps2_upper(1)-ay_quantile_mps2_lower(1))/2.0;
else
    % calculate vector with semi-axis for disturbance ellipsoids
    M_tilde(1) = P_VDC_Dist_ax_mps2;
    M_tilde(2) = P_VDC_Dist_ay_mps2;
end

end