function [ax_Target_mps2, ay_Target_mps2, v_Target_mps, FeaturePredictionOK] = ...
    tubeFeaturePrediction(ActualTargetPath, OptimizedTrajectory, tSSlow)
%% Documentation 
%
% Author: Alexander Wischnewski   
%
% Start Date:   13.12.2019
% Last Update:  12.03.2020
%
% Description:  
%   calculates the relevant features for the prediction of the control error tubes based on the
%   upcoming trajectory segment. 
%
% Inputs:
%   ActualTargetPath                Target trajectory
%   tSSlow                          TMPC Sample Rate
% 
% Outputs: 
%   ax_Target_mps2                  Feature vector for ax_Target
%   ay_Target_mps2                  Feature vector for ay_Target
%   v_Target_mps                    Feature vector for v_Target
%   FeaturePredictionOK             Feature prediction successfull
%___________________________________________________________________________________________________
%% Algorithm

% parameters 
N_pred = 51; 

% initialize output variables
ax_Target_mps2 = zeros(N_pred, 1); 
ay_Target_mps2 = zeros(N_pred, 1); 
v_Target_mps = zeros(N_pred, 1); 
FeaturePredictionOK = false; 

% calculate prediction s variables from old trajectory
s_m_predicted = [OptimizedTrajectory.s_loc_m;...
                    OptimizedTrajectory.s_loc_m(end) + tSSlow*OptimizedTrajectory.v_mps(end)]; 

% check if a path is available and long enough, otherwise do not use prediction
if(ActualTargetPath.TrajCnt > 0 && s_m_predicted(end) <= ActualTargetPath.s_loc_m(end))
    FeaturePredictionOK = true; 
    v_Target_mps = interp1(ActualTargetPath.s_loc_m, ActualTargetPath.v_mps, s_m_predicted, 'linear', 0);
    ax_Target_mps2 = interp1(ActualTargetPath.s_loc_m, ActualTargetPath.ax_mps2, s_m_predicted, 'previous');
    kappa_radpm = interp1(ActualTargetPath.s_loc_m, ActualTargetPath.kappa_radpm, s_m_predicted, 'linear', 0);
    ay_Target_mps2 = kappa_radpm.*v_Target_mps.^2;
end

