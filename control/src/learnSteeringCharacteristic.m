function [SC_Kappa_Out, SC_Vel_Out, SC_DeltaNeutral_Out, SC_meas_Out] = ...
    learnSteeringCharacteristic(steeringRequest_rad, kappa_radpm, ... 
                                v_abs_mps, P_VDC_SCLearn_Sigma, ...
                            P_VDC_SCLearn_LengthScales, l_front_m, l_rear_m, ...
                            P_VDC_SCLearn_FilterCoeff, P_VDC_SCLearn_KappaMax_radpm, ...
                            P_VDC_SCLearn_vMax_mps, P_VDC_MinVelSlipCalc_mps)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   25.01.2018
% 
% Description:  learns the current steering characteristic and can
%               therefore improve the feedforward quality. To achieve this,      
%               the steering request is learned as a function of curvature
%               and velocity. 
% 
% Inputs: 
%   steeringRequest_rad:            Steering angle request in rad
%   kappa_radpm:                    Current curvature in radpm 
%   v_abs_mps:                      Current velocity in mps
%   P_VDC_SCLearn_Sigma:            Measurement noise for learning samples 
%   P_VDC_SCLearn_LengthScales:     Measurement noise for learning samples 
%   l_front_m:                      Distance between CoG and front axle
%   l_rear_m:                       Distance between CoG and rear axle 
%   P_VDC_SCLearn_FilterCoeff:      Low pass filter coefficient for sample
%                                   aggregation 
%   P_VDC_SCLearn_KappaMax_radpm:   Maximum curvature for learning 
%   P_VDC_SCLearn_vMax_mps:         Maximum velocity for learning 
% 
% Outputs: 
%   SteeringCharacteristic: Final steering characteristic to be used as 
%                               delta to neutral steer

persistent SC_DeltaNeutral SC_visited SC_Kappa SC_Vel y_pred

%% Parameter definitions 
% number of discretization points for characteristic 
nPoints_kappa = 21;
% number of discretization poitns for characteristic 
nPoints_v = 15;
% discretized points for curvature
kappa_points = linspace(-P_VDC_SCLearn_KappaMax_radpm,...
    P_VDC_SCLearn_KappaMax_radpm, nPoints_kappa); 
% discretized points for velocity 
v_points = linspace(0, P_VDC_SCLearn_vMax_mps, nPoints_v); 
if(isempty(SC_DeltaNeutral))
    % initialize data points with zeros 
    SC_DeltaNeutral = zeros(nPoints_kappa, nPoints_v); 
    % this meshgrid stores whether a data point was already visted
    SC_visited = false(nPoints_kappa, nPoints_v); 
    % create sample matrix 
    [SC_Vel, SC_Kappa] = meshgrid(v_points, kappa_points); 
    y_pred = zeros(nPoints_kappa*nPoints_v, 1); 
end

% only update steering characteristic if above minimum speed 
if(v_abs_mps > P_VDC_MinVelSlipCalc_mps)
    %% update steering characteristic based on data 
    % find closest point to current data 
    [~, idx_kappa] = min(abs(kappa_points - kappa_radpm)); 
    [~, idx_v] = min(abs(v_points - v_abs_mps)); 

    % implement PT1 characteristic for update equation 
    SC_DeltaNeutral(idx_kappa, idx_v) = SC_DeltaNeutral(idx_kappa, idx_v)*P_VDC_SCLearn_FilterCoeff + ...
        (steeringRequest_rad - kappa_radpm*(l_front_m+l_rear_m))*(1-P_VDC_SCLearn_FilterCoeff); 
    % update visited bit for all points where we have data 
    SC_visited(idx_kappa, idx_v) = true; 

    %% Filter learned characteristic via GP and output it
    % calculate GP Kernel
    SC_GPDataKernel_dyn = kernel([SC_Kappa(SC_visited), SC_Vel(SC_visited)],...
        [SC_Kappa(SC_visited), SC_Vel(SC_visited)], P_VDC_SCLearn_LengthScales) + ...
        + P_VDC_SCLearn_Sigma.^2.*eye(length(SC_Kappa(SC_visited))); 
    % calculate kernel part which correlates predictions with measurements 
    SC_GPPredKernel_dyn = kernel([SC_Kappa(:), SC_Vel(:)],...
        [SC_Kappa(SC_visited), SC_Vel(SC_visited)], P_VDC_SCLearn_LengthScales); 
    % calculate predictions based on already acquired data points
    y_pred = SC_GPPredKernel_dyn*(SC_GPDataKernel_dyn\SC_DeltaNeutral(SC_visited)); 
end
SC_DeltaNeutral_Out = reshape(y_pred, size(SC_DeltaNeutral)); 
SC_Kappa_Out = SC_Kappa;
SC_Vel_Out = SC_Vel; 
SC_meas_Out = SC_DeltaNeutral;
end

function K = kernel(X, Y, length_scales)
    % get shape of elements
    nX = size(X);
    nY = size(Y);
    % scale x and y according to kernel hyperparameter
    X_s = X;
    for i = 1:1:nX(1)
        X_s(i, :) = X(i, :)./length_scales; 
    end
    Y_s = Y;
    for i = 1:1:nY(1)
        Y_s(i, :) = Y(i, :)./length_scales; 
    end
    % compute kernel matrix
    K = zeros(nX(1), nY(1)); 
    for i = 1:1:nX(1)
        for j = 1:1:nY(1)
            K(i, j) = 0.05.^2.*exp(-0.5*sum((X_s(i, :)-Y_s(j, :)).^2)); 
        end
    end
end