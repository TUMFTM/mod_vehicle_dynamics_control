function [x_est, P_update, res]= kf_stm(x, z, u, tS_s, InputCov,...
          P, P_VDC_OutlierBounds, P_VDC_YawAngleDepCovMode, fusionVec, psi_forCov_rad,...
          P_VDC_measCov_GPS, P_VDC_measCov_VLOC, P_VDC_measCov_velocity, P_VDC_Var_yawRate_radps)
%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   15.02.2018
% 
% Description:  implements a mix between an Extended and an Unscented Kalman Filter to
%               fuse velocity and position sensor information. The filter model is a 
%               nonlinear single track model with a four coefficient pacjeka model. 
%               The system input is the steering angle and the drivetrain force requested
%               by the controller. 
%            
% Inputs:
%   x                         State vector - [x_m, y_m, psi_rad, vx_mps, vy_mps, dPsi_radps]
%   z                         Measurement vector - 
%                               [xGPS_m, yGPS_m, psiGPS_rad, xVLOC_m, yVLOC_m, psiVLOC_rad,
%                                 vx_mps, vy_mps, dPsi_radps]
%   u                         Input vector - [Delta_rad, Fx_Powertrain_kN]
%   tS_s                      Sample time
%   InputCov                  Covariance values of the input vector u 
%   P                         Covariance matrix of the state estimation 
%   P_VDC_OutlierBounds       Number of standard deviations tolerated until 
%                               outlier rejection is applied 
%   P_VDC_YawAngleDepCovMode  Mode for YawAngle dependant covariances
%                               0: yawAngle dependancy disabled
%                               1: GPS yawAngle used for covariance rotation
%                               2: VLOC yawAngle used for covariance rotation
%   fusionVec                 Bit mask for measurement vector z
%                               which enables/disables fusion 
%   psi_forCov_rad            Current yaw Angle used for yaw Angle dependant covariance
%                               Either the GPS yaw Angle or the VLOC yaw Angle
%   P_VDC_measCov_GPS         Covariance values of the GPS measurements 
%                               [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                               in case that track dependent covariances are active. 
%                               [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                               in case that track dependent covariances are disabled 
%   P_VDC_measCov_VLOC        Covariance values of the visual localization measurements 
%                               [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                               in case that track dependent covariances are active. 
%                               [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                               in case that track dependent covariances are disabled 
%   P_VDC_measCovVelocity     Covariance values of the measurement vector 
%                               [sigma^2_vx, sigma^2_vy]
%   P_VDC_Var_yawRate_radps   Covariance values of the yaw rate measurement
%                               [sigma^2_dPsi]
% Outputs: 
%   x_est                     Updated state vector - [vxCG_mps, vyCG_mps]
%   P_updated                 Updated state estimate covariance matrix 
%   res                       Measurement residuals
%__________________________________________________________________________
           

%% Initializiation
% specify mask for states which need angular normalization 
n = logical([0, 0, 1, 0, 0, 0]');
% specify mask for measurements which need angular normalization 
m = logical([0, 0, 1, 0, 0, 1, 0, 0, 0]'); 

%% Construct measurement matrix and measurement covariance matrix
% nominal measurement matrix if all sensors are available 
H_full = [1, 0, 0, 0, 0, 0;...
          0, 1, 0, 0, 0, 0;...
          0, 0, 1, 0, 0, 0;...
          1, 0, 0, 0, 0, 0;...
          0, 1, 0, 0, 0, 0;...
          0, 0, 1, 0, 0, 0;...
          0, 0, 0, 1, 0, 0;...
          0, 0, 0, 0, 1, 0;...
          0, 0, 0, 0, 0, 1;];
% extraction matrix which only uses the sensors configured and updated
H_ext = diag(double(fusionVec));
% build final measurement matrix with zero entries for sensors not used
H_d = H_ext*H_full;  

% map measurement covariances to track orientation 
if(P_VDC_YawAngleDepCovMode>0)
  R_GPS = transCov2TrackOrientation(diag(P_VDC_measCov_GPS), psi_forCov_rad);
  R_VLOC = transCov2TrackOrientation(diag(P_VDC_measCov_VLOC), psi_forCov_rad);
else
  R_GPS = diag(P_VDC_measCov_GPS);
  R_VLOC = diag(P_VDC_measCov_VLOC);
end
% construct measurement covariance matrix
R_Vel = diag([P_VDC_measCov_velocity P_VDC_Var_yawRate_radps]);
R = [R_GPS, zeros(3, 3), zeros(3, 3);...
    zeros(3, 3), R_VLOC, zeros(3, 3);...
    zeros(3, 3), zeros(3, 3), R_Vel];

%% apply Kalman Filter Algorithm
% Covariance prediction using a numeric method to derive the linearization of the 
% system model which is used for covariance prediction. 
[x, P_pred] = ekf_prediction_numeric(x, u, P, InputCov, n, @nstm, tS_s); 
% update step 
[x_est, P_update, res] = ekf_update( x, z, H_d, R, n, m, P_pred, P_VDC_OutlierBounds); 

end

function xpred = nstm(x, u, tS)
  
  % vehicle parameters
  m = 1200; 
  J = 1260;
  lf = 1.51; 
  lr = 1.38; 
  % tire parameters 
  PacFrontLat = [3.26 9.07 5500 25];
  PacRearLat = [10 4.12 6000 8.42]; 
  
  % calculate side slip angles
  alphaF = u(1) - atan((x(5)+x(6)*lf)/max(0.5, x(4)));
  alphaR = -atan((x(5)-x(6)*lr)/max(0.5, x(4)));
  
  if(x(4) < 2)
    FyF = 0; 
    FyR = 0; 
  else
    FyF = 2*PacFrontLat(3).*sin(PacFrontLat(2).*atan(PacFrontLat(1).*alphaF - PacFrontLat(4).*(PacFrontLat(1).*alphaF - atan(PacFrontLat(1).*alphaF)))); 
    FyR = 2*PacRearLat(3).*sin(PacRearLat(2).*atan(PacRearLat(1).*alphaR - PacRearLat(4).*(PacRearLat(1).*alphaR - atan(PacRearLat(1).*alphaR)))); 
  end
    
  dx = [-sin(x(3)) * x(4) - cos(x(3)) * x(5);...
    cos(x(3)) * x(4) - sin(x(3)) * x(5);...
    x(6);...
    1/m*u(2)*1000 + x(6)*x(5);...
    1/m*(FyF+FyR) - x(6)*x(4);...
    1/J*(FyF*lf - FyR*lr);]; 

  xpred = x + tS*dx;
end