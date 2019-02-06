function [x_est, P_update, res]= kf_pose(x,...
                    z, u, fusionVec, psi_forCov_rad, InputCov, P,...
          P_VDC_YawAngleDepCovMode, P_VDC_measCov_GPS, P_VDC_measCov_VLOC,...
          tS_s, P_VDC_OutlierBounds)
%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
%               Madeline Wolz
% 
% Start Date:   03.02.2018
% 
% Description:  implements an Extended Kalman Filter for fusing velocity 
%               and pose measurements. The former are used as an input to the 
%               systm model while the latter are used for correction. The model 
%               itself is a purely kinematic forward integration of velocities
%               in a global inertial coordinate frame. Even though all state 
%               variables are directly available via measurement, the intent 
%               of this filter is to provide a inituitve framework for fusing
%               different localization algorithms, handling their corresponding 
%               strengths and weaknesses and handling their different sample rates. 
%            
% Inputs:
%   x                         State vector - [x_m, y_m, psi_rad]
%   z                         Measurement vector - 
%                               [xGPS_m, yGPS_m, psiGPS_rad, xVLOC_m, yVLOC_m, psiVLOC_rad]
%   u                         Input vector - [vx_mps, vy_mps, dPsi_radps]
%   fusionVec                 Bit mask for measurement vector z
%                               which enables/disables fusion 
%   psi_forCov_rad            Current yaw Angle used for yaw Angle dependant covariance
%                               Either the GPS yaw Angle or the VLOC yaw Angle
%   InputCov                  Covariance values of the input vector u 
%   P                         Covariance matrix of the state estimation 
%   P_VDC_YawAngleDepCovMode  Mode for YawAngle dependant covariances
%                               0: yawAngle dependancy disabled
%                               1: GPS yawAngle used for covariance rotation
%                               2: VLOC yawAngle used for covariance rotation
%   P_VDC_measCov_GPS         Covariance values of the GPS measurements 
%                               [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                               in case that track dependent covariances are active. 
%                               [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                               in case that track dependent covariances are disabled 
%   P_VDC_measCov_VLOC         Covariance values of the visual localization measurements 
%                               [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                               in case that track dependent covariances are active. 
%                               [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                               in case that track dependent covariances are disabled 
%   tS_s                      Sample time
%   P_VDC_OutlierBounds       Number of standard deviations tolerated until 
%                               outlier rejection is applied 
% Outputs: 
%   x_est                   Updated state vector - [vxCG_mps, vyCG_mps]
%   P_updated               Updated state estimate covariance matrix 
%   res                     Measurement residuals
%__________________________________________________________________________


%% Initialization
% map state and input variables to named variables for easier ussage 
x_m       = x(1);
y_m       = x(2);
psi_rad   = x(3);
vx_mps    = u(1);
vy_mps    = u(2);
dPsi_rad  = u(3);
% specify mask for states which need angular normalization 
n = logical([0, 0, 1]');
% specify mask for measurements which need angular normalization 
m = logical([0, 0, 1, 0, 0, 1]'); 

%% Construct discretization
% Jacobian matrix A consisting of partial derivatives with respect to the 
% system state from the system update equation (specified below in function pmm)  
A = [0, 0, (-cos(psi_rad) * vx_mps + sin(psi_rad) * vy_mps);...
     0, 0, (-sin(psi_rad) * vx_mps - cos(psi_rad) * vy_mps);...
     0, 0, 0];      
% discretize using euler forward
A_d = eye(length(x)) + tS_s*A; 
% Jacobian matrix B consisting of partial derivatives with respect to the 
% system input from the system update equation (specified below in function pmm)
B = [-sin(psi_rad), -cos(psi_rad), 0;...
      cos(psi_rad), -sin(psi_rad), 0;...
      0, 0, 1];
% discretize using euler forward
B_d = tS_s*B; 
  
%% Construct measurement matrix and measurement covariance matrix 
% nominal measurement matrix if all sensors are available 
H_full = [1, 0, 0;...
          0, 1, 0;...
          0, 0, 1;...
          1, 0, 0;...
          0, 1, 0;...
          0, 0, 1;];
% extraction matrix which only uses the sensors configured and updated
H_ext = diag(double(fusionVec));
% build final measurement matrix with zero entries for sensors not used
H_d = H_ext*H_full;  

% map measurement covariances to track orientation if activated
if(P_VDC_YawAngleDepCovMode>0)
  R_GPS = transCov2TrackOrientation(diag(P_VDC_measCov_GPS), psi_forCov_rad); 
  R_VLOC = transCov2TrackOrientation(diag(P_VDC_measCov_VLOC), psi_forCov_rad); 
else
  R_GPS = diag(P_VDC_measCov_GPS); 
  R_VLOC = diag(P_VDC_measCov_VLOC); 
end
% construct measurement covariance matrix 
R = [R_GPS, zeros(3, 3);...
    zeros(3, 3), R_VLOC];

%% apply Extended Kalman Filter Algorithm
% Prediction using explicitly specified analytic linearization
[x, P_pred] = ekf_prediction_analytic(x, u, P, InputCov, n, @pmm, A_d, B_d, tS_s); 
% Measurement correction
[x_est, P_update, res] = ekf_update( x, z, H_d, R, n, m, P_pred, P_VDC_OutlierBounds); 

end

function xpred = pmm(x, u, tS_s)
  %% Description  
  % rigid body model using inputs and yaw rate as inputs and applying simple forward
  % integration in cartesian coordinate frame 
  %% State and Input mapping
  psi_rad = x(3); 
  vx_mps = u(1); 
  vy_mps = u(2); 
  dPsi_radps = u(3); 
  %% Differential equations
  dx = [-sin(psi_rad) * vx_mps - cos(psi_rad) * vy_mps;...    
        cos(psi_rad) * vx_mps - sin(psi_rad) * vy_mps;... 
        dPsi_radps]; 
  %% Integration
  xpred = x + tS_s*dx;   
end