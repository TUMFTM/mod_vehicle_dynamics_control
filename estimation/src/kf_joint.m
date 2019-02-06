function [x_est, P_update, res]= kf_joint(x, z, u, fusionVec, P_VDC_measCov_GPS,...
                  InputCov, tS_s, psi_forCov_rad, P,  P_VDC_measCov_velocity,...
                  P_VDC_measCov_VLOC, P_VDC_OutlierBounds, P_VDC_YawAngleDepCovMode,... 
                  P_VDC_EnableInputCrossCorrelation)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               Madeline Wolz
% 
% Start Date:   03.02.2018
% 
% Description:  implements a an Extended Kalman Filter for fusing acceleration,
%               velocity and position sensor information. The acceleration measurements
%               are used as an input to the system model while the others are 
%               used to correct the state estimates. The system model itself is a 
%               purely kinematic forward integration of accelerations in the vehicle 
%               coordinate frame and furthermore integration of the resulting velocities
%               and the yaw rate in a global inertial frame. The main benefit of this 
%               concept compared to a more sophisticated model is its consistent accuracy
%               over the complete nonlinear tire range. Even though all state variables
%               are directly measurable, the Kalman Filter fusion improves the signal 
%               quality and provides a natural framework to handle multi rate
%               measurements. 
%            
% Inputs:
%   x                                   State vector - [x_m, y_m, psi_rad, vx_mps, vy_mps]
%   z                                   Measurement vector - 
%                                         [xGPS_m, yGPS_m, psiGPS_rad, xVLOC_m, yVLOC_m, psiVLOC_rad,
%                                         vx_mps, vy_mps]
%   u                                   Input vector - [dPsi_radps, ax_mps2, ay_mps2]
%   fusionVec                           Bit mask for measurement vector z
%                                         which enables/disables fusion 
%   P_VDC_measCov_GPS                   Covariance values of the GPS measurements 
%                                         [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                                         in case that track dependent covariances are active. 
%                                         [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                                         in case that track dependent covariances are disabled 
%   InputCov                            Covariance values of the input vector u 
%   tS_s                                Sample time
%   psi_forCov_rad                      Current yaw Angle used for yaw Angle dependant covariance
%                                         Either the GPS yaw Angle or the VLOC yaw Angle
%   P                                   Covariance matrix of the state estimation 
%   P_VDC_measCovVelocity               Covariance values of the measurement vector 
%                                         [sigma^2_vx, sigma^2_vy]
%   P_VDC_measCov_VLOC                  Covariance values of the visual localization measurements 
%                                         [sigma^2_longitudinal, sigma^2_lateral, sigma^2_orientation] 
%                                         in case that track dependent covariances are active. 
%                                         [sigma^2_x, sigma^2_y, sigma^2_orientation]
%                                         in case that track dependent covariances are disabled 
%   P_VDC_OutlierBounds                 Number of standard deviations tolerated until 
%                                         outlier rejection is applied 
%   P_VDC_YawAngleDepCovMode            Mode for YawAngle dependant covariances
%                                         0: yawAngle dependancy disabled
%                                         1: GPS yawAngle used for covariance rotation
%                                         2: VLOC yawAngle used for covariance rotation
%   P_VDC_EnableInputCrossCorrelation   enables the cross correlation between
%   input variables
%
% Outputs: 
%   x_est                   Updated state vector - [vxCG_mps, vyCG_mps]
%   P_updated               Updated state estimate covariance matrix 
%   res                     Measurement residuals
%__________________________________________________________________________
           
%% Initialization
% map state and input variables to named variables for easier usage
x_m             =  x(1);
y_m             =  x(2);
psi_rad         =  x(3);
vx_mps          =  x(4);
vy_mps          =  x(5);
dPsi_radps      = u(1);
ax_mps2         = u(2);
ay_mps2         = u(3);
% specify mask for states which need angular normalization 
n = logical([0, 0, 1, 0, 0]');
% specify mask for measurements which need angular normalization
m = logical([0, 0, 1, 0, 0, 1, 0, 0]'); 

%% Construct discretization
% Jacobian matrix A consisting of partial derivatives with respect to the 
% system state from the system update equation (specified below in function pmm)  
A = [0, 0,(-cos(psi_rad) * vx_mps + sin(psi_rad) * vy_mps),...
       -sin(psi_rad), - cos(psi_rad);...
     0, 0,( -sin(psi_rad) * vx_mps - cos(psi_rad) * vy_mps),...
        cos(psi_rad),  -sin(psi_rad);...
     0, 0, 0, 0, 0;...
     0, 0, 0, 0, dPsi_radps;...
     0, 0, 0, -dPsi_radps, 0];
% discretize using euler forward
A_d = eye(length(x)) +tS_s*A; 
% Jacobian matrix B consisting of partial derivatives with respect to the 
% system input from the system update equation (specified below in function pmm)
if(P_VDC_EnableInputCrossCorrelation)
    B = [0, 0, 0;...
         0, 0, 0;...
         1, 0, 0;...
         vy_mps, 1, 0;...
         -vx_mps, 0, 1];
else
    B = [0, 0, 0;...
         0, 0, 0;...
         1, 0, 0;...
         0, 1, 0;...
         0, 0, 1];
end
% discretize using euler forward
B_d = tS_s*B; 

%% observation matrices
% nominal measurement matrix if all sensors are available 
H_full = [1, 0, 0, 0, 0;...
          0, 1, 0, 0, 0;...
          0, 0, 1, 0, 0;...
          1, 0, 0, 0, 0;...
          0, 1, 0, 0, 0;...
          0, 0, 1, 0, 0;...
          0, 0, 0, 1, 0;...
          0, 0, 0, 0, 1;];
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
R_Vel = diag([P_VDC_measCov_velocity]);
R = [R_GPS, zeros(3, 3), zeros(3, 2);...
    zeros(3, 3), R_VLOC, zeros(3, 2);...
    zeros(2, 3), zeros(2, 3), R_Vel];


%% apply Kalman Filter Algorithm
% Prediction usingi explicitly specified analytic linearization
[x, P_pred] = ekf_prediction_analytic(x, u, P, InputCov, n, @pmm, A_d, B_d, tS_s); 
% Call kalman filter update function 
[x_est, P_update, res] = ekf_update( x, z, H_d, R, n, m, P_pred, P_VDC_OutlierBounds); 
     
end

function xpred = pmm(x, u, tS)
  %% Description  
  % rigid body model using accelerations and yaw rate as inputs and applying simple forward
  % integration in vehicle coordinate frame to obtain the velocities. The latter are then 
  % used to apply forward integration in global cartesian coordinates to obtain the
  % position. 
  %% State and Input mapping
  Psi_rad = x(3); 
  vx_mps = x(4); 
  vy_mps = x(5); 
  dPsi_radps = u(1); 
  ax_mps2 = u(2); 
  ay_mps2 = u(3); 
  %% Differential equations
  dx = [-sin(Psi_rad) * vx_mps - cos(Psi_rad) * vy_mps;...    
     cos(Psi_rad) * vx_mps - sin(Psi_rad) * vy_mps;... 
     dPsi_radps;...
     ax_mps2 + dPsi_radps * vy_mps;...
     ay_mps2 - dPsi_radps * vx_mps;]; 
  %% Integration
  xpred = x + tS*dx;   
end