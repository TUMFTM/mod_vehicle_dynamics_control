function [x_est, P_update, res]= kf_speed(x, z, u, tS_s, fusionVec, InputCov,...
                     P, P_VDC_measCov_velocity, P_VDC_OutlierBounds)
%_________________________________________________________________
%% Documentation
%   
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
%               Madeline Wolz
% 
% Start Date:   03.02.2018
%
% Description:  implements an Extended Kalman filter for fusing velocity  
%               and acceleration measurements. The latter are used as an 
%               input to the system model while the former are used for 
%               correction. The system model itself is a purely kinematic 
%               forward integration of accelerations in the vehicle coordinate 
%               frame. The main benefit of this concept compared to a 
%               more sophisticated model is its consistent accuracy over 
%               the complete nonlinear tire range. Even though all state variables 
%               are measureable, the Kalman Filter fusion significantly 
%               improves the signal quality which is beneficial for control. 
% 
% Inputs: 
%   x                       State vector - [vxCG_mps, vyCG_mps]
%   z                       Measurement vector - [vxCG_mps, vyCG_mps]
%   u                       Input vector - [axCG_mps2, ayCG_mps2, dPsi_radps]
%   tS_s                    Sample time in s
%   fusionVec               Bit mask for measurement vector z
%                             which enables/disables fusion
%   InputCov                Covariance values of the input vector u 
%   P                       Covariance matrix of the state estimate 
%   P_VDC_measCovVelocity   Covariance values of the measurement vector 
%                             [sigma^2_vx, sigma^2_vy]
%   P_VDC_OutlierBounds     Number of standard deviations tolerated until 
%                             outlier rejection is applied 
% Outputs: 
%   x_est                   Updated state vector - [vxCG_mps, vyCG_mps]
%   P_updated               Updated state estimate covariance matrix 
%   res                     Measurement residuals
%__________________________________________________________________________


%% Initialization
% map state and input variables to named variables for easier usage 
vx_mps          =    x(1); 
vy_mps          =    x(2); 
yawRate_radps   =    u(1);
ax_mps          =    u(2);
ay_mps          =    u(3);
% specify mask for states which need angular normalization 
n = [];
% specify mask for measurements which need angular normalization 
m = [];

%% Construct discretization
% Jacobian matrix A consisting of partial derivatives with respect to the 
% system state from the system update equation (specified below in function vxvy)  
A = [0, yawRate_radps;...
    -yawRate_radps, 0];
% discretize using euler forward
A_d = eye(length(x)) + tS_s*A;  
% Jacobian matrix B consisting of partial derivatives with respect to the 
% system input from the system update equation (specified below in function vxvy)
B = [1, 0, vy_mps;...
     0, 1, -vx_mps];
% discretize using euler forward
B_d = tS_s*B; 
  
%% Construct measurement matrix and measurement covariance matrix 
% nominal measurement matrix if all sensors are available 
H_full = [1, 0;...
          0, 1];
% extraction matrix which only uses the sensors configured and updated
H_ext = diag(double(fusionVec));
% build final measurement matrix with zero entries for sensors not used
H_d = H_ext*H_full;  

% construct measurement covariance matrix 
R = diag(P_VDC_measCov_velocity);

%% apply Extended Kalman Filter Algorithm
% Prediction usingi explicitly specified analytic linearization
[x, P_pred] = ekf_prediction_analytic(x, u, P, InputCov, n, @vxvy, A_d, B_d, tS_s); 
% Call kalman filter update function 
[x_est, P_update, res] = ekf_update( x, z, H_d, R, n, m, P_pred, P_VDC_OutlierBounds); 
      
end

function xpred = vxvy(x, u, tS_s)
  %% Description  
  % point mass model using simple forward integration of accelerations and yaw rate
  % in vehicle centered coordinate frame
  %% State and Input mapping
  vx_mps = x(1); 
  vy_mps = x(2); 
  dPsi_radps = u(1); 
  ax_mps2 = u(2); 
  ay_mps2 = u(3); 
  %% Differential equations
  dx = [(ax_mps2 + vy_mps*dPsi_radps);...
          (ay_mps2 - vx_mps*dPsi_radps)]; 
  %% Integration
  xpred = x + tS_s*dx;    
end