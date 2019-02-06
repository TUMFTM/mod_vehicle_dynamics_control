function [x_pred, P_pred] = ekf_prediction_numeric(x, u, P, InputCov, n, f, tS)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               Madeline Wolz
% 
% Start Date:   05.02.2018
% 
% Description:  implements the prediction step of an Extended Kalman Filter. 
%               The linearization is obtained by applying complex step differentiation 
%               to the system update equation. 
% 
% Inputs: 
%   x           System state vector 
%   u           System input vector
%   P           State estimate covariance matrix 
%   InputCov    Covariance values for the system input vector 
%   n           Mask for system state vector which specifieds whether angular 
%                 normalization has to be applied to this element 
%   f           Function handle to the system update equation 
%                 of the form x(k+1) = f(x(k), u(k), tS)
%   tS          Sample time 
% 
% Outputs: 
%   x_pred      State estimate prediction based on the system equation 
%   P_pred      Covariance prediction based on the system equation 

%% Algorithm 
% Obtain jacobian matrix B of partial derivatives of f of input vector u
[~, B_d] = jaccsd_input(f, x, u, tS);
% Calculate process noise covariance by combining the input covariance 
% and the linearized version of the system input vector 
Q = B_d * diag(InputCov) * B_d';

% Predicted state estimate function f_d
[x_pred, A_d] = jaccsd_state(f, x, u, tS);
% NormalizeAngle(x)
x_pred(n) = normalizeAngle(x_pred(n));
% Predicted covariance estimate
P_pred = A_d * P * A_d' + Q; 

end

