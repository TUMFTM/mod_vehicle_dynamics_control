function [x_pred, P_pred] = ekf_prediction_analytic(x, u, P, InputCov, n, f, A_d, B_d, tS)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               Madeline Wolz
% 
% Start Date:   05.02.2018
% 
% Description:  implements the prediction step of an Extended Kalman Filter. 
%               The linearization is given explicitly via the function arguments. 
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
%   A_d         Discrete system update matrix 
%   B_d         Discrete system input matrix 
%   tS          Sample time 
% 
% Outputs: 
%   x_pred      State estimate prediction based on the system equation 
%   P_pred      Covariance prediction based on the system equation 

%% Algorithm
% Calculate process noise covariance by combining the input covariance 
% and the linearized version of the system input vector 
Q = B_d * diag(InputCov) * B_d';

% Predicted state estimate function f_d
x_pred = f(x, u, tS);
% NormalizeAngle(x)
x_pred(n) = normalizeAngle(x_pred(n));
% Predicted covariance estimate
P_pred = A_d * P * A_d' + Q; 

end

