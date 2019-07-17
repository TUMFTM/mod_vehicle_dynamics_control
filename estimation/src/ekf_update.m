function [x_est, P_est, res] = ekf_update(x_pred, z, H_d, R, n, m,...
                                     P_pred, outlier_bound)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               Madeline Wolz 
% 
% Start Date:   06.02.2018
% 
% Description:  implements the update step of an Extended Kalman Filter. 
%               Handles outliers according to the bounds specified. 
% 
% Inputs: 
%   x_pred            State estimate vector before correction
%   z                 Measurement vector
%   H_d               Measurement matrix 
%   R                 Measurement covariance matrix 
%   n                 Mask for system state vector which specifieds whether angular 
%                       normalization has to be applied to this element 
%   m                 Mask for system state vector which specifieds whether angular 
%                       normalization has to be applied to this element 
%   P_pred            Covariance of the state estimate vector before correction
%   outlier_bound     Number of standard deviations tolerated until 
%                       outlier rejection is applied 
% 
% Outputs: 
%   x_est             State estimate after correction
%   P_est             State estimate covariance after correction
%   res               Measurement residuals
%__________________________________________________________________________


%% Update

% Innovation / measurement residual
res = z - H_d*x_pred;  
% normalize angles at vector positions given by m
res(m) = normalizeAngle(res(m));
        
% Innovation (residual) covariance
S = H_d * P_pred * H_d' + R; 
% Kalman Gain (near-optimal)
K = P_pred * H_d' / S;  

% detect outliers if they are larger than outlier_bound
% limit the values to outlier_bound
fusion_y = zeros(length(res), 1); 
for i = 1:1:length(res)
    fusion_y(i) = min(abs(res(i)), outlier_bound(i))*sign(res(i));
end

% Updated state estimate
x_est = x_pred + K * fusion_y; 
x_est(n) = normalizeAngle(x_est(n)); 

%% Outputs
 P_est     = (eye(length(x_pred)) - K*H_d) * P_pred;   % updated covariance estimate     
%y     = y;                      % measurement resiudal

end

