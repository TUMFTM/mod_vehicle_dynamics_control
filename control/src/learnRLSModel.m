function [w_upd, P_upd, mean_debug, cov_debug] = learnRLSModel(v, y_meas, w_old, P_old, acc_control_learning)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% 
% Description:  
%   learns a linaer model for the features inserted
% 
% Inputs: 
%   v:                      feature vector 
%   y_meas:                 measured value
%   w_old:                  previous iteration weights
%   P_old:                  previous iteration covariance matrix
%   acc_control_learning:   Parameter structure for the learning algorithm
%   
%
% Outputs: 

% parameters 
Q = 1e-8*eye(length(w_old)); 
R = 2; 

% prediction step 
y_pred = v'*w_old; 
res = y_meas - y_pred; 
P_pred = P_old + Q; 

% update step
S = v'*P_pred*v + R; 
K = P_pred*v/S;
w_upd = w_old + K*res; 
P_upd = (eye(length(w_old)) - K*v')*P_pred; 

% sample model and covariance at basis function points for debugging purposes 
mean_debug = zeros(length(acc_control_learning.bf_vx_mps), 1); 
cov_debug = zeros(length(acc_control_learning.bf_vx_mps), 1); 
% for i = 1:1:length(acc_control_learning.bf_vx_mps) 
%     dist_ay_feat = ((acc_control_learning.bf_vx_mps' - acc_control_learning.bf_vx_mps(i))/acc_control_learning.vx_width_mps).^2 + ...
%         ((acc_control_learning.bf_ay_req_mps2' - acc_control_learning.bf_ay_req_mps2(i))/acc_control_learning.ay_width_mps2).^2; 
%     ay_feat = [exp(-dist_ay_feat), acc_control_learning.bf_ay_req_mps2(i)]; 
%     mean_debug(i) = ay_feat*w_upd; 
%     cov_debug(i) = sqrt(ay_feat*P_upd*ay_feat'); 
% end


end

