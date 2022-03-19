function [acc_control_learning] = setupAccControlLearning(dataset)
%
% Authors:       Alexander Wischnewski
%
% Description:  
%   function used to prepare the variables for the acceleration controller learning algorithms
% 
% Input: 
%   dataset: Log file with weights used for initialization (if given)


% spread of speed values
acc_control_learning.spread_vx_mps = 10:15:100; 
acc_control_learning.spread_ay_req_mps2 = -36:6:36;

% design of basis functions
acc_control_learning.vx_width_mps = 20; 
acc_control_learning.ay_width_mps2 = 8; 

[VX, AY] = meshgrid(acc_control_learning.spread_vx_mps, acc_control_learning.spread_ay_req_mps2); 
acc_control_learning.bf_vx_mps = reshape(VX, numel(VX), 1); 
acc_control_learning.bf_ay_req_mps2 = reshape(AY, numel(AY), 1); 

% initial weights for lateral acceleration controller learning
% will be updated below if dataset is given
acc_control_learning.w0 = zeros(length(acc_control_learning.bf_vx_mps)+1, 1); 

% analyze dataset if available
if(nargin==1) 
    % extract weights from log and write to initial weights
    data = load(dataset); 
    weights = data.debug_slow.debug_slow_LatAcc_FFweights.Data(end, :); 
    acc_control_learning.w0 = weights'; 
    % calculate learned feedforward part for the end of the log
    visz_vx_mps = 0:5:100;
    visz_ay_req_mps2 = -40:0.5:40; 
    [Visz_X, Visz_Y] = meshgrid(visz_vx_mps, visz_ay_req_mps2); 
    Visz_Z = zeros(length(visz_ay_req_mps2), length(visz_vx_mps)); 
    for i = 1:1:length(visz_vx_mps)
        for j = 1:1:length(visz_ay_req_mps2)
            % apply basis functions to each sample point
            dist = ((acc_control_learning.bf_vx_mps - visz_vx_mps(i))./acc_control_learning.vx_width_mps).^2 + ...
                           ((acc_control_learning.bf_ay_req_mps2 - visz_ay_req_mps2(j))./acc_control_learning.ay_width_mps2).^2;
            Visz_Z(j, i) = weights*[exp(-dist); visz_ay_req_mps2(j)]; 
        end
    end
    figure; 
    surface(Visz_X, Visz_Y, Visz_Z); 
    xlabel('Velocity in mps'); 
    ylabel('Lateral acc. target in mps2'); 
    zlabel('Corrective steering angle in rad'); 
end