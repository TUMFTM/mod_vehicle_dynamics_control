clear all; 
close all; 
load('SimTest.mat'); 
clear learnMeanDisturbances

% parameters 
nBasisFunctions = 10; 
scaleFactors = [5; 8]; 
f1 = linspace(-25, 25, nBasisFunctions); 
f2 = linspace(0, 70, nBasisFunctions);  
tPred = 1; 
tS = 0.004; 
offset = tPred/tS; 

% interval learning based upon Non-probabilistic Bayesian Update Method for Model Validation
intervals = 0.1:0.1:20; 
p_int = ones(length(intervals), 1)./length(intervals); 
likelihood = zeros(length(p_int), 1); 
escape_rate = 0.499; 
uncertainty_induction = 1e-8; 
sigma_M = 300; 

% load data 
ay_Target_mps2 = debug.debug_mvdc_path_feedback_debug_Lat_ay_Target_mps2.Data(1:50000); 
v_Target_mps = debug.debug_mvdc_path_feedback_debug_Lat_v_Target_mps.Data(1:50000); 
ay_dist_mps2 = [debug.debug_mvdc_path_feedback_debug_Lat_DistEstimate_mps2.Data(1:19999); ...
    1.*debug.debug_mvdc_path_feedback_debug_Lat_DistEstimate_mps2.Data(20000:35000); ...
    1.*debug.debug_mvdc_path_feedback_debug_Lat_DistEstimate_mps2.Data(35001:50000)];

ay_Target_mps2 = [ay_Target_mps2; ay_Target_mps2; ay_Target_mps2; ay_Target_mps2; ]; 
v_Target_mps = [v_Target_mps; v_Target_mps; v_Target_mps; v_Target_mps; ]; 
ay_dist_mps2 = [ay_dist_mps2; ay_dist_mps2; ay_dist_mps2; ay_dist_mps2; ]; 
ay_dist_mps2 = ay_dist_mps2.*linspace(1, 4, length(ay_dist_mps2))'; 
 
ay_pred_mps2 = zeros(length(ay_dist_mps2), 1); 
pred_error = zeros(length(ay_dist_mps2), 1); 
ml_interval = ones(length(ay_dist_mps2), 1).*length(intervals); 
p_int_log = zeros(length(p_int), length(ay_dist_mps2)); 

quant_est_log = zeros(length(ay_dist_mps2), 1); 
quant_est_old = 4; 
quant_est = quant_est_old; 
q_target = 0.9995; 
a_est = q_target; 
lambda_qest = 5e-3; 
gamma_qest = 0.0001*lambda_qest; 
b_est = lambda_qest*a_est; 
mu_plus = quant_est+0.5; 
mu_minus = quant_est-0.5; 

% replay data
for i = (offset+1):1:(length(ay_dist_mps2))
    % check prediction 
    [ay_pred_mps2(i), ~, ~] = learnMeanDisturbances(...,
        [ay_Target_mps2(i); v_Target_mps(i)], 0, false);
    pred_error(i) = ay_pred_mps2(i) - ay_dist_mps2(i); 
    % add some uncertainty to intervals 
    % p_int = p_int.*(1-uncertainty_induction) + sum(p_int.*uncertainty_induction)./length(p_int); 
    % update intervals
    likelihood = zeros(1, length(intervals)); 
    % uniform
    % likelihood(abs(pred_error(i))<intervals) = 1./(2*intervals(abs(pred_error(i))<intervals)); 
    % truncated gaussian (normalization is done in next step
    % idx_update = abs(pred_error(i-offset))<intervals;
    % likelihood(idx_update) = exp(-pred_error(i-offset).^2./(2.*(intervals(idx_update)).^2+sigma_M.^2))./sqrt(2.*pi.*(intervals(idx_update).^2+sigma_M.^2)); 
    % gaussian
    % likelihood = exp(-pred_error(i-offset).^2./(2.*(intervals.^2+sigma_M.^2)))./sqrt(2.*pi.*(intervals.^2+sigma_M.^2)); 
    % update interval likelihood according to bayes rule
    % p_int = likelihood'.*p_int; 
    % p_int = p_int./sum(p_int); 
    % ml_interval(i) = find(cumsum(p_int)>0.9, 1); 
    % p_int_log(:, i) = p_int;  
    % improve weights
    
    % update quantile according to "Multiplicative Update Methods for Incremental Quantile Estimation" 
    if(quant_est < abs(pred_error(i)))
        quant_est = quant_est + lambda_qest*q_target*quant_est; 
    else
        quant_est = quant_est - lambda_qest*(1-q_target)*quant_est; 
    end
    quant_est_log(i-offset) = quant_est; 
    
%     x_val = abs(pred_error(i)); 
% 
%     quant_est = (1-b_est)*quant_est + b_est*x_val; 
%     if(x_val > quant_est_old) 
%         mu_plus = quant_est - quant_est_old + (1-gamma_qest)*mu_plus + gamma_qest*x_val; 
%         mu_minus = quant_est - quant_est_old + mu_minus; 
%     else
%         mu_plus = quant_est - quant_est_old + mu_plus; 
%         mu_minus = quant_est - quant_est_old + (1-gamma_qest)*mu_minus + gamma_qest*x_val; 
%     end
%     a_est = (q_target/(mu_plus-quant_est))/(q_target/(mu_plus-quant_est)+(1-q_target)/(quant_est-mu_minus));
%     if(x_val < quant_est)
%         b_est = lambda_qest*(1-a_est); 
%     else
%         b_est = lambda_qest*(a_est); 
%     end
%     quant_est_old = quant_est; 
%     quant_est_log(i-offset) = quant_est; 
%     
    learnMeanDisturbances(...
        [ay_Target_mps2(i-offset); v_Target_mps(i-offset)], ay_dist_mps2(i-offset), true); 
    disp(['Iteration ' num2str(i)]); 
end
% obtain final model
[~, w, xBasis] = learnMeanDisturbances(...
    [0; 0], 0, false); 

% calculate predictions for all basis function points 
xTest = zeros(length(xBasis), length(xBasis)); 
for i = 1:1:length(xBasis)
    for j = 1:1:length(xBasis)
        d = vecnorm((xBasis(i, :)' - xBasis(j, :)')./scaleFactors); 
        xTest(i, j) = exp(-d^2); 
    end
end
yTest = xTest*w; 

%% visualize results 
[X, Y] = meshgrid(f1, f2); 
Z = reshape(yTest, size(X)); 
figure; hold on; grid on; 
mesh(X, Y, Z); 
scatter3(ay_Target_mps2, v_Target_mps, ay_dist_mps2); 

figure; hold on; grid on; 
plot(ay_dist_mps2); 
plot(ay_pred_mps2); 

figure; grid on; hold on; 
plot(ay_dist_mps2 - ay_pred_mps2); 
plot(quant_est_log, 'k--'); 
plot(-quant_est_log, 'k--'); 

figure; grid on; hold on; 
sz = size(p_int_log); 
for i = 1:1:sz(1)
    plot(p_int_log(i, :));
end
