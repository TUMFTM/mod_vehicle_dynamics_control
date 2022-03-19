function [prediction, alg] = learnDisturbanceMean(x_feat, disturbance, alg)
%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   12.11.2019
% Last Update:  12.03.2020
% 
% Description:  
%   learns a mean value regression model in a recursive way for a given set of basis functions
%   using a batch version of the normalized LMS algorithm (Wischnewski, 2020)
% 
% Inputs:
%   x_feat              feature vector of current point
%   disturbance         disturbance measurement of current point
%   alg                 algorithm intermediate results and parameters
% 
% Outputs: 
%   prediction          prediction for current point
%   alg                 algorithm intermediate results and parameters
% 
% Algorithm intermediate results and parameters 
%   alg.w                   Basis function weights
%   alg.gamma               Learning rate
%   alg.mu                  Normalization factor
%   alg.N                   Samples for one batch
%   alg.count               Samples already in this batch
%   alg.delta_w_temp        Stored weight update contributions of each sample in this batch
%__________________________________________________________________________
%% Algorithm

% calculate prediction
prediction = x_feat'*alg.w;
% update weight delta for this batch
res = disturbance - prediction; 
alg.delta_w_temp = alg.delta_w_temp + alg.gamma*(alg.mu*x_feat/(1 + alg.mu*(x_feat'*x_feat)))*res; 
% count new sample
alg.count = alg.count + 1; 
% if batch size is reached, update parameters
if(alg.count >= alg.N)
    alg.w = alg.w + alg.delta_w_temp; 
    alg.delta_w_temp = [0; 0]; 
    alg.count = 0; 
end