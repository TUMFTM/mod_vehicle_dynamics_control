function [alg] = learnDisturbanceQuantile(input_signal, alg)

%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   21.11.2019
% Last Update:  12.03.2020
% 
% Description:  
%   learns a quantile estimate for the given input signal using a batch algorithm (Wischnewski, 2020)
% 
% Inputs: 
%   input_signal    Input signal used for quantile estimation
%   alg             algorithm intermediate results and parameters
%
% Algorithm intermediate results and parameters 
%   alg.r_est               Estimate for the q-quantile
%   alg.q_target            Target percentage of the quantile
%   alg.lambda              Learning rate
%   alg.N                   Samples for one batch
%   alg.count_lower         Count of samples in this batch lower than the quantile estimate
%   alg.count               Count of samples in this batch
%__________________________________________________________________________
%% Algorithm

% if the input signal is smaller than the current quantile estimate, 
% increase counter of signals lower than quantile estimate. This is used to estimate the 
% percentage of the data covered by the estimate. 
if(input_signal < alg.r_est)
    alg.count_lower = alg.count_lower + 1; 
end
alg.count = alg.count + 1; 
% 
% if the batch size is reached, update the estimate
if(alg.count >= alg.N)
    alg.r_est = alg.r_est - alg.lambda*(alg.count_lower/alg.count - alg.q_target); 
    alg.count_lower = 0; 
    alg.count = 0; 
end

