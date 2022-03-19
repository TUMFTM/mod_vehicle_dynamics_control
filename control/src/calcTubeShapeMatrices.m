function [M_s] = calcTubeShapeMatrices(A, B, K, M_1, M_vec, N)
% function calcTubeShapeMatrices
% Author:       Martin Euler         last update: 05-03-2020
% Description:  
%   function used to calculate the tube shape matrices
% Inputs/parameters:
%   A: system dynamic matrix
%   B: system input matrix
%   K: stabilising tube controller
%   M_1: shape matrix for initial set
%   M_vec: sqrt of diagonal entries of shape matrix of disturbance set
%   max_iter: number of max iterations for calculation of the terminal set
%   N: length of prediction horizon

% Outputs:
%   M_s: tube shape matrices over the planning horizon
%% -------------------- calculate shape matrices ----------------------- %%
% init vector of shape matrices
M_s = zeros(3, 3*N);
% define shape matrix dynamic
M = @(Ms) (A + B*K)*Ms*(A + B*K)';
% define minkowski sum
M_sum = @(M1,M2) (1 + sqrt(trace(M2)/max(1e-7,trace(M1))))*M1+...
          (1 + sqrt(trace(M1)/max(1e-7,trace(M2))))*M2;
% get initial tube size
Ms = M_1;
% set initial tube size
M_s(1:3, 1:3) = Ms;
for i = 2:N
    % calculate disturbance shape matrix 
    M_tilde = B*diag([M_vec(1, i-1)^2, M_vec(2, i-1)^2])*B'; 
    Ms = M_sum(M(Ms),M_tilde);
    M_s(:,(i-1)*3+1:(i-1)*3+3) = Ms;            
end    

end

