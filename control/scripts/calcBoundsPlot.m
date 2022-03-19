function [lb, ub] = calcBoundsPlot(center_points, tubeShapeMatrix, isInput, K, N, direction)
% function calcBoundsPlot
% Authors:      Martin Euler  
%               Alexander Wischnewski
%
% Description:  
%   helper function used to compute the upper and lower tube bounds
% Inputs/parameters:
%   center_points: center points of the ellipsoids
%   tubeShapeMatrix: tube shape matrices
%   isInput: flag whether data ist input or state data
%   K: tube controller
%   direction: direction on which tube should be extracted (vector)
% Outputs:
%   lb: lower bound of tube
%   ub: upper bound of tube

%% ----------------- calc tube bound centered on data -------------------- %%

% init matrices
    lb = zeros(1, N);
    ub = zeros(1, N);
    for iter = 1:1:N
        if isInput
            shapeMatrix = (K*tubeShapeMatrix(:,(iter-1)*3+1:(iter-1)*3+3)*K');
        else
            shapeMatrix = (tubeShapeMatrix(:,(iter-1)*3+1:(iter-1)*3+3));
        end
        % get upper and lower bounds
        lb(iter) = center_points(iter) - sqrt(direction(:, iter)'*shapeMatrix*direction(:, iter));
        ub(iter) = center_points(iter) + sqrt(direction(:, iter)'*shapeMatrix*direction(:, iter));
    end
end