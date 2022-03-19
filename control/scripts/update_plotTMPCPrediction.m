function update_plotTMPCPrediction(ax, handles, tPred, plot_tube, center_points, M_s, be_l, be_u, K_t, ...
    isConstrained, ub, lb, direction, isInput, TargetAvailable, target, LinearizationAvailable, lin)
% function plotStatePrediction
% Authors:      Martin Euler 
%               Alexander Wischnewski 
% Description:  
%   helper function used to plot state predictions
% Inputs/parameters:
%   tPlot: vector containing the time window for plotting [tStart, tEnd]
%   tPred: vector with the time instants corresponding to the prediction
%   plot_tube: flag for tube plots
%   center_points: data for center points
%   M_s: tube shape matrices
%   be_l: lower bound of terminal set constraints
%   be_u: upper bound of terminal set constraints
%   K_t: tube controller
%   isConstrained: flag whether the signal is constrained or not
%   ub: upper limit for the signal 
%   lb: lower limit for the signal
%   isInput: set to true in case this is an input (different calculation of tubes) 
%   TargetAvailable: target trajectory available and should be plotted
%   target: actual target trajectory
%   LinearizationAvailable: linearization trajectory available and should be plotted
%   lin: actual linearization trajectory

% plot predicted signal
set(handles.pred, 'XData', tPred, 'YData', center_points);
% plot tube and terminal sets
if plot_tube
    % get tube bounds
    [y_low, y_up] = calcBoundsPlot(center_points, M_s, isInput, K_t, length(tPred), direction);
    % plot tube lower bounds
    set(handles.tube_ub, 'XData', tPred,'YData', y_low(1,:));
    % plot tube upper bounds
    set(handles.tube_lb, 'XData', tPred,'YData', y_up(1,:));
    % plot terminal set lower bounds and use a virtual time interval of one second (half a second
    % forth and back) to visualize it properly
    set(handles.terminal_lb, 'XData', [tPred(end)-0.5, tPred(end)+0.5],'YData',  be_l*ones(2, 1));
    % plot terminal set upper bounds
    set(handles.terminal_ub, 'XData', [tPred(end)-0.5, tPred(end)+0.5],'YData',  be_u*ones(2, 1));
end
if isConstrained
    set(handles.const_lb, 'XData', tPred, 'YData', ub);
    set(handles.const_ub, 'XData', tPred, 'YData', lb);
end
if TargetAvailable
    set(handles.target, 'XData', tPred, 'YData', target);
end
if LinearizationAvailable
    set(handles.lin, 'XData', tPred, 'YData', lin); 
end

% adjust x limits appropriately such that it covers a certain range
xlim(ax, [tPred(1)-3, tPred(1)+5]); 

end



