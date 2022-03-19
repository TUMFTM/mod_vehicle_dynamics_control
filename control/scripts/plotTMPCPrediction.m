function handles = plotTMPCPrediction(ax, tPred, data, plot_tube, center_points, M_s, be_l, be_u, K_t, ...
    isConstrained, ub, lb, direction, isInput, TargetAvailable, target, LinearizationAvailable, lin)
% function plotStatePrediction
% Authors:      Martin Euler 
%               Alexander Wischnewski 
% Description:  
%   helper function used to plot state predictions
% Inputs/parameters:
%   tPlot: vector containing the time window for plotting [tStart, tEnd]
%   tPred: vector with the time instants corresponding to the prediction
%   data: data of real beahvior
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

% plot real signal
handles.real = plot(ax, data.Time, data.Data, 'Color', [0 0.3961 0.7412], ...
    'LineWidth', 1, 'DisplayName', 'Real');
% plot predicted signal
handles.pred = plot(ax, tPred, center_points, '*', 'Color', [0.8902, 0.4471, 0.1333], ...
    'DisplayName', 'Prediction');
% plot tube and terminal sets
if plot_tube
    % get tube bounds
    [y_low, y_up] = calcBoundsPlot(center_points, M_s, isInput, K_t, length(tPred), direction);
    % plot tube lower bounds
    handles.tube_ub = plot(ax, tPred, y_low(1,:), '-.', ...
        'Color', [0, 0, 0], 'LineWidth', 1, 'DisplayName', 'Tube');
    % plot tube upper bounds
    handles.tube_lb = plot(ax, tPred, y_up(1,:), '-.', ...
        'Color', [0, 0, 0], 'LineWidth', 1, 'HandleVisibility','off');
    % plot terminal set lower bounds and use a virtual time interval of one second (half a second
    % forth and back) to visualize it properly
    handles.terminal_ub = plot(ax, [tPred(end)-0.5, tPred(end)+0.5], be_l*ones(2, 1), ...
        'Color', [0, 0, 1],'LineWidth', 1.1, 'DisplayName', 'Terminal set'); 
    % plot terminal set upper bounds
    handles.terminal_lb = plot(ax, [tPred(end)-0.5, tPred(end)+0.5], be_u*ones(2, 1), ...
        'Color', [0, 0, 1], 'LineWidth', 1.1,'HandleVisibility','off'); 
else
end
if isConstrained
    handles.const_lb = plot(ax, tPred, ub, 'm--', ...
        'Color', [0.6275 0.1255 0.9412], 'LineWidth', 1, 'HandleVisibility', 'off');
    handles.const_ub = plot(ax, tPred, lb, 'm--', ...
        'Color', [0.6275 0.1255 0.9412], 'LineWidth', 1, 'DisplayName', 'Constraint');
end
if TargetAvailable
    handles.target = plot(ax, tPred, target, '-', 'DisplayName', 'Target');
end
if LinearizationAvailable
    handles.lin = plot(ax, tPred, lin, 'c', 'DisplayName', 'Linearization'); 
end

% adjust x limits appropriately such that it covers a certain range
xlim(ax, [tPred(1)-5, tPred(1)+10]); 
legend(ax); 

end



