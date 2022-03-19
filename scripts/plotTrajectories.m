function plotTrajectories(data, IDstart, IDend)
%
% Authors:       Alexander Wischnewski
%
% Description:  
%   used to visualize the timely behavior of new target trajectories
% 
% Inputs: 
%   IDstart:    Trajectory ID start (see debug_slow.debug_slow_tartraj_TrajCnt)
%   IDend:      Trajectory ID end (see debug_slow.debug_slow_tartraj_TrajCnt)
    
    % colors 
    C = {'k','b','r','g','y',[.5 .6 .7],[.8 .2 .6],[.3 .1 .2],[.1 .5 .9]}; % Cell array of colros.
    figure; 
    xy_fig = gca; 
    hold on; grid on; 
    xlabel('x East in m'); 
    ylabel('y North in m'); 
    axis equal; 
    figure; 
    kappa_fig = gca; 
    hold on; grid on; 
    xlabel('Global s-coordinate in m'); 
    ylabel('Kappa in radpm'); 
    figure; 
    v_fig = gca; 
    hold on; grid on; 
    xlabel('Global s-coordinate in m'); 
    ylabel('Velocity in mps'); 
    for i = IDstart:1:IDend
        % find idx with trajectory
        idx = find(data.debug_slow.debug_slow_tartraj_TrajCnt.Data==i, 1, 'first');
        disp(idx); 
        % map color index to available colors
        color_idx = mod(i-IDstart, length(C)-1) + 1;
        plot(xy_fig, data.debug_slow.debug_slow_tartraj_x_m.Data(idx, :), ...
            data.debug_slow.debug_slow_tartraj_y_m.Data(idx, :), 'color', C{color_idx}); 
        scatter(xy_fig, data.debug_slow.debug_slow_x_real_m.Data(idx), ...
            data.debug_slow.debug_slow_y_real_m.Data(idx), 30, 'filled', ...
            'MarkerEdgeColor', C{color_idx}, 'MarkerFaceColor', C{color_idx}); 
        plot(kappa_fig, data.debug_slow.debug_slow_tartraj_s_glob_m.Data(idx, :), ...
            data.debug_slow.debug_slow_tartraj_kappa_radpm.Data(idx, :)); 
        plot(v_fig, data.debug_slow.debug_slow_tartraj_s_glob_m.Data(idx, :), ...
            data.debug_slow.debug_slow_tartraj_v_mps.Data(idx, :)); 
    end
end