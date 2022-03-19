function [] = convertDebugToTrajectory(debug, b_plot)

% author: Thomas Herrmann <thomas.herrmann@tum.de>
% input: debug-struct

% Author: Thomas Herrmann <thomas.herrmann@tum.de>     Date: 16-10-2020
%
% Description:
%   converts logging signals from the local trajectory planner into a data
%   dictionary, which can be used for the purpose of, e.g., trajectory
%   replay for the controller development. The algorithm assumes constant
%   acceleration values. It keeps varying acceleration values
%   (measured from point to point) and removes constant values, which are
%   unnecessary to remove the data size.
%
%   KEEP_FRACTION can be increased to remove more unnecessary data points.
%   V_MIN is a threshold that specifies the minimum velocity in the
%   filtered trajectory that will be saved in a new data dictionary
%
% Input:
%   debug:              Logged signals in debug struct
%   b_plot:             Activates plots of raw and filtered signals

KEEP_FRACTION = 10;
V_MIN = 5; % mps

%% Algorithm
t = debug.debug_Time_s.Data;

%% True vehicle state
x = debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m.Data;
y = debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m.Data;
psi = debug.debug_mvdc_state_estimation_debug_StateEstimate_psi_vel_rad.Data;
kappa = debug.debug_mvdc_state_estimation_debug_StateEstimate_kappa_radpm.Data;
v = debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data;
ax = debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data;

% Calcualte distance s
s = [0; cumsum(sqrt((x(2:end) - x(1:end - 1)).^2 + ...
    (y(2:end) - y(1:end - 1)).^2))];

% figure; plot(ax); hold on;

%% Planned vehicle state
x_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_x_m.Data;
y_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_y_m.Data;
psi_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_psi_rad.Data;
kappa_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_kappa_radpm.Data;
v_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_v_mps.Data;
ax_pl = debug.debug_mvdc_path_matching_debug_ActualTrajPoint_ax_mps2.Data;

% find first true x,y_pl-values that are different from int(0)
for i = 1:length(x_pl)
    if x_pl(i) == 0
        continue
    else
        x_pl_min = x_pl(i);
        y_pl_min = y_pl(i);
        psi_pl_min = psi_pl(i);
        break
    end
end
x_pl(x_pl == 0) = x_pl_min;
y_pl(y_pl == 0) = y_pl_min;
psi_pl(psi_pl == 0) = psi_pl_min;

% Calcualte distance s_planned
s_pl = [0; cumsum(sqrt((x_pl(2:end) - x_pl(1:end - 1)).^2 + ...
    (y_pl(2:end) - y_pl(1:end - 1)).^2))];

% figure; plot(ax_pl); grid on;

%% Extract changes in constant ax_planned values
ax_switching = [ax_pl(2:end) ~= ax_pl(1:end - 1)];
ax_switching_use = ax_switching;
% add entries to extract necessary ax values
i = 0;
for i=1:length(ax_switching) - 1
    % write 1 to subsequent entry of changing constant ax-value
    if ax_switching(i) == 1
        ax_switching_use(i + 1) = 1;
    end
end
% plot(ax_switching_use);

%% extract every j-th point specified by KEEP_FRACTION on constant ax-values
j = 0;
for i = 1:length(ax_switching_use)
    if ax_switching_use(i) == 0
        j = j + 1;
        if j == KEEP_FRACTION
            % save idx of entry to be kept
            ax_switching_use(i) = 1;
            % reset j
            j = 0;
        end
    end
end
% plot(ax_switching_use, '-x')

%% Filter signals according to determined filter vector
s_pl_filter = s_pl(ax_switching_use);

x_pl_filter = x_pl(ax_switching_use);
y_pl_filter = y_pl(ax_switching_use);
psi_pl_filter = psi_pl(ax_switching_use);
kappa_pl_filter = kappa_pl(ax_switching_use);
v_pl_filter = v_pl(ax_switching_use);
ax_pl_filter = ax_pl(ax_switching_use);

if b_plot
    figure; hold on; grid on;
    plot(s_pl, ax_pl);
    plot(s_pl_filter, ax_pl_filter, 'x');
    xlabel('s in m'); ylabel('ax in mps2');
    legend('planned', 'filtered');

%     figure; hold on; grid on;
%     plot(s_pl, v_pl);
%     plot(s_pl_filter, v_pl_filter, 'x');
%     xlabel('s in m'); ylabel('v in mps');
%     legend('planned', 'filtered');

end
disp(['removed ', ...
    num2str(100 * (1 - length(ax_pl_filter)/length(ax_pl))), ...
    ' % of unnecessary entries in planned trajectory!'])

%% Increase v to avoid <= 3 mps as minimum velocity
for i = 1:length(v_pl_filter)
    if v_pl_filter(i) < V_MIN
        % determine factor of which v was increased
        accel_corr = V_MIN / v_pl_filter(i);
        % set v to V_MIN
        v_pl_filter(i) = V_MIN;
        % decrease ax by squared correction factor to keep combined
        % acceleration limit in modified trajectory
        if accel_corr == inf
            % if v_pl == 0, keep planned ax
            ax_pl_filter(i) = ax_pl_filter(i);
        else
            % if v_pl was modified, modify ax
            ax_pl_filter(i) = ax_pl_filter(i) / (accel_corr ^ 2);
        end
    end
end

% check combined acceleration after V_MIN-Manipulation
accel_check = [abs(kappa_pl_filter .* v_pl_filter.^2) / 12 + ...
    abs(ax_pl_filter) / 12 <= 1];
if sum(accel_check == 0) >= 1
    disp('Warning! Soft acceleration check on trajectory failed!');
end

%% Write output

% get unique entries in global s coordinate with previous order of entries
[~, idx_unique] = unique(s_pl_filter, 'stable');

if b_plot
    plot(s_pl_filter(idx_unique), ax_pl_filter(idx_unique), 'o');
    legend('planned', 'filtered', 's-unique');
end

M = [s_pl_filter(idx_unique), ...
    x_pl_filter(idx_unique), ...
    y_pl_filter(idx_unique), ...
    psi_pl_filter(idx_unique), ...
    kappa_pl_filter(idx_unique), ...
    v_pl_filter(idx_unique), ...
    ax_pl_filter(idx_unique)];

if length(M(:, 1)) >= 5000
    disp('Warning! Created data dictionary may be corrupted as you wrote too much content into it. Increase parameter KEEP_FRACTION!');
end

% open a file for writing
fid = fopen('racelines/filteredTrajectory.csv', 'w');
% print a title, followed by a blank line
fprintf(fid, '# This is a trajectroy created from recorded\n# planned local trajectories during a test session \n# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2\n');
% print values in column order
fprintf(fid, '%f;%f;%f;%f;%f;%f;%f\n', M');
fclose(fid);

%% Call script to create new data dictionary from previously created .csv
loadRaceline('racelines/filteredTrajectory', 'filteredTrajectory')

end
