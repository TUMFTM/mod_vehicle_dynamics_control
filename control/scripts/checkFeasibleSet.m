function [qp_feas_ns, qp_feas] = checkFeasibleSet(prob, errorState, f, lb, ub, P_x, A_x, sys, ...
    use_warmstart, x_warm_sim, y_warm_sim, v_part_only)
% Authors:
%       Salih Guemues
% Description:
%       evaluates for different initial states, if constraints and terminal
%       set is well proposed, i.e. if problem is feasible. Thus, the
%       analysis returns the feasible region of the proposed constraints.
%       - Plots the solver status to visualize which cases are infeasible
%       - Plots detected feasible initial states (depending on grids)
% Inputs:
%   prob:           OSQP-object
%   errorState:     the errorState of the recovered time-step
%   f:              linear cost function term of recovered QP
%   lb:             lower bound of recovered constraints
%   ub:             upper bound of recovered constraints
%   P_x:            nonzero values of Hessian matrix of recovered QP
%   A_x:            nonzero values of constraint matrix of recovered QP
%   sys:            structure including problem data
%   use_warmstart:  flag for using manual warm starting of QP
%   x_warm_sim:     recovered primal solution manual warm starting
%   y_warm_sim:     recovered dual solution manual warm starting
%   v_part_only:    flag for analysing only selected velocity offset values
%
% Outputs:
%   qp_feas_ns:     initial states leading to feasible solutions, however
%                   not safe (_ns) that it is feasible since solver status is >-3
%   qp_feas:        initial states leading to feasible solutions, solver
%                   status 1 or 2

%% ------------------------------- %%
% parameter for griding the initial state space
v_grid = 0.2;
d_grid = 0.1;
d_dot_grid = 0.1;
v_diff_max = 2;
d_diff_max = 1;
d_dot_diff_max = 1;
% update OSQP-settings to allow for more iterations --> more accurate
% infeasbility detection:
prob.update_settings('max_iter', 4000);
%% ------------------------- manipulate current error state ------------------------------- %%
% current state vector for controller (state vector of error diff. equat.)
% errorState = [delta_vx;d;d_dot];
% for plot of only some parts of space with regard to delta_v 
% SPECIFY EXACTLY 6 v_diff values so that 6 subplots can be created
delta_v_part = [-v_diff_max, -4*v_grid, -2*v_grid, 0, 2*v_grid, v_diff_max];

% state space grid 
i = 1;  
for d = -d_diff_max:d_grid:d_diff_max 
    for dot_d = -d_dot_diff_max:d_dot_grid:d_dot_diff_max 
        if v_part_only
            for k=1:1:length(delta_v_part)
                error_grid(:, i) = [delta_v_part(1,k); d; dot_d];  
                i = i + 1;
            end
        else
            for delta_v = -v_diff_max:v_grid:v_diff_max 
                error_grid(:, i) = [delta_v; d; dot_d];  
                i = i + 1;  
            end 
        end
    end 
end
% variables for storing results
qp_feas_ns = zeros(length(error_grid(1,:)),1);
qp_feas = zeros(length(error_grid(1,:)),1);
status_feas = zeros(length(error_grid(1,:)),1);
for i=1:1:length(error_grid(1,:))
    x_recover = errorState - error_grid(:, i); % apply offset
    if sys.use_man_precon || sys.use_own_man_precon
        % get preconditioning matrices from sys-struct (only diagonal elements are
        % stored, i.e. transform to diagonal matrix to perform Mat-Vec-Multipl.
        precon_E = diag(sys.precon_E);
        % unscale data to manipulate bounds of initial state constraint
        lb_rec = lb./diag(precon_E);
        ub_rec = ub./diag(precon_E);
    end
    %% -------------------- change initial state equality constraints ------------------------ %%
    % set equality constraint to ensure p0 = x_t
    ub_rec(3*sys.N_hor+1:3*sys.N_hor+3) = x_recover;
    lb_rec(3*sys.N_hor+1:3*sys.N_hor+3) = x_recover;
    %% ------------ perform preconditioning of QP data to reduce condition number ---------------%
    if sys.use_man_precon || sys.use_own_man_precon
        % since bounds may have been initialized with 'inf', Mat-Vec-product
        % may lead to 'NaN': [1 0;0 1]*[inf;inf] = [NaN; NaN] (0*inf = NaN).
        % Thus, use elementwise multiplication (E is a diagonal matrix)
        lb_rec      = diag(precon_E).*lb_rec;
        ub_rec      = diag(precon_E).*ub_rec;
    end
    %% ----------------------------------------------------%%
    % update QP vectors
    prob.update('q', f, 'l', lb_rec, 'u', ub_rec);
    % update QP matrices: if P_update is activated, update both, else only A_x
    if sys.trigger_Pupdate
        prob.update('Px', P_x, 'Ax', A_x);
    else
        prob.update('Ax', A_x);
    end
    if use_warmstart
        % warmstart QP with QP solution of previous time step (one before recovery)
        prob.warm_start('x', x_warm_sim, 'y', y_warm_sim)
    end
    % Solve problem
    res = prob.solve();
    status = res.info.status_val;
    %% ---------------check if solution is feasible---------------%%
    if status == 1 || status == 2 || status == -2  % not safe that optimal solution is feasible for the specified constraints, i.e. max_iter reached included
        qp_feas_ns(i,1) = 1;
    end
    if status == 1 || status == 2  % optimal solution is feasible for the specified constraints
        qp_feas(i,1) = 1;
    end
    status_feas(i,1) = status;
end
% filter feasible states
idx_qp_feas_ns = find(qp_feas_ns ==1);
idx_qp_feas = find(qp_feas ==1);
x_help = errorState - error_grid;
x_feas_ns = x_help(:, idx_qp_feas_ns);
x_feas = x_help(:, idx_qp_feas);

% mark cornerpoints of discretized state space
corner_x = [errorState(1,1) + v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) + v_diff_max, errorState(1,1) + v_diff_max, ...
            errorState(1,1) + v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) + v_diff_max, errorState(1,1) + v_diff_max, ...
            errorState(1,1) + v_diff_max, errorState(1,1) + v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) - v_diff_max, errorState(1,1) - v_diff_max, ...
            errorState(1,1) - v_diff_max];
corner_y = [errorState(2,1) - d_diff_max, errorState(2,1) - d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) - d_diff_max, ...
            errorState(2,1) - d_diff_max, errorState(2,1) - d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) - d_diff_max, ...
            errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) - d_diff_max, ...
            errorState(2,1) - d_diff_max];
corner_z = [errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max, ...
            errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, ...
            errorState(3,1) + d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, ...
            errorState(3,1) - d_dot_diff_max];
% plot results
figure;
box on; grid on; hold on;
plot(qp_feas_ns)
plot(status_feas, 'Marker', 'o')
legend('Feasible true/false', 'Solver status','interpreter','latex','FontName','Times New Roman','FontSize', 10)
xlabel('Grid point','interpreter','latex','FontName','Times New Roman','FontSize', 10)
ylabel('Status','interpreter','latex','FontName','Times New Roman','FontSize', 10)
set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);

% plot 3D-figure of gridded state space
figure;
box on; grid on; hold on;
scatter3(x_feas_ns(1, :), x_feas_ns(2, :), x_feas_ns(3, :), 'b', 'filled');
scatter3(x_feas(1, :), x_feas(2, :), x_feas(3, :), 'g', 'filled');
scatter3(errorState(1, 1), errorState(2, 1), errorState(3, 1), 'r');
view(40,35);
plot3(corner_x, corner_y, corner_z,'r-');
legend('Status is 1 or 2 or -2', 'Status is 1 or 2','recovered QP-state', 'Analysed space','interpreter','latex','FontName','Times New Roman','FontSize', 10)
xlabel('Velocity error in mps');
ylabel('Lateral error in m'); 
zlabel('Lateral speed error in mps');  

% plot only exemplarily selected slices of whole gridded state space
figure;
if length(delta_v_part)~=6
    disp('Mismatch in number of velocities for partial grid-space plot. Specify 6 values.');
end
corner_y = [errorState(2,1) - d_diff_max, errorState(2,1) - d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) + d_diff_max, errorState(2,1) - d_diff_max];
corner_z = [errorState(3,1) - d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) + d_dot_diff_max, errorState(3,1) - d_dot_diff_max, errorState(3,1) - d_dot_diff_max];
for k=1:1:length(delta_v_part)
    % filter feasible states for partial plot
    idx_qp_feas_ns_part = find(x_feas_ns(1,:) == (errorState(1,1) - delta_v_part(1,k)));
    idx_qp_feas_part = find(x_feas(1,:) == (errorState(1,1) - delta_v_part(1,k)));
    x_feas_ns_part = x_feas_ns(:, idx_qp_feas_ns_part);
    x_feas_part = x_feas(:, idx_qp_feas_part);
    % create and fill subplot
    subplot(2, 3, k);
    box on; grid on; hold on;
    scatter(x_feas_ns_part(2, :), x_feas_ns_part(3, :), 'b', 'filled');
    scatter(x_feas_part(2, :), x_feas_part(3, :), 'g', 'filled');
    scatter(errorState(2, 1), errorState(3, 1), 'r');
    plot(corner_y, corner_z,'r-');
    if k==length(delta_v_part) % show legend only once in the last subplot
        lgd = legend('Status is 1 or 2 or -2', 'Status is 1 or 2','Recovered QP-state', 'Analysed space','interpreter','latex','FontName','Times New Roman','FontSize', 10, 'Location', 'northeast');
        lgd.NumColumns = 2;
    end
    xlabel('Lateral error in m');
    ylabel('Lateral speed error in mps');
    if delta_v_part(1,k) == 0 % mark the reference plot, i.e. with the velocity of actual QP
        title(['$\Delta v_\mathrm{x}$: ',num2str(errorState(1,1) - delta_v_part(1,k)),' mps (actual QP-state)'], 'interpreter', 'latex');
    else
        title(['$\Delta v_\mathrm{x}$: ',num2str(errorState(1,1) - delta_v_part(1,k)),' mps'], 'interpreter', 'latex');
    end
end
end

