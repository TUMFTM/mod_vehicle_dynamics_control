function plotTMPC(data, tmpc_cnt, tPlot, mode)
% function plotTMPC
% Authors:      Martin Euler         
%               Salih G�m�s
%               Alexander Wischnewski
%
% Description:  
%   function used to plot MPC and TMPC prediction and tubes
% How to: 
%   Run simulation, save data via convertSimLogs(logsout, 'SimTest.mat'); and load data via 
%   data = load('SimTest.mat');. Now call this script. 
% Inputs/parameters:
%   data:           Log file struct
%   tmpc_cnt:       ID of the optimization problem which is plotted (debug_slow.tmpc_cnt)
%   tPlot:          Start and end time for which the signals are plotted [tStart, tEnd]
%   mode:           determines how the parameters for the resimulation are calculated, 
%                   either 'script' or 'DD'
% Outputs:
%   Plots for different variables

%% ------------------ define plotting configuration  --------------------- %%
plot_tube = true;           % define if tubes should be plotted (global setting) 
plot_e1 = true;             % errorState_1 delta v_x
plot_e2 = true;             % errorState_2 d_y
plot_e3 = true;             % errorState_3 delta v_y
plot_ax = true;             % longitudinal acceleration ax
plot_ay = true;             % lateral acceleration ay
plot_diamond = false;       % combined acceleration constraints
plot_solver_status = false; % solver status
plot_solver_time = false;   % solver runtime
plot_solver_iter = false;   % solver iterations
plot_vx = false;            % absolute velocity
plot_L2_predError = false;   % L2 norm of prediction errors
plot_L2_diffError = false;   % L2-Norm of difference of two consecutive predictions
plot_L2_QPSol = false;       % L2-Norm of QP-Solution
plot_slack_a = false;        % acceleration slack values
plot_slack_d = false;        % lateral error slack values
plot_cost = true;          % cost function values
plot_dualnorm = false;      % dual norm of lagrange multiplier (dual with respect to slack costs)

%% ------------------- prepare plotting ---------------------------------- %%
[sys, P_VDC_VariableTubeSize, P_VDC_UseLearnedPrediction, P_VDC_UseShiftedTerminalSet, ...
    P_VDC_PredictionSafetyFactor, P_VDC_DyLim, P_VDC_AxLim, P_VDC_AyLim, dist, ...
    errorState_1, errorState_2, errorState_3, tPred, dist_mean, p_t_1, p_t_2, p_t_3, j_x, j_u, ...
    j_rax, j_ray, lambda_dualnorm, u_t_norm_ax, u_t_norm_ay, slack_a1_low_pred, slack_a1_up_pred, ...
    slack_a2_low_pred, slack_a2_up_pred, slack_d_low_pred, slack_d_up_pred, slack_a1_low, ...
    slack_a1_up, slack_a2_low, slack_a2_up, slack_d_low, slack_d_up, j_s_lin, j_s_quad, j_s, ...
    M_s, be_u, be_l] = prepareTMPCPlotting(mode, data, tmpc_cnt); 

%% ----------------------- get L2-Norm of prediciton error -------------- %%
[L2_ax, L2_ay, L2_e1, L2_e2, L2_e3, e1_broken_con, e2_broken_con, e3_broken_con, ax_broken_con,...
    ay_broken_con, e1_diff_norm, e2_diff_norm, e3_diff_norm, ax_diff_norm, ay_diff_norm] = ...
    getL2PredError(data, errorState_1, errorState_2, errorState_3, ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2, ...
    data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2, ...
    P_VDC_UseLearnedPrediction, sys, tPred);

%% ------------------------ Plot e1 ------------------------------------ %%    
if plot_e1 
    plotTMPCPrediction(tPlot, tPred, errorState_1, plot_tube, p_t_1, M_s, be_l(1), be_u(1), ...
        sys.K_t, false, inf, 1, false); 
    xlabel('$t$ in s', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    ylabel('$\Delta v_x$ in $\mathrm{ms^{-1}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman','FontSize', 10); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% --------------------------------- Plot e2 ---------------------------- %%
if plot_e2
    plotTMPCPrediction(tPlot, tPred, errorState_2, plot_tube, p_t_2, M_s, be_l(2), be_u(2), ...
        sys.K_t, true, P_VDC_DyLim, 2, false)
    if plot_tube
        legend('Real trajectory', 'Prediction', 'Tube', 'Terminal constraints', ...
            '$d_{y,\mathrm{Lim}}$', 'interpreter', 'latex', ...
            'FontName', 'Times New Roman', 'FontSize', 10); 
    else
        legend('Real trajectory', 'Prediction', '$d_{y,\mathrm{Lim}}$', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    end
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);     
    ylabel('$d_y$ in m ', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end
%% -------------------------------- Plot e3 ---------------------------- %%
if plot_e3
    plotTMPCPrediction(tPlot, tPred, errorState_3, plot_tube, p_t_3, M_s, be_l(3), be_u(3), ...
        sys.K_t, false, inf, 3, false);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('$\dot{d_y}$ in $\mathrm{ms^{-1}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% --------------------------------- Plot ax --------------------------- %%
if plot_ax
    plotTMPCPrediction(tPlot, tPred(1:end-1), ...
        data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2, ...
        plot_tube, data.debug_slow.debug_slow_ax_pred_mps2.Data(tmpc_cnt, :)', ...
        M_s, 0, 0, sys.K_t, true, P_VDC_AxLim, 1, true); 
    if plot_tube
        legend('Real trajectory', 'Prediction', 'Tube', 'Terminal constraints', ...
            '$a_{x,\mathrm{Lim}}$', 'interpreter', 'latex', ...
            'FontName', 'Times New Roman', 'FontSize', 10); 
    else
        legend('Real trajectory', 'Prediction', '$a_{x,\mathrm{Lim}}$', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    end
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('$a_x$ in $\mathrm{ms^{-2}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ----------------------------- Plot ay --------------------------------%%
if plot_ay
    plotTMPCPrediction(tPlot, tPred(1:end-1), ...
        data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2, ...
        plot_tube, data.debug_slow.debug_slow_ay_pred_mps2.Data(tmpc_cnt, :)', ...
        M_s, 0, 0, sys.K_t, true, P_VDC_AyLim, 2, true);
    if plot_tube
        legend('Real trajectory', 'Prediction', 'Tube', 'Terminal constraints', ...
            '$a_{y,\mathrm{Lim}}$', 'interpreter', 'latex', ...
            'FontName', 'Times New Roman', 'FontSize', 10); 
    else
        legend('Real trajectory', 'Prediction', '$a_{y,\mathrm{Lim}}$', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    end
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    ylabel('$a_y$ in $\mathrm{ms^{-2}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ----------------------------- Plot diamond (ax over ay) --------------------------------%%
if plot_diamond
    figure; box on; grid on; hold on; 
    % plot real accelerations (needs to be reworked to be restricted to tPlot)
    plot(data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data, ...
        data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data, ...
        'Color',[0 0.3961 0.7412],'LineWidth', 1);
    plot(data.debug_slow.debug_slow_ay_pred_mps2.Data(tmpc_cnt, :), ...
        data.debug_slow.debug_slow_ay_pred_mps2.Data(tmpc_cnt, :), ...
        '*', 'Color', [0.8902, 0.4471, 0.1333],'LineWidth', 1);
    plot([0 P_VDC_AyLim 0 -P_VDC_AyLim 0], [P_VDC_AxLim 0 -P_VDC_AxLim 0 P_VDC_AxLim], ...
        'm--','Color',[0.6275 0.1255 0.9412],'LineWidth', 1);
    
    % plot tube
    if plot_tube
        % get tube bounds
        [ax_low, ax_up] = calcBoundsPlot(...
            data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ax_mps2.Data, ...
            M_s, true, sys.K_t, sys.N_hor, 1);
        [ay_low, ay_up] = calcBoundsPlot(...
            data.debug.debug_mvdc_state_estimation_debug_StateEstimate_ay_mps2.Data, ...
            M_s, true, sys.K_t, sys.N_hor, 2);
        % plot lower tube bound
        plot(ay_low(1,:), ax_low(1,:), '-.', ...
            'Color', [0, 0, 0], 'LineWidth', 1);
        % plot upper tube bound
        plot(ay_up(1,:), ax_up(1,:), '-.', ...
            'Color', [0, 0, 0], 'LineWidth', 1, 'HandleVisibility','off');    
        legend('Real Trajectory', 'Prediction', 'Limits', 'Tube', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    else
        legend('Real Trajectory', 'Prediction', 'Limits', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);     
    end
    xlabel('$a_y$ in $\mathrm{ms^{-2}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    ylabel('$a_x$ in $\mathrm{ms^{-2}}$ ', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ----------------------- Plot solver status -------------------------- %%
if plot_solver_status
    figure; box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_solver_state, 'Color',[0 0.3961 0.7412], 'LineWidth', 1);
    xlabel('$t$ in s','interpreter','latex','FontName','Times New Roman','FontSize', 10);
    ylabel('Solver state','interpreter','latex','FontName','Times New Roman','FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot solver time ----------------------------- %%
if plot_solver_time
    figure; box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_solver_runTime_s, ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
    plot(1e-4*data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps, ...
        'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
    plot([tPlot(1), tPlot(2)], [1 1]*mean(data.debug_slow.debug_slow_solver_runTime_s.Data), ...
        'Color', 'r', 'LineWidth', 1); 
    legend('Computation time', '$v_{\mathrm{Traj}}\cdot10^{-4}$ in $\mathrm{ms^{-1}}$', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Computation time in s', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot solver iterations  ----------------------------- %%
if plot_solver_iter
    figure; box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_solver_iteration, 'Color',[0 0.3961 0.7412], 'LineWidth', 1); 
    legend('Iterations', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Iterations', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot slack variables for acceleration limits -------------------------- %%
if plot_slack_a && sys.ns_total > 0
    figure; box on; grid on; hold on; 
    if sys.use_mpcqpslack
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_a1_up, ...
        'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
        plot(tPred, slack_a1_up_pred(:,1),'*','Color', [0.8902, 0.4471, 0.1333],'LineWidth', 1);
        legend('Slack $a_{\mathrm{up}}$', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    else
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_a1_low, ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_a1_up, ...
            'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_a2_low, ...
            'Color', [0 0.8 0], 'LineWidth', 1);
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_a2_up, ...
            'Color', [0.8, 0, 0], 'LineWidth', 1);
        plot(tPred, slack_a1_low_pred(:,1),'*','Color',[0 0.3961 0.7412],'LineWidth', 1);
        plot(tPred, slack_a1_up_pred(:,1),'*','Color', [0.8902, 0.4471, 0.1333],'LineWidth', 1);
        plot(tPred, slack_a2_low_pred(:,1),'*','Color',[0 0.8 0],'LineWidth', 1);
        plot(tPred, slack_a2_up_pred(:,1),'*','Color', [0.8, 0, 0],'LineWidth', 1);
        legend('Slack $a1_{\mathrm{low}}$', 'Slack $a1_{\mathrm{up}}$', ...
            'Slack $a2_{\mathrm{low}}$','Slack $a2_{\mathrm{up}}$', ...
            'interpreter','latex','FontName','Times New Roman','FontSize', 10);
    end
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Slack value', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot slack variables for lateral error limits ------------------------- %%
if plot_slack_d && sys.ns_total > 0
    figure; box on; grid on; hold on; 
    if sys.use_mpcqpslack
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_d_up, ...
            'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
        plot(tPred, slack_d_up_pred(:,1),'*','Color', [0.8902, 0.4471, 0.1333],'LineWidth', 1);
        legend('Slack $d_{\mathrm{up}}$','interpreter','latex', ...
            'FontName','Times New Roman','FontSize', 10);
    else
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_d_low, ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
        plot(data.debug_slow.debug_slow_u_opt_total.Time, slack_d_up, ...
            'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
        plot(tPred, slack_d_low_pred(:,1),'*','Color',[0 0.3961 0.7412],'LineWidth', 1);
        plot(tPred, slack_d_up_pred(:,1),'*','Color', [0.8902, 0.4471, 0.1333],'LineWidth', 1);
        legend('Slack $d_{\mathrm{low}}$','Slack $d_{\mathrm{up}}$','interpreter','latex', ...
            'FontName','Times New Roman','FontSize', 10);
    end
    xlabel('$t$ in s','interpreter','latex','FontName','Times New Roman','FontSize', 10);
    ylabel('Slack value','interpreter','latex','FontName','Times New Roman','FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot cost function terms  ----------------------------- %%
if plot_cost
    % all cost are capped to ensure reliable plotting
    figure; 
    semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_rax + j_ray, 1e-2), ...
        'Color', [0 0.4 0.4], 'LineWidth', 1);
    hold on; grid on; box on; 
    semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_x, 1e-2), ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
    semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_u, 1e-2), ...
        'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
    if sys.ns_total > 0
        semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_s_lin, 1e-2), ...
            'Color',[0.6275 0.1255 0.9412],'LineWidth', 1);
        semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_s_quad, 1e-2), ...
            'Color',[0 0.8 0.4],'LineWidth', 1);
        semilogy(data.debug_slow.debug_slow_u_opt_total.Time, max(j_s, 1e-2), ...
            'Color', [0 0 0],'LineWidth', 1);
        legend('Summed reg. cost $j_\mathrm{ax}$ and $j_\mathrm{ay}$', ...
            'State cost $j_\mathrm{x}$', 'Input cost $j_\mathrm{u}$', ...
            'Lin. slack cost $j_\mathrm{s, lin}$', 'Quadr. slack cost $j_\mathrm{s, quad}$', ...
            'Total slack cost $j_\mathrm{s}$', 'interpreter', 'latex', ...
            'FontName', 'Times New Roman', 'FontSize', 10);
    else
        legend('State cost $j_\mathrm{x} \cdot10^{-2}$', 'Input cost $j_\mathrm{u}$', ...
            'Reg. cost $j_\mathrm{ax}$', 'Reg. cost $j_\mathrm{ay}$', ...
            'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    end
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Costs', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot dual norm terms  ----------------------------- %%
if plot_dualnorm    % only meaningful if slacks inactive
    figure; box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_u_opt_total.Time, lambda_dualnorm, ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel({'Lagrange multiplier:', 'dual-($\infty$)-norm'}, 'interpreter', 'latex', ...
        'FontName', 'Times New Roman', 'FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot vehicle velocity -------------------------- %%
if plot_vx
    figure; box on; grid on; hold on; 
    plot(data.debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps, ...
        'Color', [0 0.3961 0.7412], 'LineWidth', 1);
    plot(data.debug_slow.debug_slow_v_traj_mps.Time, ...
        data.debug_slow.debug_slow_v_traj_mps.Data(:, 1), ...
        'Color', [0.8902, 0.4471, 0.1333], 'LineWidth', 1);
    legend('$v_{\mathrm{Veh}}$', '$v_{\mathrm{TubeMPC - Trajectory}}$', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('$v$ in $\mathrm{ms^{-1}}$', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---------------------- Plot L2-Prediction error ------------------------ %%
% plots the L2-Norm of the deviation of the prediction from the real states
% and how many real states are outside the tube
if plot_L2_predError
    figure; 
    subplot(3, 1, 1); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, L2_e1, 'LineWidth', 1); 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e1_broken_con, 'LineWidth', 1); 
    legend('$L_2$-Norm $\Delta v$ prediction error', 'Predictions outside the tube', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', ...
        'FontSize', 12, 'Location', 'northwest'); 
    xlabel('$t$ in s','interpreter','latex','FontName','Times New Roman','FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
    
    subplot(3, 1, 2); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, L2_e2, 'LineWidth', 1); 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e2_broken_con, 'LineWidth', 1); 
    legend('$L_2$-Norm $d_y$ prediction error', 'Predictions outside the tube', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', ...
        'FontSize', 12, 'Location', 'northwest'); 
    xlabel('$t$ in s','interpreter','latex','FontName','Times New Roman','FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
    
    subplot(3, 1, 3); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, L2_e3, 'LineWidth', 1); 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e3_broken_con, 'LineWidth', 1); 
    legend('$L_2$-Norm $\dot{d}_y$ prediction error', 'Predictions outside the tube', ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', ...
        'FontSize', 12, 'Location', 'northwest'); 
    xlabel('$t$ in s','interpreter','latex','FontName','Times New Roman','FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ------------------- Plot L2-Norm of the QP-Solution -------------------- %%
% if the controller counteracts to the vehicle motion u increases ->
% L2-Norm of QP-Solution increases
if plot_L2_QPSol     
    figure; 
    subplot(2, 1, 1); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, u_t_norm_ax, 'LineWidth', 1); 
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('$L_2$-Norm optimized $\Delta a_x$ solution', 'interpreter', 'latex', 'FontName', ...
        'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);    
    
    subplot(2, 1, 2); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, u_t_norm_ay, 'LineWidth', 1);
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('$L_2$-Norm optimized $\Delta a_y$ solution ', 'interpreter', 'latex', 'FontName', ...
        'Times New Roman', 'FontSize', 10);
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end

%% ---- Plot L2-Norm of the difference of two consecutive predictions ----- %%
if plot_L2_diffError
    figure; 
    subplot(3, 1, 1); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e1_diff_norm, 'LineWidth', 1, ...
        'Color', [0.8902, 0.4471, 0.1333]); 
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    ylabel({'$L_2$-Norm difference',  '$\Delta v_x$ predictions'}, ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
    
    subplot(3, 1, 2); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e2_diff_norm, 'LineWidth', 1, ...
        'Color', [0.8902, 0.4471, 0.1333]); 
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10)
    ylabel({'$L_2$-Norm difference',  '$d_y$ predictions'}, ...
        'interpreter', 'latex','FontName','Times New Roman','FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
    
    subplot(3, 1, 3); box on; grid on; hold on; 
    plot(data.debug_slow.debug_slow_tmpc_cnt.Time, e3_diff_norm, 'LineWidth', 1, ...
        'Color', [0.8902, 0.4471, 0.1333]);   
    xlabel('$t$ in s', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    ylabel({'$L_2$-Norm difference', '$\dot{d}_y$ predictions'}, ...
        'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10); 
    xlim(tPlot); 
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15);
end    

end
