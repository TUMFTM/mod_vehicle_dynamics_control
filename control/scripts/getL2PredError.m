function [L2_e1, L2_e2, L2_e3, L2_ax, L2_ay, e1_out_tube, e2_out_tube, e3_out_tube, ax_out_tube,...
          ay_out_tube, e1_diff_norm, e2_diff_norm, e3_diff_norm, ax_diff_norm, ay_diff_norm] = ...
          getL2PredError(tube_data, error_1, error_2, error_3, ax_real, ay_real, ...
          learned_prediction_used, sys, tPred)
% function getL2PredError
% Authors:       Martin Euler
%                Alexander Wischnewski
% Description:  
%   helper function used to calculate L2-Norms of error predictions
% Inputs/parameters:
%   tube_data: sim data struct
%   N: length of prediction horizon
%   t_start:
%   sim_T: length of sim plot in seconds
%   error_1: real e1 error states
%   error_2: real e2 error states
%   error_3: real e3 error states
%   ax_real: real ax inputs
%   ay_real: real ay inputs
%   ABK_MPC: QP state transformation matrix
%   AD_MPC: QP disturbance transformation matrix
%   learned_prediction_used: flag whether prediction learning was used or not
%   sys: Parameter structed used for recalculation
%   tPred: time vector for prediction horizon

% Outputs:
%   L2_e1: L2-Norm of e1 prediction error
%   L2_e2: L2-Norm of e2 prediction error
%   L2_e3: L2-Norm of e3 prediction error
%   L2_ax: L2-Norm of ax prediction error
%   L2_ay: L2-Norm of ay prediction error
%   e1_out_tube: Number of predictions outside the tube in e1
%   e2_out_tube: Number of predictions outside the tube in e2
%   e3_out_tube: Number of predictions outside the tube in e3
%   ax_out_tube: Number of predictions outside the tube in ax
%   ay_out_tube: Number of predictions outside the tube in ay
%   e1_diff_norm: L2-Norm of difference of two consecutive e1-predictions
%   e2_diff_norm: L2-Norm of difference of two consecutive e2-predictions
%   e3_diff_norm: L2-Norm of difference of two consecutive e3-predictions
%   ax_diff_norm: L2-Norm of difference of two consecutive ax-predictions
%   ay_diff_norm: L2-Norm of difference of two consecutive ay-predictions

%% ----------------------- init matrices ------------------------------- %%
% store previous error states calculated from QP solution
e1_pred_prev = zeros(sys.N_hor, 1);
e2_pred_prev = zeros(sys.N_hor, 1);
e3_pred_prev = zeros(sys.N_hor, 1);
% mean of disturbance prediction
dist_mean = zeros(2*sys.N_hor,1);
% store previous QP-Solution
u_total_prev = zeros(2*sys.N_hor+sys.n_sys+sys.ns_total, 1);
% calculate length of data file to preallocate memoery 
len_data = length(tube_data.debug_slow.debug_slow_tmpc_cnt.Data); 
% init L2-error matrices
L2_e1 = zeros(len_data, 1);
L2_e2 = zeros(len_data, 1);
L2_e3 = zeros(len_data, 1);
L2_ax = zeros(len_data, 1);
L2_ay = zeros(len_data, 1);
dist = zeros(2, 1);
% init matrices for # of broken constraints
e1_out_tube = zeros(len_data, 1);
e2_out_tube = zeros(len_data, 1);
e3_out_tube = zeros(len_data, 1);
ax_out_tube = zeros(len_data, 1);
ay_out_tube = zeros(len_data, 1);
% init matrices for error prediction difference
e1_diff_norm = zeros(len_data, 1);
e2_diff_norm = zeros(len_data, 1);
e3_diff_norm = zeros(len_data, 1);
ax_diff_norm = zeros(len_data, 1);
ay_diff_norm = zeros(len_data, 1);

%% ----------------------------- calc L2-Norms ---------------------------- %%
% calculate errors for all values up to the last few where not enough data is available to judge
% the appearing predictions
for i = 1:(len_data - sys.N_hor)
    % get half axes of disturbance uncertainty
    dist(1) = (tube_data.debug_slow.debug_slow_ax_dist_pred_ub_mps2.Data(i,1)-...
               tube_data.debug_slow.debug_slow_ax_dist_pred_lb_mps2.Data(i,1))/2.0;
    dist(2) = (tube_data.debug_slow.debug_slow_ay_dist_pred_ub_mps2.Data(i,1)-...
               tube_data.debug_slow.debug_slow_ay_dist_pred_lb_mps2.Data(i,1))/2.0;
    % calculate the tube shape matrices
    [M_s] = calcTubeShapeMatrices(sys.A_d, sys.B_d, sys.K_t, sys.M_1, dist, false, ...
        sys.max_iter_terminal_set, sys.N_hor);
    % get mean of dist prediction and put it in correct vector format
    if learned_prediction_used
        dist_mean(1:2:end) = tube_data.debug_slow.debug_slow_ax_dist_pred_mean_mps2.Data(i, :);                             
        dist_mean(2:2:end) = tube_data.debug_slow.debug_slow_ay_dist_pred_mean_mps2.Data(i, :);
    end
    % get QP-Solution for current iteration step
    u_total = tube_data.debug_slow.debug_slow_u_opt_total.Data(i, :)';
    % calc error state prediction
    error_pred = sys.ABK_MPC*u_total(:,1) + sys.AD_MPC*dist_mean;
    e1_pred = error_pred(4:3:153); 
    e2_pred = error_pred(5:3:153); 
    e3_pred = error_pred(6:3:153); 
    % get predictions for ax and ay for current iteration step
    ax_pred = tube_data.debug_slow.debug_slow_ax_pred_mps2.Data(i, 1:sys.N_hor)';
    ay_pred = tube_data.debug_slow.debug_slow_ay_req_pred_mps2.Data(i, 1:sys.N_hor)';
    
    % extract corresponding values from actual measured signals via tPred since the time
    % discretization is not necessary equal, subtract tPred(1) to reset initial time to 0 for tPred
    tPred_actual = tube_data.debug_slow.debug_slow_tmpc_cnt.Time(i) + tPred - tPred(1); 
    ax_actual = interp1(ax_real.Time, ax_real.Data, tPred_actual, 'previous'); 
    ay_actual = interp1(ay_real.Time, ay_real.Data, tPred_actual, 'previous'); 
    e1_actual = interp1(error_1.Time, error_1.Data, tPred_actual, 'previous'); 
    e2_actual = interp1(error_2.Time, error_2.Data, tPred_actual, 'previous'); 
    e3_actual = interp1(error_3.Time, error_3.Data, tPred_actual, 'previous'); 
    
    % quantify the change rate of the solution for the accelerations and states
    ax_diff_norm(i) = norm(u_total(1:2:end-5) - u_total_prev(3:2:end-3), 2);
    ay_diff_norm(i) = norm(u_total(2:2:end-5) - u_total_prev(4:2:end-3), 2);
    e1_diff_norm(i) = norm(e1_pred(1:end-1) - e1_pred_prev(2:end), 2);
    e2_diff_norm(i) = norm(e2_pred(1:end-1) - e2_pred_prev(2:end), 2);
    e3_diff_norm(i) = norm(e3_pred(1:end-1) - e3_pred_prev(2:end), 2);
    
    % store current predictions for next iteration step 
    u_total_prev = u_total;
    e1_pred_prev = e1_pred; 
    e2_pred_prev = e2_pred; 
    e3_pred_prev = e3_pred; 
    
    % calculate L2 norms of prediction error 
    L2_ax(i) = norm(ax_pred - ax_actual, 2); 
    L2_ay(i) = norm(ay_pred - ay_actual, 2); 
    L2_e1(i) = norm(e1_pred - e1_actual, 2); 
    L2_e2(i) = norm(e2_pred - e2_actual, 2); 
    L2_e3(i) = norm(e3_pred - e3_actual, 2); 
    
    % calculate number of predictions which are outside of the tubes
    for j = 1:sys.N_hor
        % get input shape matrix
        M_input_s = sys.K_t*M_s(:,3*j-2:3*j)*sys.K_t';
        % check if real values are outside the tube
        if abs(ax_pred(j) - ax_actual(j)) > sqrt(M_input_s(1, 1))
            ax_out_tube(i) = ax_out_tube(i) +1;
        end
        if abs(ay_pred(j) - ay_actual(j)) > sqrt(M_input_s(2,2))
            ay_out_tube(i) = ay_out_tube(i) + 1;
        end
        if abs(e1_pred(j) - e1_actual(j)) > sqrt(M_s(1, 3*j-2))
            e1_out_tube(i) = e1_out_tube(i)+1;
        end
        if abs(e2_pred(j) - e2_actual(j)) > sqrt(M_s(2, 3*j-1))
            e2_out_tube(i) = e2_out_tube(i) + 1;
        end
        if abs(e3_pred(j) - e3_actual(j)) > sqrt(M_s(3, 3*j))
            e3_out_tube(i) = e3_out_tube(i) + 1;
        end          
    end    
end
