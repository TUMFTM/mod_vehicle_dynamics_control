function compareDyOutside()
% script plotTMPC
% Author:       Martin Euler         last update: 17-03-2020
% Description:  
%   function used to compare two TMPC controllers or scenarios
% Inputs/parameters:
%   SimTest.mat: File with recorded data
% Outputs:
%   Plots for different variables

%% -------------------- HOW TO USE ---------------------------- %%
% Run Simulink model controller_dev -> 
% execute convertSimLogs(logsout,'SimTest.mat'); in this folder ->
% define plotting parameters -> run this script
close all
%% ------------------ define parameters for plotting ------------------- %%
% number of iteration steps
N = 50;
% stepsize
Ts = 0.032;
% flag whether to compare two or three scenarios/controller
compareThree = true;
% planning horizon length
ending = N*Ts;
% maximum number of iterations for the terminal set
max_iter_terminal_set = 60; 
% define if the tube should be plotted
plot_tube = true;
% define whether tube has a static or variable size
VariableSize = true;
% define whether disturbance prediction was used
learned_prediction_used = true;
% admissible lateral error
d_y_lim = 1;
% admissible accelerations
a_y_lim = 13.5;
a_x_lim = 13.5;
% plotting starts at 't_sim_start' seconds of the simulation and plots for
% 'sim_T' seconds
t_sim_start =  20;
sim_T =  200;%109.9840;
t_start = floor(t_sim_start/Ts);
% define time step at which the tube and prediction is plotted
t_pred_start = 25;
t_shift = floor(t_pred_start/Ts); % matrix index for shift
% specify track
% Tracks: 'Berlin2019', 'LVMS', 'Millbrook', 'Modena', 'Zalazone'
track_name = 'Monteblanco2019_track';
% if data is stored with the same frequency as 1/Ts skip = 1, if data is
% stored 2 times slower than the step size -> skip = 2
skip = 1;
%% ----------------- define system parameters -------------------------- %%
% initial state uncertainty represented as an ellipsoid
M_1 = diag([0.2^2,0.05^2,0.2^2]);
% maximal disturbance bounds 
dist = [1,2.5];
% state and input dimensions
nx = 3;
nu = 2;
% import data struct of default controller/vehicle
struct_def = importdata('SimTest90.mat');
% extract tube and tube prediction data
tube_data_def = struct_def;
% import data struct of variation 1
struct_var_1 = importdata('SimTest925.mat');
% extract tube and tube prediction data
tube_data_var_1 = struct_var_1;
% import data struct of variation 1
struct_var_2 = importdata('SimTest95.mat');
% extract tube and tube prediction data
tube_data_var_2 = struct_var_2;
struct_var_3 = importdata('SimTest975.mat');
% extract tube and tube prediction data
tube_data_var_3 = struct_var_3;
struct_var_4 = importdata('SimTest99.mat');
% extract tube and tube prediction data
tube_data_var_4 = struct_var_4;
    
%% ----------------------- import raceline data -------------------------- %%
DictionaryObj = Simulink.data.dictionary.open(strcat(track_name,'.sldd'));
dDataSectObj = getSection(DictionaryObj,'Design Data');
Raceline = getValue(getEntry(dDataSectObj,'Raceline'));
% safety checks
if t_pred_start < t_sim_start
    error('t_pred_start must be >= than t_sim_start')
elseif t_pred_start + ending > t_sim_start + sim_T + 2 
    error('plotting of prediction must be in range of simulation plot -> decrease t_pred_start or increase sim_T')
end
A_j = [1 0 0;0 1 Ts;0 0 1];
B_j = [Ts 0;0 0;0 Ts];
Q_system = diag([5.3,18.5,0]);  
R_system = 1*eye(2);
[K_LQR, S_LQR, ~] = dlqr(A_j, B_j, Q_system, R_system);
K_LQR = -K_LQR;
K_6 = [-4.99999836650163,-2.05226098931609e-12,-2.94790872121019e-11;4.46919108031321e-11,-28.1332640797971,-10.4378393328938];
K_t = K_6;
% get QP transformation matrices
[AD_MPC, ABK_MPC, ~, ~] = calcQPTransformationMatrices(A_j, B_j, Q_system, R_system, S_LQR, N, nx, nu);
%% ------------------ import stored data from simulation ----------------%%
[error_1, error_2, error_3, ~, ~, ~,...
    ax_real, ay_real, ~, ~, v_traj, v_real, ~, ~, ~,...
    ~, ~, ~, ~, ~, ~, ~,~, dist, ~, ~, s_global] = initMatricesForPlots(N, nx, sim_T, Ts,...
    tube_data_def, learned_prediction_used, dist, t_shift, t_start);

[error_1_var_1, error_2_var_1, error_3_var_1, ~, ~, ~,...
    ax_real_var_1, ay_real_var_1, ~,  ~, v_traj_var_1, v_real_var_1,  ~,  ~,  ~,...
     ~,  ~,  ~,  ~,  ~,  ~,  ~,  ~, dist_var_1,  ~,  ~, s_global_var_1] = initMatricesForPlots(N, nx, sim_T, Ts,...
    tube_data_var_1, learned_prediction_used, dist, t_shift, t_start);

[error_1_var_2, error_2_var_2, error_3_var_2, ~, ~, ~,...
    ax_real_var_2, ay_real_var_2, ~,  ~, v_traj_var_2, v_real_var_2,  ~,  ~,  ~,...
     ~,  ~,  ~,  ~,  ~,  ~,  ~,  ~, dist_var_2,  ~,  ~, s_global_var_2] = initMatricesForPlots(N, nx, sim_T, Ts,...
    tube_data_var_2, learned_prediction_used, dist, t_shift, t_start);

[error_1_var_3, error_2_var_3, error_3_var_3, ~, ~, ~,...
    ax_real_var_3, ay_real_var_3, ~,  ~, v_traj_var_3, v_real_var_3,  ~,  ~,  ~,...
     ~,  ~,  ~,  ~,  ~,  ~,  ~,  ~, dist_var_3,  ~,  ~, s_global_var_3] = initMatricesForPlots(N, nx, sim_T, Ts,...
    tube_data_var_3, learned_prediction_used, dist, t_shift, t_start);

[error_1_var_4, error_2_var_4, error_3_var_4, ~, ~, ~,...
    ax_real_var_4, ay_real_var_4, ~,  ~, v_traj_var_4, v_real_var_4,  ~,  ~,  ~,...
     ~,  ~,  ~,  ~,  ~,  ~,  ~,  ~, dist_var_4,  ~,  ~, s_global_var_4] = initMatricesForPlots(N, nx, sim_T, Ts,...
    tube_data_var_4, learned_prediction_used, dist, t_shift, t_start);

[~,  ~,  ~, L2_e2,  ~,  ~, e2_out_tube,  ~,  ~,...
           ~,  ~,  ~,  ~] = getL2PredError(tube_data_def, N,...
          t_start, sim_T, error_1, error_2, error_3, ax_real, ay_real, ABK_MPC, AD_MPC,...
          learned_prediction_used, nx, Ts,A_j, B_j, K_t, M_1);
      
[~,  ~,  ~, L2_e2_var_1,  ~,  ~, e2_out_tube_var_1,  ~,  ~,...
   ~,  ~,  ~,  ~] = getL2PredError(tube_data_var_1, N,...
  t_start, sim_T, error_1_var_1, error_2_var_1, error_3_var_1, ax_real_var_1, ay_real_var_1, ABK_MPC, AD_MPC,...
  learned_prediction_used, nx, Ts,A_j, B_j, K_t, M_1);

[~,  ~,  ~, L2_e2_var_2,  ~,  ~, e2_out_tube_var_2,  ~,  ~,...
   ~,  ~,  ~,  ~] = getL2PredError(tube_data_var_2, N,...
  t_start, sim_T, error_1_var_2, error_2_var_2, error_3_var_2, ax_real_var_2, ay_real_var_2, ABK_MPC, AD_MPC,...
  learned_prediction_used, nx, Ts,A_j, B_j, K_t, M_1);

[~,  ~,  ~, L2_e2_var_3,  ~,  ~, e2_out_tube_var_3,  ~,  ~,...
   ~,  ~,  ~,  ~] = getL2PredError(tube_data_var_3, N,...
  t_start, sim_T, error_1_var_3, error_2_var_3, error_3_var_3, ax_real_var_3, ay_real_var_3, ABK_MPC, AD_MPC,...
  learned_prediction_used, nx, Ts,A_j, B_j, K_t, M_1);

[~,  ~,  ~, L2_e2_var_4,  ~,  ~, e2_out_tube_var_4,  ~,  ~,...
   ~,  ~,  ~,  ~] = getL2PredError(tube_data_var_4, N,...
  t_start, sim_T, error_1_var_4, error_2_var_4, error_3_var_4, ax_real_var_4, ay_real_var_4, ABK_MPC, AD_MPC,...
  learned_prediction_used, nx, Ts,A_j, B_j, K_t, M_1);

sum_1 = cumsum(e2_out_tube(1,:));
sum_2 = cumsum(e2_out_tube_var_1(1,:));
sum_3 = cumsum(e2_out_tube_var_2(1,:));
sum_4 = cumsum(e2_out_tube_var_3(1,:));
sum_5 = cumsum(e2_out_tube_var_4(1,:));
figure
subplot(2,1,1)
    grid on
    hold on
    disp(t_sim_start:skip*Ts:sim_T+t_sim_start)
    disp(length(v_real(1,:)))
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start, v_real(1,1:end-N),'Color',[0 0.3961 0.7412],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start,v_real_var_1(1,1:end-N),'Color',[0.8902, 0.4471, 0.1333],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start,v_real_var_2(1,1:end-N),'Color',[0, 0, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start,v_real_var_3(1,1:end-N),'Color',[0, 0.7, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start,v_real_var_4(1,1:end-N),'Color',[0.6275 0.1255 0.9412],'LineWidth', 1)
    legend('Abdeckung: 90\% ','Abdeckung: 92.5\%','Abdeckung: 95\% ','Abdeckung: 97.5\% ','Abdeckung: 99\% ','interpreter','latex','FontName','Times New Roman','FontSize', 10)
    ylabel('$v_{\mathrm{Fzg}}$ in $\mathrm{ms^{-1}}$ ','interpreter','latex','FontName','Times New Roman','FontSize', 10)
subplot(2,1,2)
    grid on
    hold on
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,e2_out_tube(1,:),'Color',[0 0.3961 0.7412],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,e2_out_tube_var_1(1,:),'Color',[0.8902, 0.4471, 0.1333],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,e2_out_tube_var_2(1,:),'Color',[0, 0, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,e2_out_tube_var_3(1,:),'Color',[0, 0.7, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,e2_out_tube_var_4(1,:),'Color',[0.6275 0.1255 0.9412],'LineWidth', 1)
    xlabel('t in s','interpreter','latex','FontName','Times New Roman','FontSize', 10)   
    ylabel('Anzahl $d_y$ au{\ss}erhalb','interpreter','latex','FontName','Times New Roman','FontSize', 10) 
figure
    grid on
    hold on
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,sum_1(1,:),'Color',[0 0.3961 0.7412],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,sum_2(1,:),'Color',[0.8902, 0.4471, 0.1333],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,sum_3(1,:),'Color',[0, 0, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,sum_4(1,:),'Color',[0, 0.7, 0],'LineWidth', 1)
    plot(t_sim_start:skip*Ts:sim_T+t_sim_start-Ts,sum_5(1,:),'Color',[0.6275 0.1255 0.9412],'LineWidth', 1)
    legend('Abdeckung: 90\% ','Abdeckung: 92.5\%','Abdeckung: 95\% ','Abdeckung: 97.5\% ','Abdeckung: 99\% ','interpreter','latex','FontName','Times New Roman','FontSize', 10)
    xlabel('t in s','interpreter','latex','FontName','Times New Roman','FontSize', 10)   
    ylabel('Summe Anzahl $d_y$ au{\ss}erhalb','interpreter','latex','FontName','Times New Roman','FontSize', 10) 