function [d_upper_m, d_lower_m, v_upper_mps, v_lower_mps] = ...
    calcPredictionTube(ax_pred_mps2, ay_pred_mps2, ax_quantile_mps2_upper, ax_quantile_mps2_lower,...
                        ay_quantile_mps2_upper, ay_quantile_mps2_lower, d_Tube_p, v_Tube_p)
    %_________________________________________________________________
    %% Documentation       
    %
    % Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
    % 
    % Start Date:   05.12.2019
    % 
    % Description:  
    %         
    % Inputs:
    %   
    %
    % Outputs: 

    %% Parameters
    % tube prediction
    mode = 1; % 1 is disturbance upper/lower bound, 2 is disturbance energy bound
    % general
    w0 = 2; 
    D = 0.6; 
    m = 1160; 
    VelKp = 2500; 
    TVelDist = 1; 
    tS = 0.04; 
    N = 51; % number of prediction elements, first is identical with current point
    axEnergyLim = 20; 
    ayEnergyLim = 10; 
    
    %% Initialization 
    d_upper_m = zeros(N, 1); 
    d_lower_m = zeros(N, 1); 
    v_upper_mps = zeros(N, 1); 
    v_lower_mps = zeros(N, 1); 

    %% Calc dynamic system representations
    % velocity control
    Av = [-VelKp/m, -1; 0, -1/TVelDist]; 
    Bv = [1; 1/TVelDist]; 
    Av_dis = eye(2) + tS*Av; 
    Bv_dis = tS*Bv; 
    % lateral control
    Alat = [0, 1;...
        -w0^2, -2*D*w0]; 
    Blat = [0; 1;];
    Alat_dis = eye(2) + Alat*tS; 
    Blat_dis = Blat*tS; 
    % initial set uncertainty 
    d_Tube_Q = diag([0.001, 0.01]); 
    v_Tube_Q = diag([0.001, 0.1]); 

    %% Tube prediction 
    % convert initial point to prediction 
    [d_upper_m(1), d_lower_m(1)] = calcInterval(d_Tube_p, d_Tube_Q, 1); 
    [v_upper_mps(1), v_lower_mps(1)] = calcInterval(v_Tube_p, v_Tube_Q, 1); 

    % calculate rest of the tube
    for i = 2:1:N
        switch(mode) 
            case 1 
                % mean of upper and lower quantile for offset of prediction
                mean_ax = ax_pred_mps2(i-1) + 0.5*(ax_quantile_mps2_upper + ax_quantile_mps2_lower);
                mean_ay = ay_pred_mps2(i-1) + 0.5*(ay_quantile_mps2_upper + ay_quantile_mps2_lower);
                % spread of upper and lower quantile for spread of disturbance prediction
                spread_ax_quantile = 0.5*(ax_quantile_mps2_upper - ax_quantile_mps2_lower);
                spread_ay_quantile = 0.5*(ay_quantile_mps2_upper - ay_quantile_mps2_lower);
                % transform uncertainty ellipsoid to next time stamp
                [v_Tube_p, v_Tube_Q] = updateSystem(v_Tube_p, v_Tube_Q, mean_ax, spread_ax_quantile, Av_dis, Bv_dis); 
                [d_Tube_p, d_Tube_Q] = updateSystem(d_Tube_p, d_Tube_Q, mean_ay, spread_ay_quantile, Alat_dis, Blat_dis); 
            case 2
                [v_Tube_p, v_Tube_Q] = updateEnergyLimitedReachset(v_Tube_p, v_Tube_Q, ax_pred_mps2(i-1), 50, Av_dis, Bv_dis); 
                [d_Tube_p, d_Tube_Q] = updateEnergyLimitedReachset(d_Tube_p, d_Tube_Q, ay_pred_mps2(i-1), 100, Alat_dis, Blat_dis); 
        end
        % obtain prediction interval from uncertainty ellipsoid 
        [v_upper_mps(i), v_lower_mps(i)] = calcInterval(v_Tube_p, v_Tube_Q, 1);   
        [d_upper_m(i), d_lower_m(i)] = calcInterval(d_Tube_p, d_Tube_Q, 1);   
    end
end

function [p_new, Q_new]  = updateEnergyLimitedReachset(d_Tube_p, d_Tube_Q, d_mean, d_spread, A, B)
    Q_new = A*d_Tube_Q*A' + B*d_spread*B'; 
    % transform center point of ellipsoid
    p_new = A*d_Tube_p + B*d_mean; 
end

function [p_new, Q_new] = updateSystem(d_Tube_p, d_Tube_Q, d_mean, d_spread, A, B)
    % perform transformation of uncertainty via system equation 
    Q1 = A*d_Tube_Q*A'; 
    Q2 = B*d_spread^2*B'; 
    % calc minkowski sum 
    c = sqrt(trace(Q1)/max(1e-5, trace(Q2))); 
    Q_new = (1+1/c)*Q1+(1+c)*Q2; 
    % transform center point of ellipsoid
    p_new = A*d_Tube_p + B*d_mean; 
end

function [upper, lower] = calcInterval(p, Q, dim)
    % p   --> center point in R^n
    % Q   --> Ellipsoids matrix in R^(nxn)
    % dim --> dimension to extract
    upper = p(dim) + sqrt(Q(dim, dim));
    lower = p(dim) - sqrt(Q(dim, dim)); 
end