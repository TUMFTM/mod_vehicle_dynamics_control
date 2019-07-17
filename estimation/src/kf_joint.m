function [x_est, P_update, res]= kf_joint(x, P, z, u, fusionVec,...
                  tS_s, P_VDC_InputCov,...
                  P_VDC_OutlierBounds, P_VDC_YawAngleDepCovMode,... 
                  P_VDC_EnableInputCrossCorrelation, P_VDC_MeasCov)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
%               Madeline Wolz
% 
% Start Date:   03.02.2018
% 
% Description:  implements a an Extended Kalman Filter for fusing acceleration,
%               velocity and position sensor information. The acceleration measurements
%               are used as an input to the system model while the others are 
%               used to correct the state estimates. The system model itself is a 
%               purely kinematic forward integration of accelerations in the vehicle 
%               coordinate frame and furthermore integration of the resulting velocities
%               and the yaw rate in a global inertial frame. The main benefit of this 
%               concept compared to a more sophisticated model is its consistent accuracy
%               over the complete nonlinear tire range. Even though all state variables
%               are directly measurable, the Kalman Filter fusion improves the signal 
%               quality and provides a natural framework to handle multi rate
%               measurements. 
%            
% Inputs:
%   x                                   State vector - [x_m, y_m, psi_rad, vx_mps, vy_mps]
%   P                                   Covariance matrix of the state estimation 
%   z                                   Measurement vector - 
%                                         [x_Loc1_m, y_Loc1_m, psi_YawAngleLoc1_rad, 
%                                          x_Loc2_m, y_Loc2_m, psi_YawAngleLoc2_rad,
%                                          vx_Vel1CoG_mps, vy_Vel2CoG_mps,
%                                          vx_Vel2CoG_mps, vy_Vel2CoG_mps]
%   u                                   Input vector - [dPsi_radps, ax_mps2, ay_mps2]
%   fusionVec                           Bit mask for measurement vector z
%                                         which enables/disables fusion 
%   tS_s                                Sample time
%   P_VDC_InputCov                      Vector with covariances for the
%                                         inputs in u (same order)
%   P_VDC_OutlierBounds                 Number of standard deviations tolerated until 
%                                         outlier rejection is applied 
%   P_VDC_YawAngleDepCovMode            Source for YawAngle dependant covariances
%                                         0: yawAngle dependancy disabled
%                                         1: Loc1 yawAngle measurement
%                                         2: Loc2 yawAngle measurement
%                                         3: Current yawAngle estimate
%   P_VDC_EnableInputCrossCorrelation   enables the cross correlation between
%                                         input variables
%   P_VDC_MeasCov                       Vector with covariances for the
%                                         measurements in z (same order)
%
% Outputs: 
%   x_est                   Updated state vector - [x_m, y_m, psi_YawAngle_rad, 
%                                                   vx_VelCoG_mps, vy_VelCoG_mps]
%   P_updated               Updated state estimation covariance matrix 
%   res                     Pre-fusion measurement residuals
%__________________________________________________________________________
           
    %% Initialization
    % map state and input variables to named variables for easier usage
    x_m             =  x(1);
    y_m             =  x(2);
    psi_rad         =  x(3);
    vx_mps          =  x(4);
    vy_mps          =  x(5);
    dPsi_radps      = u(1);
    ax_mps2         = u(2);
    ay_mps2         = u(3);
    % specify mask for states which need angular normalization 
    n = logical([0, 0, 1, 0, 0]');
    % specify mask for measurements which need angular normalization
    m = logical([0, 0, 1, 0, 0, 1, 0, 0, 0, 0]'); 

    %% Construct discretization
    % Jacobian matrix A consisting of partial derivatives with respect to the 
    % system state from the system update equation (specified below in function pmm)  
    A = [0, 0,(-cos(psi_rad) * vx_mps + sin(psi_rad) * vy_mps),...
           -sin(psi_rad), - cos(psi_rad);...
         0, 0,( -sin(psi_rad) * vx_mps - cos(psi_rad) * vy_mps),...
            cos(psi_rad),  -sin(psi_rad);...
         0, 0, 0, 0, 0;...
         0, 0, 0, 0, dPsi_radps;...
         0, 0, 0, -dPsi_radps, 0];
    % discretize using euler forward
    A_d = eye(length(x)) + tS_s*A; 
    % Jacobian matrix B consisting of partial derivatives with respect to the 
    % system input from the system update equation (specified below in function pmm)
    if(P_VDC_EnableInputCrossCorrelation)
        B = [0, 0, 0;...
             0, 0, 0;...
             1, 0, 0;...
             vy_mps, 1, 0;...
             -vx_mps, 0, 1];
    else
        B = [0, 0, 0;...
             0, 0, 0;...
             1, 0, 0;...
             0, 1, 0;...
             0, 0, 1];
    end
    % discretize using euler forward
    B_d = tS_s*B; 

    %% observation matrices
    % nominal measurement matrix if all sensors are available 
    H_full = [1, 0, 0, 0, 0;...
              0, 1, 0, 0, 0;...
              0, 0, 1, 0, 0;...
              1, 0, 0, 0, 0;...
              0, 1, 0, 0, 0;...
              0, 0, 1, 0, 0;...
              0, 0, 0, 1, 0;...
              0, 0, 0, 0, 1;...
              0, 0, 0, 1, 0;...
              0, 0, 0, 0, 1];
    % extraction matrix which only uses the sensors configured and updated
    H_ext = diag(double(fusionVec));
    % build final measurement matrix with zero entries for sensors not used
    H_d = H_ext*H_full;  

    % construct measurement covariance matrix 
    R = diag(P_VDC_MeasCov);
    % map measurement covariances to track orientation 
    psi_forCov_rad = 0; 
    if(P_VDC_YawAngleDepCovMode>0)
        switch P_VDC_YawAngleDepCovMode
            case 1
                psi_forCov_rad = z(3); 
            case 2
                psi_forCov_rad = z(6);
            case 3
                psi_forCov_rad = x(3); 
        end
        R(1:3, 1:3) = transCov2TrackOrientation(diag(P_VDC_MeasCov(1:3)), psi_forCov_rad); 
        R(4:6, 4:6) = transCov2TrackOrientation(diag(P_VDC_MeasCov(4:6)), psi_forCov_rad); 
    end


    %% apply Kalman Filter Algorithm
    % Prediction using explicitly specified analytic linearization
    [x, P_pred] = ekf_prediction_analytic(x, u, P, P_VDC_InputCov, n, @pmm, A_d, B_d, tS_s); 
    % Call kalman filter update function 
    [x_est, P_update, res_tmp] = ekf_update( x, z, H_d, R, n, m, P_pred, P_VDC_OutlierBounds); 

    %% Calculate additional residual information
    % fills complete residual bus with additional information 
    % see bus definition for details
    res_yawRate1_radps = 0; 
    res_yawRate2_radps = 0; 
    res_s_Loc1_m = cos(-(psi_forCov_rad+pi/2))*res_tmp(1) - sin(-(psi_forCov_rad+pi/2))*res_tmp(2); 
    res_d_Loc1_m = sin(-(psi_forCov_rad+pi/2))*res_tmp(1) + cos(-(psi_forCov_rad+pi/2))*res_tmp(2);
    res_s_Loc2_m = cos(-(psi_forCov_rad+pi/2))*res_tmp(4) - sin(-(psi_forCov_rad+pi/2))*res_tmp(5); 
    res_d_Loc2_m = sin(-(psi_forCov_rad+pi/2))*res_tmp(4) + cos(-(psi_forCov_rad+pi/2))*res_tmp(5); 
    res = [res_tmp; res_yawRate1_radps; res_yawRate2_radps; ...
                res_s_Loc1_m; res_d_Loc1_m; res_s_Loc2_m; res_d_Loc2_m;]; 
end

function xpred = pmm(x, u, tS)
    %% Description  
    % rigid body model using accelerations and yaw rate as inputs and applying simple forward
    % integration in vehicle coordinate frame to obtain the velocities. The latter are then 
    % used to apply forward integration in global cartesian coordinates to obtain the
    % position. 
    %% State and Input mapping
    Psi_rad = x(3); 
    vx_mps = x(4); 
    vy_mps = x(5); 
    dPsi_radps = u(1); 
    ax_mps2 = u(2); 
    ay_mps2 = u(3); 
    %% Differential equations
    dx = [-sin(Psi_rad) * vx_mps - cos(Psi_rad) * vy_mps;...    
     cos(Psi_rad) * vx_mps - sin(Psi_rad) * vy_mps;... 
     dPsi_radps;...
     ax_mps2 + dPsi_radps * vy_mps;...
     ay_mps2 - dPsi_radps * vx_mps;]; 
    %% Integration
    xpred = x + tS*dx;   
end

function [M_trans] = transCov2TrackOrientation(M, psi_track)
    % Description:  transforms covariance matrix M (in global cooridnates) 
    %               to a coordinate system along the track 
    % 
    % Inputs:   
    %   M               Covariance matrix which should be transformed
    %   psi_Track_rad   Track orientation


    % Transformation Matrix T based on the standard rotation matrix 
    % as the covariance Matrix M also includes a third component (the orientation) 
    % which shall not be transformed, the third dimension is added. 
    T = [cos(psi_track+pi/2), -sin(psi_track+pi/2), 0;...
         sin(psi_track+pi/2),  cos(psi_track+pi/2), 0;... 
         0, 0, 1];

    M_trans = T * M * T';

end

