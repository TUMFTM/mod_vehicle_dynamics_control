function [Fx_N, Delta_rad, mvdc_exactlin_feedback_debug] = exactlin_feedback(ActualPathPoint, VehicleDynamicState, ActualPathPosition, vehiclemass_kg, roh_air, drag_coefficient, l_front_m, l_rear_m, P_VDC_MinVelSlipCalc_mps,...
    P_VDC_EL_axMaxFB_mps2, P_VDC_EL_c_F_N, P_VDC_EL_c_R_N, P_VDC_EL_c_1prad, P_VDC_EL_alphaMax_rad, P_VDC_EL_vFullControl_mps, P_VDC_EL_LatControl_w_0, P_VDC_LatControl_D, P_VDC_LongControl_Kp)

%% Documentation 
%
% Author: Alexander Wischnewski     Start Date: 22.05.2019
%
% Description:  
%   A nonlinear control design method similar to exact linearization to
%   derive a path and velocity tracking controller. The key steps are the
%   specification of the output closed loop dynamics similar to exact
%   lineariziation and a nonlinear transformation calculating the
%   therefore required steering angle and force request. In contrast to
%   most approaches we utilize a steady-state assumption for the
%   inversion of the lateral dynamics (calculate a steering angle from a
%   required lateral acceleration). This increases robustness since the
%   race-car dynamics are fast which causes oscillation due to sensitivity
%   of the inversion. 
%
% Inputs:
%   ActualPathPoint                 Current path point and all derivative
%                                       information 
%   VehicleDynamicState             Vehicle state variables 
%   ActualPathPosition              Current vehicle pose in path
%                                       coordinates 
%   vehiclemass_kg                  Vehicle mass in kg
%   roh_air                         Air densitiy 
%   drag_coefficient                Aerodynamic drag coefficient (cw*A)
%   l_front_m                       Distance from cog to front axle
%   l_rear_m                        Distance from cog to rear axle 
%   P_VDC_MinVelSlipCalc_mps        Minimum speed for which reliable slip
%                                   information are available 
%   P_VDC_EL_axMaxFB_mps2           Maximum acceleration from feedback
%                                       control for the velocity dynamics
%   P_VDC_EL_c_F_N                  Front axle lateral force coefficient
%   P_VDC_EL_c_R_N                  Rear axle lateral force coefficient
%   P_VDC_EL_c_1prad                Lateral tire stiffness coefficient
%                                       (both axles)
%   P_VDC_EL_alphaMax_rad           Maximum side slip angle requested 
%   P_VDC_EL_vFullControl_mps       Speed above which the lateral
%                                       controller can request full feedback 
%   P_VDC_EL_LatControl_w_0         Lateral closed loop target frequency 
%   P_VDC_LatControl_D              Lateral closed loop damping 
%   P_VDC_LongControl_Kp            Velocity feedback 
%
% Outputs: 
%   Fx_N                            Force request
%   Delta_rad                       Steering request 
%   mvdc_exactlin_feedback_debug    Debug variables 

%% Calculate control errors 
% calculate velocity error
e_v = ActualPathPoint.v_mps - VehicleDynamicState.v_mps; 
% calculate longitudinal controller target
d_e_v = sign(e_v)*min(abs(P_VDC_LongControl_Kp*e_v), P_VDC_EL_axMaxFB_mps2);
% calculate lateral error 
e_d = ActualPathPosition.d_m; 
% calculate lateral error derivative
d_e_d = (ActualPathPosition.psi_rad + VehicleDynamicState.beta_rad)*VehicleDynamicState.v_mps; 
% calculate lateral controller target 
dd_e_d = -P_VDC_EL_LatControl_w_0^2*e_d - 2*P_VDC_LatControl_D*P_VDC_EL_LatControl_w_0*d_e_d; 

%% Calculate control requests
% use nonlinear formulas for higher speeds
% otherwise use kinematic formulas
if(VehicleDynamicState.v_mps > P_VDC_MinVelSlipCalc_mps) 
    % apply scaling based on speed
    if(VehicleDynamicState.v_mps < P_VDC_EL_vFullControl_mps)
        dd_e_d = (VehicleDynamicState.v_mps/P_VDC_EL_vFullControl_mps)^2*dd_e_d;
    end
    ay_Target_mps2 = (dd_e_d + ActualPathPoint.kappa_radpm*VehicleDynamicState.v_mps^2); 
    % calculate nonlinear steady state matching current lateral
    % acceleration request using: 
    % cf*atan(c*alphaF) + cR*atan(c*alphaR) = m*ay
    % cf*atan(c*alphaF)*lf - cR*atan(c*alphaR)*lr = 0
    % and calculate atan(c*alphaF) and atan(c*alphaR)
    b = [1, 1; l_front_m, -l_rear_m]\[vehiclemass_kg*ay_Target_mps2; 0];
    % limit forces to value at alphaMax 
    Fy_F = sign(b(1))*min(abs(b(1)), P_VDC_EL_c_F_N*atan(P_VDC_EL_c_1prad*P_VDC_EL_alphaMax_rad)); 
    Fy_R = sign(b(2))*min(abs(b(2)), P_VDC_EL_c_R_N*atan(P_VDC_EL_c_1prad*P_VDC_EL_alphaMax_rad)); 
    % calculate axle target sideslip 
    alpha_F_rad = tan(Fy_F/P_VDC_EL_c_F_N)/P_VDC_EL_c_1prad; 
    alpha_R_rad = tan(Fy_R/P_VDC_EL_c_R_N)/P_VDC_EL_c_1prad; 
    % calculate target steering angle using
    % alpha_F_rad = Delta_rad - VehicleDynamicState.beta_rad -
    % l_f*dPsi_radps/v_mps; 
    Delta_rad = alpha_F_rad + VehicleDynamicState.beta_rad + l_front_m*VehicleDynamicState.dPsi_radps/VehicleDynamicState.v_mps; 
    % modify longitudinal target acceleration to fit path parameter
    % dependent representation 
    ax_T = ActualPathPoint.ax_mps2*(VehicleDynamicState.v_mps/max(ActualPathPoint.v_mps, P_VDC_MinVelSlipCalc_mps)); 
else
    Delta_rad = ((1/P_VDC_EL_vFullControl_mps^2)*dd_e_d + ActualPathPoint.kappa_radpm)*(l_front_m+l_rear_m);
    alpha_F_rad = 0; 
    alpha_R_rad = 0; 
    Fy_F = 0; 
    Fy_R = 0; 
    ax_T = ActualPathPoint.ax_mps2; 
end
% calculate required longitudinal force force from 
% d_e_v = ax_T - (Fx - 0.5*1.2*cw*v^2)/m
Fx_N = vehiclemass_kg*(ax_T + d_e_v) + 0.5*roh_air*drag_coefficient*VehicleDynamicState.v_mps.^2; 

%% write debug information 
mvdc_exactlin_feedback_debug.VelocityError_mps = e_v; 
mvdc_exactlin_feedback_debug.LongAccTarget_mps2 = d_e_v; 
mvdc_exactlin_feedback_debug.LatError_m = e_d; 
mvdc_exactlin_feedback_debug.LatErrorDer_mps = d_e_d; 
mvdc_exactlin_feedback_debug.LatAccTarget_mps2 = dd_e_d; 
mvdc_exactlin_feedback_debug.EstimateAlphaF_rad = alpha_F_rad; 
mvdc_exactlin_feedback_debug.EstimateAlphaR_rad = alpha_R_rad; 
mvdc_exactlin_feedback_debug.EstimateFyF_N = Fy_F; 
mvdc_exactlin_feedback_debug.EstimateFyR_N = Fy_R; 
mvdc_exactlin_feedback_debug.RequestDelta_rad = Delta_rad; 
mvdc_exactlin_feedback_debug.RequestFx_N = Fx_N; 


