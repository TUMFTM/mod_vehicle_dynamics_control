% load dataset 
load('SimTest.mat'); 
% current data 
steering_req_vec = debug.debug_mvdc_curvvel_tracking_debug_Req_Delta_rad.Data; 
steering_vec = debug.debug_VehicleSensorData_Delta_Wheel_rad.Data; 
kappa_vec = debug.debug_mvdc_state_estimation_debug_StateEstimate_kappa_radpm.Data; 
velocity_vec = debug.debug_mvdc_state_estimation_debug_StateEstimate_v_mps.Data; 
curv_req = debug.debug_mvdc_curvvel_tracking_debug_Curv_TargetCurv_radpm.Data; 

% parameters
filter_coeff_virtual_ss = 0.07; 
P_VDC_SCLearn_Sigma = 0.01;            
P_VDC_SCLearn_LengthScales = [0.04, 5];     
l_front_m = 1.51; 
l_rear_m = 1.388;               
P_VDC_SCLearn_FilterCoeff = 0.999;       
P_VDC_SCLearn_KappaMax_radpm = 0.13; 
P_VDC_SCLearn_vMax_mps = 60; 
P_VDC_MinVelSlipCalc_mps = 3; 

% iterate via all data points
steering_steady_virtual = zeros(length(steering_req_vec)+1, 1); 
steering_comp = zeros(length(steering_req_vec), 1); 
% clear persistent variaables
clear learnSteeringCharacteristic; 
for i = 1:1:length(steering_req_vec)
    % call steering characteristic update 
    [SC_Kappa_Out, SC_Vel_Out, SC_DeltaNeutral_Out, SC_meas] =...
        learnSteeringCharacteristic(steering_steady_virtual(i), ...
                                        kappa_vec(i), velocity_vec(i), P_VDC_SCLearn_Sigma, ...
                            P_VDC_SCLearn_LengthScales, l_front_m, l_rear_m, ...
                            P_VDC_SCLearn_FilterCoeff, P_VDC_SCLearn_KappaMax_radpm, ...
                            P_VDC_SCLearn_vMax_mps, P_VDC_MinVelSlipCalc_mps); 
    steering_steady_virtual(i+1) = (1-filter_coeff_virtual_ss)*steering_steady_virtual(i) + ...
                                    filter_coeff_virtual_ss*steering_req_vec(i);
    steering_comp(i) = interp2(SC_Vel_Out, SC_Kappa_Out, SC_DeltaNeutral_Out,...
                            velocity_vec(i), curv_req(i),...
                            'linear', 0);                           
end

%% visualize Results
close all; 

figure; 
surface(SC_Vel_Out, SC_Kappa_Out, SC_DeltaNeutral_Out); 
colorbar; 
xlabel('Velocity in mps'); 
ylabel('Kappa in radpm'); 
zlabel('Steering Angle Correction in rad'); 
title('Smoothed Understeer Compensation Profile'); 

figure; 
surface(SC_Vel_Out, SC_Kappa_Out, SC_meas); 
colorbar; 
xlabel('Velocity in mps'); 
ylabel('Kappa in radpm'); 
zlabel('Steering Angle Correction in rad'); 
title('Raw Understeer Compensation Profile'); 

% visualize steering time values 
figure; 
plot(steering_vec); 
grid on; hold on; 
plot(kappa_vec*2.898);
plot(steering_req_vec);
plot(steering_steady_virtual);
plot(steering_comp); 
legend('steering', 'neutral', 'req', 'virtual_ss', 'ff'); 
