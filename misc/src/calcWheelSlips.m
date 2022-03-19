function [lambda_perc, alpha_rad] = calcWheelSlips(omega_rad, VehicleState, DeltaWheel_rad, tw_front_m, tw_rear_m, l_front_m, l_rear_m, tyreradius_front_m, tyreradius_rear_m, v_min)

% initialize variables
alpha_rad = zeros(4, 1); 
v_ref = zeros(4, 1); 
v_wheel = zeros(4, 1); 

vxCG_mps = VehicleState(1); 
vyCG_mps = VehicleState(2); 
dPsi_radps = VehicleState(3); 

% calculate chassis velocities at wheel centers projected to the ground
vx_fr = vxCG_mps + (dPsi_radps*tw_front_m*0.5);
vx_fl = vxCG_mps - (dPsi_radps*tw_front_m*0.5);
vx_rr = vxCG_mps + (dPsi_radps*tw_rear_m*0.5);
vx_rl = vxCG_mps - (dPsi_radps*tw_rear_m*0.5);
vy_fr = vyCG_mps + (dPsi_radps*l_front_m);
vy_fl = vyCG_mps + (dPsi_radps*l_front_m);
vy_rr = vyCG_mps - (dPsi_radps*l_rear_m);
vy_rl = vyCG_mps - (dPsi_radps*l_rear_m);
% calculate rotation matrix for front tires to transform to tire coordinates
R = [cos(-DeltaWheel_rad), -sin(-DeltaWheel_rad);...
  sin(-DeltaWheel_rad), cos(-DeltaWheel_rad)];
% transform to tire coordinates
vxT_fl = R(1, :)*[vx_fl; vy_fl]; 
vyT_fl = R(2, :)*[vx_fl; vy_fl]; 
vxT_fr = R(1, :)*[vx_fr; vy_fr]; 
vyT_fr = R(2, :)*[vx_fr; vy_fr];
% calculate tire slip angles 
if(vxCG_mps > v_min)
  alpha_rad(1) = atan2(-vyT_fl, vxT_fl); 
  alpha_rad(2) = atan2(-vyT_fr, vxT_fr); 
  alpha_rad(3) = atan2(-vy_rl, vx_rl); 
  alpha_rad(4) = atan2(-vy_rr, vx_rr); 

    % calculate absolute reference velocities of tire
    v_ref(1) = vxT_fl; 
    v_ref(2) = vxT_fr; 
    v_ref(3) = vx_rl; 
    v_ref(4) = vx_rr; 
    % calculate wheel over ground velocities 
    v_wheel(1:2) = tyreradius_front_m.*omega_rad(1:2); 
    v_wheel(3:4) = tyreradius_rear_m.*omega_rad(3:4); 
    % calculate tireslips
    lambda_perc = 100*(v_wheel-v_ref)./(max(v_min, v_ref)); 
else
  alpha_rad = zeros(4,1); 
  lambda_perc = zeros(4, 1);
end
