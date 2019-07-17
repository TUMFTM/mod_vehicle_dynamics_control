function [ dBeta_radps ] = calc_dBeta( vx_mps, vy_mps, ax_mps2, ay_mps2, dPsi_radps )
% calculates the derivative of the side slip angle from accelerations and velocities 

% minimum speed for calculation of beta derivatives 
if(vx_mps > 8) 
  dvx_mps = ax_mps2 + dPsi_radps*vy_mps; 
  dvy_mps = ay_mps2 - dPsi_radps*vx_mps; 
  dBeta_radps = (vx_mps*dvy_mps - vy_mps*dvx_mps)/(vx_mps.^2)*...
    1/(1+(vy_mps/vx_mps).^2);
else
  dBeta_radps = 0; 
end

end

