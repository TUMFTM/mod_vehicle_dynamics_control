function [OdometryPosition] = integrateOdometry(vx_mps, vy_mps, dPsi_rad, OdometryPosition_old, tS_s)
%__________________________________________________________________________
%% Documentation
% 
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
% 
% Start Date:   22.02.2018
% 
% Description:  implements an integration procedure for vehicle odometry based on 
%               velocity and yaw rate inputs. The odometry is obtained in inertial 
%               coordinates. 
% Inputs: 
%   vx_mps                  Vehicle speed along the longitudinal axis 
%   vy_mps                  Vehicle speed along the lateral axis 
%   dPsi_rad                Vehicle yaw rate 
%   OdometryPosition_old    Vehicle pose before update step (struct)
%                             [x_m, y_m, psi_rad]
%   tS_s                    Sample time 
% 
% Outputs: 
%   OdometryPosition        Vehicle pose after update step (struct) 
%                             [x_m, y_m, psi_rad]

%% Odometry integration algorithm
% transform orientation such that standard rotation matrix can be applied 
psi_rad = OdometryPosition_old.psi_rad + pi/2; 
% calculate derivatives 
dx = cos(psi_rad)*vx_mps - sin(psi_rad)*vy_mps; 
dy = sin(psi_rad)*vx_mps + cos(psi_rad)*vy_mps; 
dPsi = dPsi_rad; 
% integrate using heun scheme
x_m_mid = OdometryPosition_old.x_m + dx*tS_s/2; 
y_m_mid = OdometryPosition_old.y_m + dy*tS_s/2; 
psi_rad_mid = normalizeAngle(OdometryPosition_old.psi_rad + dPsi*tS_s/2); 
% calculate derivatives for second half
dx = cos(psi_rad_mid+pi/2)*vx_mps - sin(psi_rad_mid+pi/2)*vy_mps; 
dy = sin(psi_rad_mid+pi/2)*vx_mps + cos(psi_rad_mid+pi/2)*vy_mps; 
dPsi = dPsi_rad; 
% integrate using heun scheme 
OdometryPosition.x_m = x_m_mid + dx*tS_s/2; 
OdometryPosition.y_m = y_m_mid + dy*tS_s/2; 
OdometryPosition.psi_rad = normalizeAngle(psi_rad_mid + dPsi*tS_s/2); 