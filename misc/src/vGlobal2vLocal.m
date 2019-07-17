function [vx_mps, vy_mps] = vGlobal2vLocal(vEast_mps, vNorth_mps, aYaw_rad)
%
% vx_mps: velocity in vehicle longitudinal direction (forward positive) 
% vy_mps: velocity in vehicle lateral direction (left positive) 
% beta_rad: angle between the absolute velocity vector and the vehicle frame
% (positive if velocity vector is rotated counterclockwise against the vehicle frame) 
% 
% vEast_mps: velocity in global east direction 
% vNorth_mps: velocity in global north direction 
% aYaw_rad: Vehicle orientation in global coordinates. Zero at north and counterclockwise
% positive

vx_mps = zeros(length(vEast_mps), 1); 
vy_mps = zeros(length(vEast_mps), 1); 
for i = 1:1:length(vEast_mps)
  % construct rotation matrix for rotation in clockwise direction 
  R = [cos(-aYaw_rad(i)), -sin(-aYaw_rad(i));...
    sin(-aYaw_rad(i)), cos(-aYaw_rad(i))]; 
  % transform velocity vector into vehicle frame 
  vx_mps(i) = R(2, :)*[vEast_mps(i); vNorth_mps(i)]; 
  vy_mps(i) = -R(1, :)*[vEast_mps(i); vNorth_mps(i)];
end
