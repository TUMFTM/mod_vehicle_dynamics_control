function [TrajectoryPoints] = ...
  trajectoryInterpolation(TargetTrajectory, s_request_m)

%% Documentation 
%
% Author: Alexander Wischnewski     Start Date: 23.03.2020
%                                   Last update: 23.03.2020
%
% Description:  
%   reads a set of trajectory points at a given set of arc length coordinates
%
% Inputs:
%   TargetTrajectory               	  Trajectory which should be read out
%   s_request_m                       Requested set of arc length coordinates (vector) 
% 
% Outputs: 
%   TrajectoryPoints                  Trajectory points belonging to the requested arc
%                                       length coordinates

%% initialize trajectory points 
% initialize a prototype for a trajectory point
TrajectoryPoints.LapCnt = uint32(0); 
TrajectoryPoints.TrajCnt = uint32(0); 
TrajectoryPoints.PointIdx = uint16(0); 
TrajectoryPoints.s_loc_m = zeros(length(s_request_m), 1); 
TrajectoryPoints.s_glob_m = zeros(length(s_request_m), 1); 
TrajectoryPoints.x_m = zeros(length(s_request_m), 1); 
TrajectoryPoints.y_m = zeros(length(s_request_m), 1); 
TrajectoryPoints.psi_rad = zeros(length(s_request_m), 1); 
TrajectoryPoints.kappa_radpm = zeros(length(s_request_m), 1); 
TrajectoryPoints.v_mps = zeros(length(s_request_m), 1); 
TrajectoryPoints.ax_mps2 = zeros(length(s_request_m), 1);
TrajectoryPoints.banking_rad = zeros(length(s_request_m), 1);
TrajectoryPoints.ax_lim_mps2 = zeros(length(s_request_m), 1);
TrajectoryPoints.ay_lim_mps2 = zeros(length(s_request_m), 1);
TrajectoryPoints.tube_r_m = zeros(length(s_request_m), 1);
TrajectoryPoints.tube_l_m = zeros(length(s_request_m), 1);

%% readout trajectory according to required path coordinates 
TrajectoryPoints.LapCnt = TargetTrajectory.LapCnt; 
TrajectoryPoints.TrajCnt = TargetTrajectory.TrajCnt; 
TrajectoryPoints.PointIdx = uint16(0); 
TrajectoryPoints.s_loc_m = s_request_m; 
TrajectoryPoints.s_glob_m = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.s_glob_m, s_request_m, 'linear', 'extrap'); 
TrajectoryPoints.x_m = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.x_m, s_request_m, 'linear', 'extrap'); 
TrajectoryPoints.y_m = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.y_m, s_request_m, 'linear', 'extrap'); 
TrajectoryPoints.psi_rad = interp1_angle(TargetTrajectory.s_loc_m, TargetTrajectory.psi_rad, s_request_m, 'linear', TargetTrajectory.psi_rad(end)); 
TrajectoryPoints.kappa_radpm = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.kappa_radpm, s_request_m, 'linear', TargetTrajectory.kappa_radpm(end)); 
TrajectoryPoints.v_mps = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.v_mps, s_request_m, 'linear', 0); 
TrajectoryPoints.banking_rad = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.banking_rad, s_request_m, 'linear', TargetTrajectory.banking_rad(end)); 
TrajectoryPoints.ax_lim_mps2 = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.ax_lim_mps2, s_request_m, 'linear', TargetTrajectory.ax_lim_mps2(end)); 
TrajectoryPoints.ay_lim_mps2 = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.ay_lim_mps2, s_request_m, 'linear', TargetTrajectory.ay_lim_mps2(end));
TrajectoryPoints.tube_r_m = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.tube_r_m, s_request_m, 'linear', TargetTrajectory.tube_r_m(end));
TrajectoryPoints.tube_l_m = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.tube_l_m, s_request_m, 'linear', TargetTrajectory.tube_l_m(end)); 
% recalculate ax if s_request is vector valued to ensure path description is valid
if(length(s_request_m) > 1) 
    TrajectoryPoints.ax_mps2 = calcPathAx(TrajectoryPoints.s_loc_m, TrajectoryPoints.v_mps);
else
    TrajectoryPoints.ax_mps2 = interp1(TargetTrajectory.s_loc_m, TargetTrajectory.ax_mps2, s_request_m, 'previous', 0);
end
