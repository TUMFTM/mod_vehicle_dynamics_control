function [SmoothTrajectory, debugInformation] =...
  smoothRaceline(RawTrajectory, weight_lat_d_m, weight_factor_deltakappa)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   01.02.2018
% 
% Description:  smoothes a path using an optimization problem. The problem 
%               is formulated in the frenet coordinate system. The actual 
%               path is reformulated interpreting the change in curvature 
%               from step to step as the input to a differential equation. 
%               The system is then overlayed with the resulting difference 
%               path of the same differential equation driven by the 
%               difference to the input signal of the original path. 
%               The optimization then seeks to minimize the lateral 
%               deviation in the overlay path coordinates and the sum of
%               curvature change of overlay path and original path. 
%         
% Inputs
%   RawTrajectory               Original trajectory which shall be smoothed
%   weight_lat_d_m              Weight factor for lateral deviation from original path
%   weight_factor_deltakappa    Weight factor for curvature derivative 

% Outputs
%   SmoothTrajectory            Smoothed trajectory which is the result of the process
%   debugInformation            A vector with a with debug information for logging 
%
% approximate differential equation for the overlay path in frenet coordinates
% d_f: lateral deviation
% psi_f: heading deviation 
% kappa_radpm_f: additional curvature of overlay path 
% d_f/ds = cos(psi_f)
% psi_f/ds = kappa_radpm_f

% discrete system state = [d_f, psi_f, kappa_radpm_f]
% with the new system input delta_kappa_radpm^2_f which is the derivative of 
% kappa_radpm_f along the path 
% system output = [x_m, y_m, delta_kappa_radpm_f]

% Inputs
%   RawTrajectory               Original trajectory which shall be smoothed
%   weight_lat_d_m              Weight factor for lateral deviation from original path
%   weight_factor_deltakappa    Weight factor for curvature derivative 

% Outputs
%   SmoothTrajectory            Smoothed trajectory which is the result of the process
%   debugInformation            A vector with a with debug information for logging 



%% map trajectory structure to local variables 
s_m_orig = double(RawTrajectory.s_m); 
x_m_orig = double(RawTrajectory.x_m); 
y_m_orig = double(RawTrajectory.y_m); 
psi_rad_orig = double(RawTrajectory.psi_rad); 
kappa_radpm_orig = double(RawTrajectory.kappa_radpm); 
v_mps_orig = double(RawTrajectory.v_mps); 
    
%% optimization algorithm
% basic check to ensure the path is valid and can be smoothed
if(~any(diff(s_m_orig)<=0))
  %% construct optimization problem
  % specify system parameters 
  B = [0; 0; 1];
  nStates = 3; 
  nMeas = 2; 
  % construct toeplitz matrix for state response of the following form
  % [0, 0, 0, 0, 0,...
  % [B, 0, 0, 0, 0,...
  % [AB, B, 0, 0, 0,...
  % [A^2B, AB, 0, 0,...
  % [A^3B, A^2B, AB, 0,...
  % [....
  T_state = zeros(nStates*length(s_m_orig), length(s_m_orig)); 
  T_state(nStates+1:2*nStates, 1) = B;
  for i = 3:1:length(s_m_orig)
    % construct next system step 
    ds_m = s_m_orig(i) - s_m_orig(i-1); 
    A = [1, ds_m, ds_m^2/2;...
      0, 1, ds_m;...
      0, 0, 1]; 
    % add up next step of the system response of the linearization around the trajectory 
    T_state((i-1)*nStates+1:i*nStates, 1:i) = [A*T_state((i-2)*nStates+1:(i-1)*3, 1), T_state((i-2)*nStates+1:(i-1)*3, 1:(i-1))];
  end
  % construct toeplitz matrix for system output response of the following form 
  % [D, 0, 0, 0, 0,...
  % [CB, D, 0, 0, 0,...
  % [CAB, CB, D, 0, 0,...
  % [CA^2B, CAB, D, 0,...
  % [CA^3B, CA^2B, CAB, D,...
  % [....
  T_output = zeros(nMeas*length(s_m_orig), length(s_m_orig));
  y_out = zeros(nMeas*length(s_m_orig), 1); 
  for i = 1:1:length(s_m_orig)
    % calculate correction factor to get d_ay from delta_kappa
    D_output = max(v_mps_orig(i), 1).^3./ds_m;
    % ((kappa_max - abs(kappa_radpm_orig(i)))/kappa_max)^2*...
    % fill in delta kappa weights and relate it to the path length 
    deltakappa_weight = weight_factor_deltakappa*D_output;
    if(i < length(s_m_orig)-1) 
      deltakappa_radpm = kappa_radpm_orig(i+1) - kappa_radpm_orig(i); 
    else
      deltakappa_radpm = 0; 
    end
    C = [weight_lat_d_m, 0, 0;...
      0, 0, 0]; 
    D = [0; deltakappa_weight]; 
    T_output(nMeas*(i-1)+1:nMeas*i, i) = D;
    % premultiply row with output matrix 
    T_output((i-1)*nMeas+1:i*nMeas, 1:(i-1)) = C*T_state((i-1)*nStates+1:i*nStates, 1:(i-1));    
    y_out((i-1)*nMeas+1:i*nMeas) = -D*deltakappa_radpm; 
  end

  %% solve linear equations system 
  sol = T_output\y_out; 
  kappa_radpm_err = cumsum(sol);
  kappa_radpm_smooth = [0; kappa_radpm_err(1:end-1)] + kappa_radpm_orig; 
  x_projected = T_state*sol; 

  %% reconstruct new path 
  % calculate new path orientation 
  psi_rad_smooth = x_projected(2:nStates:end) + psi_rad_orig;
  % calculate new x/y coordinates
  x_m_smooth = -cos(psi_rad_orig).*x_projected(1:nStates:end) + x_m_orig; 
  y_m_smooth = -sin(psi_rad_orig).*x_projected(1:nStates:end) + y_m_orig;
  d_m_smooth = x_projected(1:nStates:end);

  %% construct new trajectory 
  % copy trajectory and fill new fields 
  SmoothTrajectory = RawTrajectory; 
  SmoothTrajectory.x_m = single(x_m_smooth);
  SmoothTrajectory.y_m = single(y_m_smooth);
  SmoothTrajectory.psi_rad = single(psi_rad_smooth); 
  SmoothTrajectory.kappa_radpm = single(kappa_radpm_smooth);
  % fill debug information
  debugInformation = zeros(2, 1); 
  debugInformation(1) = sqrt(sum(d_m_smooth.^2)./length(x_m_smooth)); 
  debugInformation(2) = max(abs(d_m_smooth)); 
else
  % feedthrough trajectory in case there is no trajectory for optimization available
  SmoothTrajectory = RawTrajectory; 
  debugInformation = zeros(2, 1); 
end

