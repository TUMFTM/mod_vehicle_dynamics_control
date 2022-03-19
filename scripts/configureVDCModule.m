function configureVDCModule(vehicle)
%
% Author: Alexander Wischnewski     Date: 28-09-2018
% 
% Description: 
%   configures the vehicle dynamics control repository 
%   to use a specific parameter set 
% 
% Input: 
%   vehicle: Vehicle code string, e.g. 'db' or 'rc'
try 
    % general stuff 
    configureDD('vehicleparameter', vehicle); 
    % control component
    configureDD('mvdc_curvvel_tracking', vehicle); 
    configureDD('mvdc_path_feedback', vehicle);
    configureDD('mvdc_path_matching', vehicle);
    configureDD('mvdc_mpc', vehicle);
    configureDD('mvdc_disturbance_learning', vehicle);
    configureDD('mvdc_trajectory_driver', vehicle);
    % state estimation component 
    configureDD('mvdc_KF_PointMassJoint', vehicle);
    configureDD('mvdc_state_estimation', vehicle);
    configureDD('mvdc_estimation_preprocessing', vehicle);
    
catch e
    warning(['Something went wrong during configuration of ' ...
            'vehicle dynamics control parameters']); 
    disp('******************************************');     
    disp('Exception: ');
    disp(getReport(e))
    disp('******************************************');
end


