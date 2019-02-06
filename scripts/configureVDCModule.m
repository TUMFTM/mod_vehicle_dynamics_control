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
    configureDD('mloc_diag_vehicle', vehicle);
    % control component
    configureDD('mvdc_curvvel_tracking', vehicle); 
    configureDD('mvdc_path_feedback', vehicle);
    configureDD('mvdc_path_matching', vehicle);
    % state estimation component 
    configureDD('mvdc_KF', vehicle);
    configureDD('mvdc_state_estimation', vehicle);
    configureDD('mloc_estimation_postprocessing', vehicle);
catch e
    warning(['Something went wrong during configuration of ' ...
            'vehicle dynamics control parameters']); 
    disp('******************************************');     
    disp('Exception: ');
    disp(getReport(e))
    disp('******************************************');
end


