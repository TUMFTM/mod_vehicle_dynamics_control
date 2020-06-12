function result = analyzePathDeviationDetection(debug, PerformanceSpec)

% Author: Alexander Wischnewski     Date: 01-07-2019
% 
% Description: 
%   scans the debug data and ground truth whether the path deviation
%   detection works as specified
% 
% Input: 
%   debug:          Debug structure of the vehicle
%   PerformanceSpec Structure with fields: 
%       HighDeviationLat_m 
%       VeryHighDeviationLat_m 
%       HighDeviationHeading_rad
%       VeryHighDeviationHeading_rad
%       TimeSpec_s
% 
% Outputs: 
%   result:       1 if valid according to spec, 0 otherwise

result = 1; 

disp(' ');
disp('------------------------------');
disp('Check High Path Deviation Detection: ');
disp('------------------------------');
idxFailure = find(abs(debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data) >= PerformanceSpec.HighDeviationLat_m | ...
    abs(debug.debug_mvdc_path_matching_debug_PathPos_psi_rad.Data) >= PerformanceSpec.HighDeviationHeading_rad, 1);
if isempty(idxFailure)
    disp('The vehicle did not deviate enough from the path for high path deviation detection.'); 
else
    % load time where failure occured
    tFailure = debug.debug_mloc_statemachine_debug_TUMVehicleState.Time(idxFailure); 
    % find time where vehicle has gone to emergency
    idxDetect = find(uint16(debug.debug_mloc_statemachine_debug_TUMVehicleState.Data) == 50, 1);
    tFailureDetected_s = debug.debug_mloc_statemachine_debug_TUMVehicleState.Time(idxDetect);
    disp(['The failure was detected within ' num2str(tFailureDetected_s-tFailure) 's.']); 
    if(tFailureDetected_s - tFailure > PerformanceSpec.TimeSpec_s)
        disp('The failure was not detected within the specified time window'); 
        result = 0; 
        return
    end
    disp('The high path deviation failure was detected within the specified time window'); 
end

disp(' ');
disp('------------------------------');
disp('Check Very High Path Deviation Detection: ');
disp('------------------------------');
idxFailure = find(abs(debug.debug_mvdc_path_matching_debug_PathPos_d_m.Data) >= PerformanceSpec.VeryHighDeviationLat_m | ...
    abs(debug.debug_mvdc_path_matching_debug_PathPos_psi_rad.Data) >= PerformanceSpec.VeryHighDeviationHeading_rad, 1);
if isempty(idxFailure)
    disp('The vehicle did not deviate enough from the path for very high path deviation detection.'); 
else
    % load time where failure occured
    tFailure = debug.debug_mloc_statemachine_debug_TUMVehicleState.Time(idxFailure); 
    % find time where vehicle has gone to emergency
    idxDetect = find(uint16(debug.debug_mloc_statemachine_debug_TUMVehicleState.Data) == 60, 1);
    tFailureDetected_s = debug.debug_mloc_statemachine_debug_TUMVehicleState.Time(idxDetect);
    disp(['The failure was detected within ' num2str(tFailureDetected_s-tFailure) 's.']); 
    if((tFailureDetected_s - tFailure) > PerformanceSpec.TimeSpec_s)
        disp('The failure was not detected within the specified time window'); 
        result = 0; 
        return
    end
    disp('The very high path deviation failure was detected within the specified time window'); 
end

disp(' ');
disp('All checks passed.');