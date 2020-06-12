function result = analyzeHardStop(debug, ground_truth, tFailure_s, PerformanceSpec)

% Author: Alexander Wischnewski     Date: 24-02-2019
% 
% Description: 
%   scans the debug data and ground truth whether a hard stop
%   was performed within the given specifications
% 
% Input: 
%   debug:          Debug structure of the vehicle
%   ground_truth:   Physical ground truth data from simulation
%   tFailure_s:     Timestamp where the failure occured
%   PerformanceSpec Structure with fields: 
%       AccxMin_mps2        Minimum mean negative acceleration applied until vehicle
%                           stops
%       tDetectionMax_s     Maximum time to detect the hard stop
% 
% Outputs: 
%   result:       1 if a valid emergency stop was detected, 0 otherwise

result = 1; 

disp(' ');
disp('------------------------------');
disp('Check Hard Stop: ');
disp('------------------------------');
% find timestamp where vehicle has gone to hard stop 
idxDetect = find(uint16(debug.debug_mloc_statemachine_debug_TUMVehicleState.Data) == 60, 1);
if isempty(idxDetect)
    disp('The vehicle did not detect the hard stop'); 
    result = 0; 
    return 
else
    tFailuredDetected_s = debug.debug_mloc_statemachine_debug_TUMVehicleState.Time(idxDetect);
    disp(['The vehicle took ' num2str(tFailuredDetected_s - tFailure_s) ' s to detect the failure.']); 
end
% check if fault detection time was within spec 
if(tFailuredDetected_s <= 0 && tFailuredDetected_s >= PerformanceSpec.tDetectionMax_s)
    disp('The vehicle did not detect the emergency within the specified time'); 
    result = 0; 
    return
end
% find point where failure was detected 
idxDetect_gt = find(ground_truth.SimRealState_ax_mps2.Time > tFailuredDetected_s, 1); 
% find timestamp where velocity has gone to new stopped 
% for very low velocites, the stop controller does not behave precisly in
% sim, therefore its tested against roughly 5kph. 
idxStop = find(ground_truth.SimRealState_vx_mps.Data(idxDetect_gt:end) < 2, 1) + idxDetect_gt; 
if isempty(idxStop)
    disp('The vehicle did not stop.'); 
    result = 0; 
    return
else
    tStop_s = ground_truth.SimRealState_vx_mps.Time(idxStop); 
    disp(['The vehicle took ' num2str(tStop_s - tFailure_s) ' s to stop.']); 
end
% check if control performance was ok 
meanAccX = mean(ground_truth.SimRealState_ax_mps2.Data(idxDetect_gt:idxStop));
disp(['The mean longitudinal accleeration was: ' num2str(meanAccX)]); 
if(meanAccX > PerformanceSpec.AccxMin_mps2)
    disp('The vehicle did not stay within the specified deceleration bounds.'); 
    result = 0; 
    return
end

disp(' ');
disp('All checks passed.');