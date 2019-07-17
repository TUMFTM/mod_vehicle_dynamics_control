function result = analyzeNormalOperation(debug, ControlSpec)

% Author: Alexander Wischnewski     Date: 24-02-2019
% 
% Description: 
%   scans whether the vehicle operates normally
% 
% Input: 
%   debug:              Debug structure of the vehicle
%   ground_truth:       Physical ground truth data from simulation
%   ControlSpec:        Structure specifying KPIs for controllers
% 
% Outputs: 
%   result:       1 if behavior is within specifications, 0 otherwise

result = 1; 

[nLaps, CP] = analyzeTrackingPerformance(debug, false, false);
        
disp(' ');
disp('------------------------------');
disp('Check Normal Operation: ');
disp('------------------------------');
disp('******************************************');
if(nLaps<ControlSpec.nLaps)
    result=0;
    disp('!ERROR: Not enough laps found');
else
    disp([num2str(nLaps) ' laps found']);
end
disp('******************************************');
if(any(CP.eTraj_lat_m_Peak(2:end)>=ControlSpec.eMaxLat_m))
    result=0;
    disp(['!ERROR: eTraj_lat_m_Peak > ' num2str(ControlSpec.eMaxLat_m)]);
end
disp('eTraj_lat_m_Peak = ');
disp(CP.eTraj_lat_m_Peak);
disp('******************************************');
if(any(CP.eTraj_v_mps_Peak(2:end)>=ControlSpec.eMaxVel_mps))
    result=0;
    disp(['!ERROR: eTraj_v_mps_Peak > ' num2str(ControlSpec.eMaxVel_mps)]);
end
disp('eTraj_v_mps_Peak = ');
disp(CP.eTraj_v_mps_Peak);
disp('******************************************');
if(any(debug.debug_mloc_statemachine_debug_TUMVehicleState.Data>30))
    result=0;
    disp(['!ERROR: debug_mloc_statemachine_debug_TUMVehicleState > 30']);
else
    disp(['No emergency situation was found']);
end
disp('******************************************');

if result == 1
    disp(' ');
    disp('All normal operation checks passed.');
end
