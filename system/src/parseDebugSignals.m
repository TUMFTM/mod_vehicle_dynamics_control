function ErrorCode = parseDebugSignals(mvdc_path_matching_debug, mvdc_trajectory_driver_debug)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   31.07.2019
% 
% Description:  generates error codes based on debug signals and buffers them until they are send
%               to the outside. 
%         
% Inputs:
%   mvdc_path_matching_debug        Debug data structure for path matching
%   mvdc_trajectory_driver_debug    Debug data structure for trajectory driver
%
% Outputs: 
%   ErrorCode                       error code integer

ErrorCode = uint32(0); 

%% Fault Detection Logic
% Error Code - 001 - Performance Trajectory not ok 
if(~mvdc_path_matching_debug.PerformanceTrajOK_b)
    ErrorCode = bitset(ErrorCode, 1); 
end

% Error Code - 002 - Emergency Trajectory not ok 
if(~mvdc_path_matching_debug.EmergencyTrajOK_b)
    ErrorCode = bitset(ErrorCode, 2); 
end

% Error Code - 003 - Path Deviation too high (below critical)
if(mvdc_trajectory_driver_debug.PathMatchingStatus == TUMPathMatchingState.PM_VALID_HIGHDEVIATION)
    ErrorCode = bitset(ErrorCode, 3);  
end

% Error Code - 004 - Path Deviation too high (above critical)
if(mvdc_trajectory_driver_debug.PathMatchingStatus == TUMPathMatchingState.PM_VALID_VERYHIGHDEVIATION)
    ErrorCode = bitset(ErrorCode, 4);  
end

% Error Code - 005 - Trajectory Comms not ok
if(~(mvdc_trajectory_driver_debug.Trajectories_CommsStatus == TUMHealthStatus.OK))
    ErrorCode = bitset(ErrorCode, 5);  
end

% Error Code - 006 - Strategy Comms not ok
if(~(mvdc_trajectory_driver_debug.Strategy_CommsStatus == TUMHealthStatus.OK))
    ErrorCode = bitset(ErrorCode, 6);  
end

end