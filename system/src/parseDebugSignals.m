function ErrorCode = parseDebugSignals(mloc_statemachine_debug, ...
    mvdc_path_matching_debug, mvdc_trajectory_driver_debug, flag_s_request)
%_________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   31.07.2019
% 
% Description:  generates error codes based on debug signals and buffers them until they are send
%               to the outside. Error codes are set bitwise, therefore they have to be decoded
%               accordingly. 
%         
% Inputs:
%   mloc_statemachine_debug         Debug data structure for state machine
%   mvdc_path_matching_debug        Debug data structure for path matching
%   mvdc_trajectory_driver_debug    Debug data structure for trajectory driver
%   flag_s_request                  Debug flag from TMPC controller
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

% Error Code - 004 - Path Deviation too high (below critical)
if(mvdc_trajectory_driver_debug.PathMatchingStatus == TUMPathMatchingState.PM_VALID_HIGHDEVIATION)
    ErrorCode = bitset(ErrorCode, 3);  
end

% Error Code - 008 - Path Deviation too high (above critical)
if(mvdc_trajectory_driver_debug.PathMatchingStatus == TUMPathMatchingState.PM_VALID_VERYHIGHDEVIATION)
    ErrorCode = bitset(ErrorCode, 4);  
end

% Error Code - 016 - Trajectory Comms not ok
if(~(mvdc_trajectory_driver_debug.Trajectories_CommsStatus == TUMHealthStatus.OK))
    ErrorCode = bitset(ErrorCode, 5);  
end

% Error Code - 032 - Strategy Comms not ok
if(~(mvdc_trajectory_driver_debug.Strategy_CommsStatus == TUMHealthStatus.OK))
    ErrorCode = bitset(ErrorCode, 6);  
end

% Error Code - 064 - Controller not ok
if (~(mvdc_trajectory_driver_debug.ControllerStatus == TUMHealthStatus.OK))
    ErrorCode = bitset(ErrorCode, 7); 
end

% Error Code - 128 - Trajectory too short
if (flag_s_request)
    ErrorCode = bitset(ErrorCode, 8); 
end

% Error Code - 256 - Vehicle unstable
if (mloc_statemachine_debug.VehicleUnsafe)
    ErrorCode = bitset(ErrorCode, 9); 
end

% Error Code - 512 - Software stack not OK
if (~((mvdc_trajectory_driver_debug.Strategy_Status < 50) && (mvdc_trajectory_driver_debug.Strategy_Status >= 30)))
    ErrorCode = bitset(ErrorCode, 10); 
end

end