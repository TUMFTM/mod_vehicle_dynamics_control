function [lapTimes, AvgSolverTime, MaxSolverTime] = getLapTimes(numLaps)
% function getLapTimes
% Author:       Martin Euler         last update: 21-03-2020
% Description:  
%   function used to get the lap times
% Inputs/parameters:
%   numLaps: number of driven laps

% Outputs:
%   lapTimes: lap times
%   AvgSolverTime: average solver time
%   MaxSolverTime: max solver time
%% -------------------------- getLapTimes -------------------------------- %%
% import stored data
struct = importdata('SimTest925L.mat');
% init matrix
lapTimes = zeros(numLaps,1);
% get lap time data
lap_time = struct.debug.debug_mvdc_trajectory_driver_perf_LapTime_s.Data;
% get solver time
solver_time = struct.mvdc_tube_mpc_debug.mvdc_tube_mpc_debug_solver_runTime_s.Data(114:end);
% calc average solver time
AvgSolverTime = mean(solver_time);
% calc max solver time
MaxSolverTime = max(solver_time);
% get lap times
k = 1;
for i = 1:length(lap_time)-1    
    if lap_time(i) > lap_time(i+1)
        k = k+1;
        % ignore first lap time because laptime is set to zero when car
        % starts to drive
        if k > 2
            lapTimes(k-2) = lap_time(i);
        end        
    end
end


        