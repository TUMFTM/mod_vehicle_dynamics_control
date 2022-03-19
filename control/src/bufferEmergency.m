function [ValidEmergencyTrajectory] = bufferEmergency(LatestEmergencyTrajectory, ...
    LatestEvaluationID, LatestEvaluationValid_b)
%___________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   20.10.2020
% 
% Description:  buffers last emergency trajectories and outputs latest valid trajectory
%         
% Inputs:
%   LatestEmergencyTrajectory   Most recent emergency trajectory received by controller
%   LatestEvaluationID          ID of last evaluated trajectory
%   LatestEvaluationValid_b     True if last evaluated trajectory is OK
%
% Outputs: 
%   ValidEmergencyTrajectory    Most recent valid emergency trajectory

% define sizes 
nBuffer = 10;       % buffer
nTrajPoints = 50;   % number of points per trajectory 

% create and initialize static variables 
persistent buffer buffer_valid last_valid_trajectory
if isempty(buffer)
    InitTrajectory.LapCnt = uint32(0); 
    InitTrajectory.TrajCnt = uint32(0);
    InitTrajectory.s_loc_m = zeros(nTrajPoints, 1);
    InitTrajectory.s_glob_m = zeros(nTrajPoints, 1);
    InitTrajectory.x_m = zeros(nTrajPoints, 1);
    InitTrajectory.y_m = zeros(nTrajPoints, 1);
    InitTrajectory.psi_rad = zeros(nTrajPoints, 1); 
    InitTrajectory.kappa_radpm = zeros(nTrajPoints, 1);
    InitTrajectory.v_mps = zeros(nTrajPoints, 1);
    InitTrajectory.ax_mps2 = zeros(nTrajPoints, 1); 
    InitTrajectory.banking_rad = zeros(nTrajPoints, 1); 
    InitTrajectory.ax_lim_mps2 = zeros(nTrajPoints, 1); 
    InitTrajectory.ay_lim_mps2 = zeros(nTrajPoints, 1);
    InitTrajectory.tube_r_m = zeros(nTrajPoints, 1);
    InitTrajectory.tube_l_m = zeros(nTrajPoints, 1);
    buffer = repmat(InitTrajectory, 1, nBuffer); 
    buffer_valid = zeros(nBuffer, 1, 'logical'); 
    last_valid_trajectory = InitTrajectory; 
end

% update buffer if newer trajectory is available
if(LatestEmergencyTrajectory.TrajCnt~=buffer(1).TrajCnt)
    for i = nBuffer:-1:2
       buffer(nBuffer) = buffer(nBuffer-1);  
    end
    buffer(1) = LatestEmergencyTrajectory; 
    buffer_valid(1) = false; 
end

% label trajectory which received a valid
for i = 1:1:nBuffer
    if(buffer(i).TrajCnt==LatestEvaluationID && LatestEvaluationValid_b)
        buffer_valid(i) = true; 
    end
end

% check if any trajectory is valid
if(any(buffer_valid))
    EM_valid = true;
    % find index of latest valid trajectory
    % loop required due to code generation restrictions
    idx_valid = 0; 
    for i = 1:1:length(buffer_valid) 
       if(buffer_valid(i))
           idx_valid = i; 
           break; 
       end
    end
    last_valid_trajectory = buffer(idx_valid); 
else
    EM_valid = false;
end
ValidEmergencyTrajectory = last_valid_trajectory; 
