function [] = CreateNewTrack(csvName,startReadRow)
%__________________________________________________________________________
%% Documentation       
%
% Author:       Thomas Herrmann (thomas.herrmann@tum.de)
% 
% Start Date:   10.12.2018
% 
% Description: Put your .csv in opttraj-format into the folder 'tracks' and
% run this script. Delimiters to be used are semicolons ';'.
% A data dict will be created in 'tracks', too. You can 
% then add a reference to this data dict in your Simulink simulation.
% startReadRow: Argument '0' start form beginning of files,
% argument '1' skips first row and starts rading from row 2.
% 
% Inputs: Expected data-format of your csv-file is a
%         Matrix with dimensions n x 10 where n < 5000:
%         n is the number of valid data points
%         [s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2;...
%         delta_rad;F_N;ValidPointCnt]
% 
% Outputs:
%
%% Algorithm
numMaxDataPoints = 5000;

%% csvread
csvPath = strcat(csvName,'.csv');
% Start reading from row startReadRow
inM = dlmread(csvPath,';',startReadRow,0);

%% Create new data dict
dictPath = strcat(csvName,'.sldd');

% Check if dict already exists
try
    DictionaryObj = Simulink.data.dictionary.create(dictPath);
catch
    ik = 1;
    while exist(dictPath)
        ik = ik+1;
        % Change new DD name as long as it is already existing
        dictPath = strcat(csvName,num2str(ik),'.sldd');
    end
    DictionaryObj = Simulink.data.dictionary.create(dictPath);
    fprintf('Warning: DD specified has already existed.\nNew name for your DD is: ');
    disp(dictPath);
end
    
dDataSectObj = getSection(DictionaryObj,'Design Data');
%% Create new entries in the DD
% Raceline
addEntry(dDataSectObj,'Raceline',...
    struct('s_m',[],'x_m',[],'y_m',[],...
        'psi_rad',[],'kappa_radpm',[],'vx_mps',[],...
        'ax_mps2',[],'delta_rad',[],'F_N',[],...
        'ValidPointCnt',[],'s_m_end',[]));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Add data to DD object %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rlObj = getEntry(dDataSectObj,'Raceline');
rlObj_val = getValue(rlObj);
% To add a new struct-field (if necessary)
% [rlObj_val(:).anyNewStruct] = [];

size_inM = size(inM);
% To reduce size of inputMatrix to max. numDataPoints data points
dist = ceil(size_inM(1)/numMaxDataPoints);
% If start and end point would be the same, remove last data point from csv
% 
if mod(length(inM(1:dist:end,1)),dist) == 1 || dist == 1
    eofR = length(inM(1:end,1))-1;
else
    eofR = length(inM(1:end,1));
end
% New values for the datav dictionary
rlObj_val.s_m = inM(1:dist:eofR,1);
% s_m must be continuously increasing
rlObj_val.s_m(end+1:numMaxDataPoints) = ...
    linspace(ceil(rlObj_val.s_m(end))+1,...
    ceil(rlObj_val.s_m(end))+1+numMaxDataPoints-length(rlObj_val.s_m),...
    numMaxDataPoints-length(rlObj_val.s_m));
% TODO: end in eofR
rlObj_val.x_m = inM(1:dist:eofR,2);
    rlObj_val.x_m(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.x_m),1);
rlObj_val.y_m = inM(1:dist:eofR,3);
    rlObj_val.y_m(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.y_m),1);
rlObj_val.psi_rad = inM(1:dist:eofR,4);
    rlObj_val.psi_rad(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.psi_rad),1);
rlObj_val.kappa_radpm = inM(1:dist:eofR,5);
    rlObj_val.kappa_radpm(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.kappa_radpm),1);
rlObj_val.vx_mps = inM(1:dist:eofR,6);
    rlObj_val.vx_mps(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.vx_mps),1);
rlObj_val.ax_mps2 = inM(1:dist:eofR,7);
    rlObj_val.ax_mps2(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.ax_mps2),1);
rlObj_val.delta_rad = zeros(numMaxDataPoints,1);
    rlObj_val.delta_rad(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.delta_rad),1);
rlObj_val.F_N = zeros(numMaxDataPoints,1);
    rlObj_val.F_N(end+1:numMaxDataPoints) = ...
        zeros(numMaxDataPoints-length(rlObj_val.F_N),1);
rlObj_val.ValidPointCnt = length(inM(1:dist:eofR,1));
rlObj_val.s_m_end = rlObj_val.s_m(length(inM(1:dist:eofR,1)));
% Set new values into DD object and save changes
setValue(rlObj,rlObj_val);

% Copy to opponent vehicles
addEntry(dDataSectObj,'Raceline_Op1',...
    struct('s_m',rlObj_val.s_m,'x_m',rlObj_val.x_m,'y_m',rlObj_val.y_m,...
        'psi_rad',rlObj_val.psi_rad,'kappa_radpm',rlObj_val.kappa_radpm,'vx_mps',rlObj_val.vx_mps,...
        'ax_mps2',rlObj_val.ax_mps2,'delta_rad',rlObj_val.delta_rad,'F_N',rlObj_val.F_N,...
        'ValidPointCnt',rlObj_val.ValidPointCnt,'s_m_end',rlObj_val.s_m_end));
addEntry(dDataSectObj,'Raceline_Op2',...
    struct('s_m',rlObj_val.s_m,'x_m',rlObj_val.x_m,'y_m',rlObj_val.y_m,...
        'psi_rad',rlObj_val.psi_rad,'kappa_radpm',rlObj_val.kappa_radpm,'vx_mps',rlObj_val.vx_mps,...
        'ax_mps2',rlObj_val.ax_mps2,'delta_rad',rlObj_val.delta_rad,'F_N',rlObj_val.F_N,...
        'ValidPointCnt',rlObj_val.ValidPointCnt,'s_m_end',rlObj_val.s_m_end));
addEntry(dDataSectObj,'s0_m_Op1',10);
addEntry(dDataSectObj,'s0_m_Op2',20);
addEntry(dDataSectObj,'s0_m_Op3',30);
addEntry(dDataSectObj,'s0_m_Op4',40);
addEntry(dDataSectObj,'velGain_Op1',0);
addEntry(dDataSectObj,'velGain_Op2',0);
addEntry(dDataSectObj,'velGain_Op3',0);
addEntry(dDataSectObj,'velGain_Op4',0);
addEntry(dDataSectObj,'x0_vehiclepose_stm',...
    [rlObj_val.x_m(1),rlObj_val.y_m(1),rlObj_val.psi_rad(1)]);
addEntry(dDataSectObj,'x0_vehiclepose_Op1',...
    [rlObj_val.x_m(1),rlObj_val.y_m(1),rlObj_val.psi_rad(1)]);
addEntry(dDataSectObj,'x0_vehiclepose_Op2',...
    [rlObj_val.x_m(1),rlObj_val.y_m(1),rlObj_val.psi_rad(1)]);
addEntry(dDataSectObj,'x0_vehiclepose_Op3',...
    [rlObj_val.x_m(1),rlObj_val.y_m(1),rlObj_val.psi_rad(1)]);
addEntry(dDataSectObj,'x0_vehiclepose_Op4',...
    [rlObj_val.x_m(1),rlObj_val.y_m(1),rlObj_val.psi_rad(1)]);
addEntry(dDataSectObj,'x0_vehiclepose_dtm',...
    [rlObj_val.x_m(1),-1*rlObj_val.y_m(1),0,0,0,normalizeAngle((-pi/2)-rlObj_val.psi_rad(1))]);

%% Save new values to new data dict
fprintf('\nEntries in new DD are:\n');
DictionaryObj.listEntry;
saveChanges(DictionaryObj);

%% Adapt ValidPointCnt ini-value in trajecotry replay
% load_system('externalVehicles\models\trajectoryReplay.slx');
% hws = get_param(bdroot, 'modelworkspace');
% hws.assignin('ValidPointCnt_Op', rlObj_val.ValidPointCnt);

end