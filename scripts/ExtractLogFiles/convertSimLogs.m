function [debug, SimRealState] = convertSimLogs(logsout, save_filename)  
%
%% Documentation
% 
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de) 
% 
% Start Date:   26.01.2019
% 
% Description:  converts simulation log data to common log data structure
%               which can be read by debug tool. 
% Inputs: 
%   logsout:        Structure created by simulink for logged data 
%   save_filename:  File where data is saved to (can be left blank) 
% 
% Outputs: 
%   debug:          debug data structure
%   SimRealState:   ground truth data structure
    
    disp('Start log conversion ...'); 
    names = getElementNames(logsout); 
    for i = 1:1:numElements(logsout)
        eval([names{i} ' = struct2sims(logsout.getElement(''' names{i} ''').Values, ''' names{i} ''');']);
    end
    % only save data to file if there is a second function argument
    if(nargin == 2) 
        save(save_filename, names{:}); 
    end
    disp('Conversion successful!'); 
    
end