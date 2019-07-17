function loadScenario(scenario, frictionmodel)
% Author: Alexander Wischnewski     Date: 21-12-2018
%         Thomas Herrmann
% 
% Description: 
%   loads a scenario from the tracks folder to the necessary data
%   dictionary and configures it appropriatly.
%   'frictionmodel' can optionally be filled.
% 
% Input: 
%   scenario:   name of the scenario to be loaded 

% open scenario data dictionary 
DD = Simulink.data.dictionary.open('ScenarioDefinition.sldd');
dDataSectObj = getSection(DD,'Design Data');
% does var_fric already exist?
if ~exist(dDataSectObj, 'activate_VariableFriction')
    addEntry(dDataSectObj, 'activate_VariableFriction', 0);
end
    activate_VariableFriction_Obj = getEntry(dDataSectObj, ...
        'activate_VariableFriction');
    setValue(activate_VariableFriction_Obj, 0); % set var_fric to 'off'

% clear all subreferences 
for i = 1:length(DD.DataSources)
    % Do not put 'i' as an index here since number of entries gets reduced!
    removeDataSource(DD, DD.DataSources{1}); 
end

% load new scenario
addDataSource(DD, [scenario '.sldd']);
% add friction maps to scenario dictionary if argument is filled
if nargin > 1
    addDataSource(DD, [frictionmodel '.sldd']);
    setValue(activate_VariableFriction_Obj, 1); % set var_fric to 'on'
end
saveChanges(DD)