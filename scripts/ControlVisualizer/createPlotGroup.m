function plotGroup = createPlotGroup(debug, plotdef)
%__________________________________________________________________________
%% Documentation       
%
% Authors:      Alexander Wischnewski (alexander.wischnewski@tum.de)
% 
% Start Date:   31.10.2018
% 
% Description:  takes a json string in matlab format and creates the corresponding 
%               figure object based on the timeseries which are specified
%               in the json 
% 
% Inputs: 
%   debug       structure with debug data timeseries 
%   plotdef     json structure which specifies final plot layout (including
%                   subplots) 
% 
% Outputs: 
%   plotGroup   Cell array with tyaxes plot definitions 


% this function creates a plot group from a json string definition 

% empty plot group 
plotGroup = {}; 
% get number of fields 
fields = fieldnames(plotdef);

for i = 1:1:numel(fields)
    % get subplot definition 
    subplotdef = getfield(plotdef, fields{i});
    % get coordinates of final subplot
    sp_x = str2num(getfield(subplotdef, 'posx'));
    sp_y = str2num(getfield(subplotdef, 'posy'));
    % create cell array for timeseries 
    ts_array = {}; 
    for i = 1:1:numel(subplotdef.ts)
        try
            % load timeseries from debug file
            ts_temp = getfield(debug, subplotdef.ts{i});
            % set plot name
            ts_temp.set('Name', subplotdef.labels{i});
            % write to data structure
        catch 
            disp(['Could not find ' subplotdef.ts{i} ' in data file']); 
        end
        ts_array{i} = ts_temp; 
    end
    % write everything to the plot group
    plotGroup{sp_y, sp_x} = ty_axes(ts_array, {getfield(subplotdef,'ylabel')});
end



