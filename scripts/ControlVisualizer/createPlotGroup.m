function plotGroup = createPlotGroup(analysisdata, analysismode, plotdef)
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
            switch analysismode
                case {'debug','real_physics'}
                    % load timeseries from debug file
                    ts_temp = getfield(analysisdata, subplotdef.ts{i});
                    % set plot name
                    ts_temp.set('Name', subplotdef.labels{i});
                    % write to data structure
                    ts_array{end+1} = ts_temp;
                case 'sim_physics'
                    % divide string in different parts
                    path=strsplit(subplotdef.ts{i},'.');
                    currentpath=analysisdata;
                    % follow path and load timeseries from sim_physics file
                    for j=1:numel(path)
                        ts_temp = getfield(currentpath, path{j});
                        currentpath=ts_temp;
                    end
                    % reshape sim_physics mxnxX timeseries structure into 1xX timeseries structure and write to data structure
                    ts_array{end+1} = timeseries(reshape(ts_temp.Data(str2num(subplotdef.index1{i}),str2num(subplotdef.index2{i}),:),size(ts_temp.Time)),ts_temp.Time,'Name',subplotdef.labels{i});
            end
        catch 
            disp(['Could not find ' subplotdef.ts{i} ' in data file']); 
        end 
    end
    % write everything to the plot group
    plotGroup{sp_y, sp_x} = ty_axes(ts_array, {getfield(subplotdef,'ylabel')});
end



