% ty_axes stores all information necessary to generate a nice looking 
% ty-Plot for possible multiple timeseries datasets. It provides methods 
% to generate this plot if a target axes is given. 
%
% It expects the following interface: 
% ty_axes(ts_list, yaxis_string) 
% with ts_list being a cell array of multiple time series to display and 
% yaxis_string being the label placed at the y axis. 
classdef ty_axes
   properties
    % cell array which holds multiple timeseries, arrangement does not matter
    ts_list
    % stores the y axis label shared by all timeseries
    yaxis_string
   end
   methods
    % constructor 
    function obj = ty_axes(ts_list, yaxis_string)
      obj.ts_list = ts_list; 
      obj.yaxis_string = yaxis_string; 
    end
    % plots all timeseries objects and setups the axes properties right
    function plot_all(obj, ax, tStart, tEnd) 
      % setup general properties
      grid(ax, 'on'); 
      hold(ax, 'on'); 
      xlabel(ax, 'Time in s'); 
      ylabel(ax, obj.yaxis_string{1}); 
      % initialize plot handles which store the resulting plots to match 
      % with the legends later
      ph = []; 
      legend_bool = []; 
      ph_idx = 1; 
      % iterate via all timeseries 
      for i = 1:1:numel(obj.ts_list)
        if(nargin==4)
          % if time points are given, specify them
          ph = [ph, obj.plot_ts(obj.ts_list{i}, ax, tStart, tEnd)]; 
          legend_bool(ph_idx) = 1; 
        else
          % otherwise plot complete timeseries by suppression of time points
          ph = [ph, obj.plot_ts(obj.ts_list{i}, ax)]; 
          legend_bool(ph_idx) = 1; 
        end
        ph_idx = ph_idx + 1; 
      end
      % tidy up
      if(numel(obj.ts_list) > 1) 
        % only display legend if more than one signal is present
        legend(ax, ph(logical(legend_bool)), 'Location', 'best'); 
      end
      box(ax, 'on'); 
      hold(ax, 'off'); 
    end
   % helper function which takes care of time management 
   function ph = plot_ts(obj, ts, ax, tStart, tEnd)
    if(nargin~=5)
      % if not all aguments are given, assume that the timeseries is plotted completly
      tStart = ts.Time(1); 
      tEnd = ts.Time(end); 
    end
    try 
      [idxStart, idxEnd] = find_ts_idx(ts, tStart, tEnd);      
      % plot data and configure axes 
      ph = stairs(ax, ts.Time(idxStart:idxEnd), ts.Data(idxStart:idxEnd), 'DisplayName', strrep(ts.Name,'_',' ')); 
    catch 
      warning(['Error during plotting of ' ts.Name]); 
    end
   end
 end
end