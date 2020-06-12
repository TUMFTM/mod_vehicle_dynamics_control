% tx_axes stores all information necessary to generate a nice looking 
% tx-Plot for possible multiple datasets. It provides methods 
% to generate this plot if a target axes is given. 
classdef xy_axes
   properties
    % these three cell arrays hold the data necessary to generate a xy-Plot
    % with possibly multiple dataseries. The arrangement of data has to match 
    % between these three. xvalues stores the timeseries holding the data
    % defining the x-Axis values, yvalues stores the timeseries holding the 
    % data defining the y-Axis values and names holds the associated strings 
    % which are displayed in the legend. 
    ts_list_xvalues
    ts_list_yvalues
    ts_list_names
    % stores the x and y axis label shared by all plots
    xaxis_string
    yaxis_string
    axis_equal
    legend_on
   end
   methods
    % constructor 
    function obj = xy_axes(ts_list_xvalues, ts_list_yvalues, ts_list_names, xaxis_string, yaxis_string, axis_equal, legend_on)
      obj.ts_list_xvalues = ts_list_xvalues; 
      obj.ts_list_yvalues = ts_list_yvalues; 
      obj.ts_list_names = ts_list_names; 
      obj.xaxis_string = xaxis_string; 
      obj.yaxis_string = yaxis_string; 
      obj.axis_equal = axis_equal; 
      obj.legend_on = legend_on; 
    end
    % plots all timeseries objects and setups the axes properties right
    function plot_all(obj, ax, tStart, tEnd) 
      % setup general properties
      grid(ax, 'on'); 
      hold(ax, 'on'); 
      % check if dataset is setup properly (all ts_list objects contain 
      % the same number of datasets)
      if(~((numel(obj.ts_list_xvalues) == numel(obj.ts_list_yvalues)) && ...
          (numel(obj.ts_list_xvalues) == numel(obj.ts_list_names))))
        warning('xy - Axes class is not setup properly. Stop plotting.'); 
        return; 
      end
      % initalize plot handles
      ph = []; 
      legend_bool = []; 
      ph_idx = 1; 
      % iterate via all print objects
      for i = 1:1:numel(obj.ts_list_xvalues)
        if(nargin==4)
          % if time points are given, specify them
          [new_ph, new_legend_bool] = obj.plot_xy(obj.ts_list_xvalues{i}, obj.ts_list_yvalues{i},...
            obj.ts_list_names{i}, ax, tStart, tEnd); 
          ph = [ph, new_ph]; 
          legend_bool = [legend_bool, new_legend_bool];
        else
          % otherwise plot complete dataset by suppression of time points
          [new_ph, new_legend_bool] = obj.plot_xy(obj.ts_list_xvalues{i}, obj.ts_list_yvalues{i},... 
          obj.ts_list_names{i}, ax);  
          ph = [ph, new_ph]; 
          legend_bool = [legend_bool, new_legend_bool];
        end
      end
      % tidy up
      if(obj.legend_on)
        legend(ax, ph(logical(legend_bool)), 'Location', 'best'); 
      end
      hold(ax, 'off'); 
      box(ax, 'on'); 
      if(obj.axis_equal)
        % if axis is specified to be equal
        axis(ax, 'square');
        axis(ax, 'equal');
      end
      xlabel(ax, obj.xaxis_string); 
      ylabel(ax, obj.yaxis_string); 
    end
   % helper function which takes care of time management 
   function [ph, legend_bool] = plot_xy(obj, ts_x, ts_y, Name, ax, tStart, tEnd)
    if(nargin~=7)
      % if not all aguments are given, assume that the timeseries is plotted completly
      tStart = ts_x.Time(1); 
      tEnd = ts_x.Time(end); 
    end
    % find start and end point indices
    [idxStart_x, idxEnd_x] = find_ts_idx(ts_x, tStart, tEnd);
    [idxStart_y, idxEnd_y] = find_ts_idx(ts_y, tStart, tEnd);
    if(any([idxStart_x, idxEnd_x, idxStart_y, idxEnd_y] == -1)) 
      % error in any of the specified indices, return to caller
      % error reporting is done by find_ts_idx
      return; 
    end
    % get data size for advanced processing
    data_x_size = size(ts_x.Data); 
    data_y_size = size(ts_y.Data); 
    % check if data dimension are equal
    if(data_x_size(2) ~= data_y_size(2)) 
      warning('Data series dimension do not match'); 
      return; 
    end
    % initialize plot handles which store the resulting plots to match 
    % with the legends later
    ph = []; 
    legend_bool = []; 
    ph_idx = 1; 
    if(data_x_size(2) > 1 && data_y_size(2) > 1)
    % check if both datasets have spatial domain 
      % plot data of all subdatasets
      hold(ax, 'on'); 
      % iterate via all spatial domain data and give a plot
      for j = idxStart_x:1:idxEnd_x
        % only add a display name for the first 
        if(j == idxStart_x) 
          ph(ph_idx) = plot(ax, ts_x.Data(j, :), ts_y.Data(j, :),...
            'DisplayName', Name); 
          legend_bool(ph_idx) = 1; 
        else
          % get color from first plot
          color = get(ph(1), 'Color'); 
          ph(ph_idx) = plot(ax, ts_x.Data(j, :), ts_y.Data(j, :), 'Color', color);
          legend_bool(ph_idx) = 0; 
        end
        ph_idx = ph_idx + 1; 
      end
    elseif(data_x_size(2) == 1 && data_y_size(2) == 1)
    % check if both datasets have no spatial domain 
      % plot timely data and configure axes 
      ph(ph_idx) = plot(ax, ts_x.Data(idxStart_x:idxEnd_x), ts_y.Data(idxStart_y:idxEnd_y),...
        'DisplayName', Name); 
      legend_bool(ph_idx) = 1; 
      ph_idx = ph_idx + 1; 
    else
      warning('One data series has spatial domain, the other does not. Nothing done for plotting'); 
    end
   end
 end
end