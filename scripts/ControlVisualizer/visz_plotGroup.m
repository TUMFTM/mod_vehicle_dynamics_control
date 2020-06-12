% This functions allows to visualize a group of ty and xy plots defined 
% via the classes ty_axes and xy_axes. 
% Inputs: 
%   data: Cell array arranged in the form the plots should be displayed, 
%           containing a ty_axes or xy_axes class at each position. 
%   fig:  Figure handle of figure which should be used
%   startTime: starting time of the plot
%   endTime: ending time of the plot
% 
% If the latter two are not specified, the complete dataset is plotted
% 
function visz_plotGroup(data, fig, startTime, endTime)
  try
    figure(fig); 
    % clean up figure 
    clf(fig); 
  catch 
    warning('Figure could not be opened'); 
    return; 
  end
  try
    % get it done
    data_size = size(data); 
    % flip data to achieve matching of data structure from and subplot form
    data_tmp = data';
    ax_cnt = 0; 
    for idx = 1:1:(data_size(1)*data_size(2))
      if(isempty(data_tmp{idx}))
        % skip this cell if it is empty in data 
        continue; 
      end
      ax_cnt = ax_cnt+1; 
      % create axes structure necessary to display data
      ax_store(ax_cnt) = subplot(data_size(1), data_size(2), idx); 
      if(nargin < 4) 
        data_tmp{idx}.plot_all(ax_store(ax_cnt)); 
      else
        % if time points are given plot cropped dataset
        data_tmp{idx}.plot_all(ax_store(ax_cnt), startTime, endTime); 
      end
    end
    linkaxes(ax_store, 'x'); 
  catch 
    warning(['Error during plotting of figure ' fig.Name]); 
    return; 
  end
  disp(['Plotting of figure ' fig.Name ' finished']); 
end