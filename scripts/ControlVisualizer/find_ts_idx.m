function [idxStart, idxEnd] = find_ts_idx(ts, tStart, tEnd)
    % find indices for starting and ending time points
    if(tStart > ts.Time(end))
      warning(['Start time is greater than last point in time for timeseries: ' ts.Name]); 
      idxStart = -1; 
      idxEnd = -1; 
      return; 
    else
      idxStart = find(ts.Time >= tStart, 1); 
    end
    if(tEnd < ts.Time(1))
      warning(['End time is smaller than first point in time for timeseries: ' ts.Name]); 
      idxStart = -1; 
      idxEnd = -1; 
    else
      idxEnd = find(ts.Time <= tEnd, 1, 'last'); 
    end
    if(tStart >= tEnd) 
      warning(['Start time is larger than end time']); 
      idxStart = -1; 
      idxEnd = -1; 
    end
end
