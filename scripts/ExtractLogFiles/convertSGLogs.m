%%
% converts speedgoat logs .dat files to matlab struct and write it to the specified file
% in the 'debug' structure 
%
% folder: specify the folder which contains the .dat files
% save_filename: specify the output file name
% 
% second argument is not mandatory, just leave it empty and it will use the folder name 
%
% example: go to the folder where the log files are located and type: 
%             convertSGLogs(pwd); 
%%
function convertSGLogs(folder, save_filename)
  % check if file name was specified
  if(nargin < 2) 
    % take folder name as filename 
    out = regexp(folder,'\','split'); 
    % check if last character is \
    if(strcmp(folder(end), '\'))
      save_filename = out{end-1};
    else
      save_filename = out{end};
    end
  end
  % get all speedgoat logs in folder
  d = dir(folder); 
  % iterate via all data and convert to matlab struct 
  for i = 3:numel(d)
    % check file ending 
    [filepath, filename, fileext] = fileparts(d(i).name); 
    if(~strcmp(fileext, '.DAT') && ~strcmp(fileext, '.dat'))
      % if file extension is not .dat continue to next file
      continue; 
    end
    disp(['Process ' d(i).name ' ...']); 
    % get all signals in the current file
    SIGdata = SimulinkRealTime.utils.getFileScopeData([folder, '/' d(i).name]);
    for j = 1:numel(SIGdata.signalNames)
        % retrieve string and split it 
        buf = strsplit(SIGdata.signalNames{j}, '/');
        signalTime = SIGdata.data(:, end);
        signalData = SIGdata.data(:, j);
        % check if currently parsing debug slow
        if(contains(d(i).name, 'SLOW'))
            % check if signal is vector valued
            if(length(buf) > 3)
                signal_name = [buf{3} '_' buf{4}(2:end)]; 
                debug_slow.(signal_name) = timeseries(signalData, signalTime); 
            else
                debug_slow.(buf{end}) = timeseries(signalData, signalTime); 
            end
        else
            debug.(buf{end}) = timeseries(signalData, signalTime); 
        end
    end
  end
  % postprocess vector valued slow logs 
  debug_slow = convertSeperateTsToArrayTs(debug_slow); 
  % write result to file 
  save(save_filename, 'debug', 'debug_slow', '-v7.3'); 
end