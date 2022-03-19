function [debug] = convertERTSimLogs(data_file, parser_config, save_file, slow_on)  

% Author: Alexander Wischnewski     Date: 21-12-2018
% 
% Description: 
%   converts the logs of a GRT target based debug file (e.g. compiled
%   binary simulation on windows or unix) into the common log format. 
% 
% Input: 
%   data_file:              Log data file
%   parser_config:          Parser config
%   save_file:              Output data file
%   slow_on:                true/false value for parsing extended slow
%   dataset
% Output: 
%   debug:                  Struct with all timeseries in there

    % check if file ending is given and modify load files accordingly
    if(endsWith(data_file, '.txt'))
        file_standard = data_file; 
        file_slow = [data_file(1:end-4) '_slow.txt']; 
    else
        file_standard = [data_file '.txt']; 
        file_slow = [data_file '_slow.txt']; 
    end
    % check if parser config ending is given and modify load files accordingly
    if(endsWith(parser_config, '.txt'))
        config_standard = parser_config; 
        config_slow = [parser_config(1:end-4) '_slow.txt']; 
    else
        config_standard = [parser_config '.txt']; 
        config_slow = [parser_config '_slow.txt']; 
    end
    % load and parse data from standard logging file
    try
        data = dlmread(file_standard, ';'); 
        % skip last data point as it might not be fully written!
        debug = convertArrayToTs(data(1:end-1, :), config_standard); 
        disp('Successfully parsed standard logging file!'); 
    catch
        disp('Could not parse standard logging file!'); 
        debug = 0; 
    end    
    
    % load and parse data from slow logging file
    if slow_on
        data_slow = dlmread(file_slow, ';'); 
        % skip last data point as it might not be fully written!
        debug_slow = convertArrayToTs(data_slow(1:end-1, :), config_slow); 
        debug_slow = convertSeperateTsToArrayTs(debug_slow);
        disp('Successfully parsed slow logging file!'); 
            % write to file if its wanted by the user
        if(nargin == 4)
          save(save_file, 'debug', 'debug_slow', '-v7.3'); 
        end
    else
        % write to file if its wanted by the user
        if(nargin == 4)
          save(save_file, 'debug','-v7.3'); 
        end
    end  
end