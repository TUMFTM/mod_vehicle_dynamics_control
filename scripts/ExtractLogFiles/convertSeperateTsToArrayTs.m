function [data] = convertSeperateTsToArrayTs(debug_slow)
% Author: Salih Guemues     Date: 06-06-2020
% 
% Description: 
%   merges the set of timeseries of scalars belonging to array dimensional 
%   signal from the logs resulting from the Scope Debugging
%   mechanism of Simulink. The output is a struct including a set of timeseries. 
% 
% Input: 
%   debug_slow:    Log data struct, with vector-signals being seperated in 
%                  to scalar timeseries  
% Output: 
%   data:          Struct with all timeseries in there (merged vectors)

% define struct for output
data = struct();
% get signal names of logs
names = fieldnames(debug_slow);
same_signal = 0;
for i=1:1:numel(names)
    % check if last charakter of fieldname is a number (--> array signal)
    if isnan(str2double(names{i}(end)))
        % last charakter not a number, scalar signal can be copied
        % extend signal name by "slow" for the output-struct
        data.(insertAfter(names{i},"debug_","slow_")) = debug_slow.(names{i});
        same_signal = 0;
    else
        % last charakter is number --> array signal has to be merged
        % if next entry does not belong to same signal (same_signal==0) or 
        % if current entry is first array-signal-entry, create new
        % struct-entry corresponding to the array-signal
        if ~same_signal
            % get signal name without increment (get "a" instead of "a_1")
            tmp_name = names{i}(1:end-2);
            len_tmp_name = numel(tmp_name);
            % extend signal name by "slow" for the output-struct
            tmp_name_slow = insertAfter(tmp_name,"debug_","slow_");
            tmp_TS = debug_slow.(names{i});
            tmp_TS.name = tmp_name_slow;    % rename timeseries to array-signal-name   
            data.(tmp_name_slow) = tmp_TS;  % store timeseries in output
        else
            % concatenate signals back to array and store in output-struct
            data.(tmp_name_slow).Data = [data.(tmp_name_slow).Data, debug_slow.(names{i}).Data(:)];
        end
        if i<(numel(names)-same_signal)
            % get signal-name of next logged signal
            tmp_name_next = names{i+1}(1:end-2);
            len_tmp_name_next = numel(tmp_name_next);
            % check if next signal-name is shorter, if yes, not same signal
            if len_tmp_name > len_tmp_name_next     
                same_signal = 0;
            elseif strcmp(names{i}(1:len_tmp_name), names{i+1}(1:len_tmp_name))
            % if next signal-name is equal to current -> belongs to same
            % array-signal (check necessary, since two array signals with 
            % the same name-length could have been logged in succession)
            % e.g. a_1; a_2; a_3; b_1; b_2; b_3
                same_signal = 1;  
            else
                same_signal = 0;
            end
        end
    end
end
end