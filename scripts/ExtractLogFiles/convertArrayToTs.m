function data = convertArrayToTs(array, descriptionfile)

% Author: Alexander Wischnewski     Date: 21-12-2018
% 
% Description: 
%   parses an array from the logs resulting from the Scope Debugging
%   mechanism of Simulink. The output is a set of timeseries. 
% 
% Input: 
%   array:              Log data array, First column: time, All other: data 
%   descriptionfile:    Description file which holdes the timeseries names in
%                          comma separated structure. 
% Output: 
%   data:               Struct with all timeseries in there

% load description file
fid = fopen(descriptionfile, 'r');
% iterate through file and array
i = 2; 
while(~feof(fid))
    name = fgetl(fid); 
    data.(name) = timeseries(array(:, i), array(:, 1),...
        'Name', name); 
    i = i + 1;
end
fclose(fid);
