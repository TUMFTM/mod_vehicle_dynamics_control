function getSGLogs(tg, folder)

% Author: Alexander Wischnewski     Date: 21-12-2018
% 
% Description: 
%   copies the log data from the speedgoat to the host computer and
%   converts them to the .mat format 
% 
% Input: 
%   tg:     Speedgoat target object
%   folder: If given, the files are copied to the speciefied folder.
%           Otherwise the current folder is used. 

% standard folder for logs 
std_folder = 'C:/RoboSim/Logs/';
% check if standard folder exits 
if(~exist(std_folder, 'dir'))
    mkdir(std_folder); 
end

% configure time formats for automatic folder creation
format_day = 'yy_mm_dd';
format_time = 'HH_MM_SS'; 

% if no target folder was specified, use standard folder 
if(nargin<2)
    % check if a folder with the current date exists 
    if(~exist([std_folder datestr(now, format_day)], 'dir'))
       % create a new folder if this does not exits 
       mkdir([std_folder datestr(now, format_day)]); 
    end
    % create a new folder with the current time stamp 
    folder = [std_folder datestr(now, format_day) '/'...
        datestr(now, format_time)]; 
    mkdir(folder); 
end

% get ftp object on target
ftp = SimulinkRealTime.openFTP(tg); 

% switch to logs folder
cd(ftp,'C:\Logs\');

% get directory of log files
d = dir(ftp);

% retrieve all log files into current folder
disp('Copy files from RMC');
for i = 3:numel(d)
    fileName = [d(i).name];
    disp(['Copy ' fileName]);
    mget(ftp, fileName, folder);
end

disp('Convert data to .mat'); 
old_folder = pwd; 
cd(folder); 
convertSGLogs(pwd);
cd(old_folder); 

disp('Finished');








