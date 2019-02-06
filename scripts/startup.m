% This script defines a project shortcut. 
%
% To get a handle to the current project use the following function:
%
% project = simulinkproject();
%
% You can use the fields of project to get information about the currently 
% loaded project. 
%
% See: help simulinkproject

project = simulinkproject;
projectRoot = project.RootFolder;

try
  % check if build folder exists and clean it up
  if(exist([projectRoot '/build'], 'dir'))
    rmdir([projectRoot '/build'], 's'); 
  end
  % create an empty build directory 
  mkdir([projectRoot '/build']); 
  disp('Build folder setup done.'); 
catch e
    disp('Exception: ');
    disp(getReport(e))
  error('Setup of build folder failed.'); 
end

myCacheFolder = fullfile(projectRoot, 'build');
myCodeFolder = fullfile(projectRoot, 'build');

Simulink.fileGenControl('set',...
    'CacheFolder', myCacheFolder,...
    'CodeGenFolder', myCodeFolder,...
    'createDir', true)
  