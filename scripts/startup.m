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

% store current path 
currentPath = pwd; 
% switch to project root
cd(projectRoot); 

try
  % cleanup build folder
  dinfo = dir('build'); 
  type = {dinfo.isdir}; 
  name = {dinfo.name}; 
  if(length(type) > 2)
      cd('build'); 
      for i = 3:1:length(type)
          if type{i} == 1
              rmdir(name{i}, 's'); 
          else
              % skip readme
              if(strcmp(name{i}, 'Readme.md'))
                  continue; 
              end
              delete(name{i}); 
          end
      end
  end
  cd('..'); 
  disp('Build folder setup done.'); 
catch e
    disp('Exception: ');
    disp(getReport(e))
  error('Setup of build folder failed.'); 
end

try
    % make sure that one parameter set is loaded
    configureVDCModule('pa');
    configureVDCBuildModelConfig('GRT'); 
    % configure vehicle dynamics simulation 
    configureSimBuildModelConfig('GRT'); 
    % configure scenario 
    loadScenario('Berlin2019'); 
catch e
    disp('Exception: ');
    disp(getReport(e))
  error('Parameter setup failed.'); 
end

myCacheFolder = fullfile(projectRoot, 'build');
myCodeFolder = fullfile(projectRoot, 'build');

Simulink.fileGenControl('set',...
    'CacheFolder', myCacheFolder,...
    'CodeGenFolder', myCodeFolder,...
    'createDir', true)
  
% go back to original path 
cd(currentPath); 