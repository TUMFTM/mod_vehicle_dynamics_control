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
projectRoot_vdc = project.RootFolder;

% store current path
currentPath = pwd;
% switch to project root
cd(projectRoot_vdc);

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
    cd('..');
    end
    disp('Build folder setup done.');
catch e
    disp('Exception: ');
    disp(getReport(e))
    error('Setup of build folder failed.');
end

try
    % make sure that one parameter set is loaded
    configureVDCModule('pa');
    configureVDCBuildModelConfig('ERT');
    % configure vehicle dynamics simulation
    configureSimBuildModelConfig('ERT');
    configureSimParams('pa');
    % configure scenario
    loadRaceline('LVMS_GPS');
catch e
    disp('Exception: ');
    disp(getReport(e))
    error('Parameter setup failed.');
end

try
    % locate standard toolchain
    disp('Start building project dependencies ...');
    cc = mex.getCompilerConfigurations('C');
    % build project dependencies
    if(contains(cc.Name, 'Microsoft'))
        disp('Found Microsoft Visual C++ as standard compiler ... ');
        mex misc/osqp/osqp_sfun.c misc/osqp/osqp_wrapper.c -Lmisc/osqp/lib/ -losqp_vs_x64 -outdir build -silent -DWINDOWS_BUILD
    else
        if (contains(cc.Name, 'gcc'))
            disp('Found gcc as standard compiler ... ');
            % disable warnings at this point to not worry beginner users for "accepted" flaws in the code
            mex CFLAGS='$CFLAGS -w' misc/osqp/osqp_sfun.c misc/osqp/osqp_wrapper.c -Lmisc/osqp/lib/ -losqp -ldl -outdir build -silent
        else
            disp('Found MinGW as standard compiler ... ');
            % disable warnings at this point to not worry beginner users for "accepted" flaws in the code
            mex CFLAGS='$CFLAGS -w' misc/osqp/osqp_sfun.c misc/osqp/osqp_wrapper.c -Lmisc/osqp/lib/ -losqp_mingw -outdir build -silent -DWINDOWS_BUILD
        end
    end

    disp('Building of project dependencies successful ...');
    % copy tlc files to build folders due to path restrictions in code generation
    copyfile('misc/osqp/osqp_sfun.tlc', 'build/osqp_sfun.tlc');
catch e
    disp('Exception: ');
    disp(getReport(e))
    disp('Make sure that you have a compiler installed and setup for use with MATLAB using: mex -setup');
    error('Building of project dependencies failed ...');
end

myCacheFolder = fullfile(projectRoot_vdc, 'build');
myCodeFolder = fullfile(projectRoot_vdc, 'build');

Simulink.fileGenControl('set',...
    'CacheFolder', myCacheFolder,...
    'CodeGenFolder', myCodeFolder,...
    'createDir', true)

% go back to original path
cd(currentPath);
