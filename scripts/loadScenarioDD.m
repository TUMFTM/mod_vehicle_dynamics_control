function loadScenarioDD(filename_DD)
%
% Author:
%   Leonhard Hermansdorfer      Date: 03.12.2020
%
% Description:
%   function used to load a scenario into the raceline data dictionary which is then used
%   for trajectory planning emulation.
%
% Input: 
%   filename_DD:        name of data dictionary where to load scenario from;
%                       this file must be created via loadRaceline()


%% read csv-file which contains the raceline
    
    % add .sldd to data dictionary name if not already included
    if ~contains(filename_DD, ".sldd")
        filename_DD = strcat(filename_DD,'.sldd');
    end

    % raise error when no raceline file with specified name is found
    if isempty(which(filename_DD))
        error(strcat('No raceline data dictionary found with name: "', filename_DD, '"!'))
    end


%% assign struct to data dictionary

    DDname = "raceline.sldd";
    
    try

        datadict_raceline = Simulink.data.dictionary.open(DDname);
        datadict_scenario = Simulink.data.dictionary.open(filename_DD);

        % delete old data dictionary content
        dDataSectObj_racelineDD = getSection(datadict_raceline,'Design Data');
        list_struct_entries = find(dDataSectObj_racelineDD, 'DataSource', DDname);

        % delete all data dict entries of type struct
        for i=1:length(list_struct_entries)
            deleteEntry(dDataSectObj_racelineDD, list_struct_entries(i).Name)
        end
        
        % write raceline, start position and raceline file info to data dictionary
        disp(strcat('Write scenario of "', filename_DD, '" to raceline data dictionary...'));
        
        % get data of scenario data dictionary
      	dDataSectObj_scenarioDD = getSection(datadict_scenario,'Design Data');
        list_struct_entries = find(dDataSectObj_scenarioDD, 'DataSource', filename_DD);

     	% copy all data dict entries to target data dict
        for i=1:length(list_struct_entries)
            temp = getEntry(dDataSectObj_scenarioDD, list_struct_entries(i).Name);
            addEntry(dDataSectObj_racelineDD, list_struct_entries(i).Name, getValue(temp));
        end
        
        addEntry(dDataSectObj_racelineDD, 'current_scenario', filename_DD)
        
        % save and close data dictionary
        datadict_raceline.saveChanges()
        datadict_raceline.close(); 
        datadict_scenario.close();
        
    catch e
        warning(['Something went wrong during configuration of ' ...
                'raceline data dictionary']);
        disp('******************************************');
        disp('Exception: ');
        disp(getReport(e))
        disp('******************************************');
        
    end
    
end
