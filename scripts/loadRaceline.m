function loadRaceline(filename_raceline, DDname)
%
% Author:
%   Leonhard Hermansdorfer      Date: 03.12.2020
%
% Description:
%   function used to load a raceline for trajectory planning emulation.
%
% Input: 
%   filename_raceline:  name of file containing the raceline to be loaded
%                       data format:
%                       - delimiter: ';'
%                       - header char: '#'
%                       - max. number of s-coordinates: Variable "int_max_datapoints"
%                       - contained data:
%                       [s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2(;banking_rad)]
%   DDname (optional):  name of data dictionary where to save raceline;
%                       default is "raceline.sldd"


%% user input

% fixed length of raceline data arrays
int_max_datapoints = 5000;

% number of vehicles (necessary for starting and parking position calculation)
int_count_vehicles = 10;


%% read csv-file which contains the raceline

    % name of data dictionary where to write raceline
    if ~exist("DDname", "var")
        DDname = "raceline.sldd";
    end
    
    % add .sldd to data dictionary name if not already included
    if ~contains(DDname, ".sldd")
        DDname = strcat(DDname,'.sldd');
    end

    % add .csv to filename if not already included
    if ~contains(filename_raceline, ".csv")
        filename_raceline = strcat(filename_raceline,'.csv');
    end

    % raise error when no raceline file with specified name is found
    if isempty(which(filename_raceline))
        error(strcat('No raceline file found with name: "', filename_raceline, '"!'))
    end

    % find first line without comment char '#' to determine where to start
    % reading the raceline file
    fid = fopen(filename_raceline);
    tline = fgetl(fid);

    int_row_start = 0;

    while contains(tline, '#')
        tline = fgetl(fid);
        int_row_start = int_row_start + 1;
    end

    % start reading raceline file at first row which contains relevant data
    raceline_data = dlmread(filename_raceline,';',int_row_start,0);

    % load corresponding racetrack file if available
    loadRacetrack(strcat('traj_ltpl_cl_', filename_raceline));
    

%% preprocess and reshape raceline data

    % get length of original raceline
    size_raceline_init = size(raceline_data);

    % get factor for reducing number of data points in output raceline array if it is above specified limit
    dist = ceil(size_raceline_init(1) / int_max_datapoints);

    % discard last row, if x,y-coordinates are the same (trajectory is closed)
    if abs(raceline_data(1, 2) - raceline_data(end, 2)) <= 0.1 && ...
        abs(raceline_data(1, 3) - raceline_data(end, 3)) <= 0.1

        int_count_validpoints = size_raceline_init(1) - 1;

    else
        int_count_validpoints = size_raceline_init(1);

    end

    % crop raceline data to fit maximum number of data points (rows)
    raceline_data_cropped = raceline_data(1:dist:int_count_validpoints, :);
    size_raceline_cropped = size(raceline_data_cropped,1);
    
    % get mean distance of cropped raceline for interpolation of
    % s-coordinate
    mean_sdistance = mean(diff(raceline_data_cropped(:, 1)));

    % fill raceline array with zeros to match max number of data points (rows)
    raceline_data_out = [raceline_data_cropped;
                zeros((int_max_datapoints - size_raceline_cropped), size_raceline_init(2))];

    % create Raceline struct which is then stored in raceline data dictionary
    Raceline.s_m = raceline_data_out(:,1);
    % extrapolate s-coordinate to end of raceline array
    Raceline.s_m(size_raceline_cropped+1:int_max_datapoints, 1) = ...
        (Raceline.s_m(size_raceline_cropped) + [1:(int_max_datapoints-size_raceline_cropped)] * mean_sdistance)';

    Raceline.x_m = raceline_data_out(:,2);
    Raceline.y_m = raceline_data_out(:,3);
    Raceline.psi_rad = raceline_data_out(:,4);
    Raceline.kappa_radpm = raceline_data_out(:,5);
    Raceline.vx_mps = raceline_data_out(:,6);
    Raceline.ax_mps2 = raceline_data_out(:,7);

    Raceline.ValidPointCnt = length(raceline_data(1:dist:int_count_validpoints,1));
    Raceline.s_m_end = Raceline.s_m(Raceline.ValidPointCnt);
    
    % if no banking information is available, write zeros
    try
        Raceline.banking_rad = raceline_data_out(:,8);
    catch
        Raceline.banking_rad = zeros(length(raceline_data_out(:,1)), 1);
    end


%% calculate default starting (activated vehicles) and parking position (deactivated vehicles)

    % TODO: add options as function arguments (e.g. "chain", "grid")

    % starting position
    s_distance_m = 15;
    
    x0_vehiclepose_start = [Raceline.x_m(1), Raceline.y_m(1), Raceline.psi_rad(1); ...
                            zeros((int_count_vehicles - 1), 3)];
    
    for i=2:int_count_vehicles
        
        s_target = Raceline.s_m(int_count_validpoints + 1) - s_distance_m * (i - 1);
        abs(Raceline.s_m - s_target);
        idx = find(abs(Raceline.s_m - s_target)==min(abs(Raceline.s_m - s_target)));
        
        x0_vehiclepose_start(i, :) = [Raceline.x_m(idx), Raceline.y_m(idx), Raceline.psi_rad(idx)];
        
    end
    
    % parking position
    s_distance_toraceline_m = 15;
    s_distance_betweenvehicles_m = 7;

    direction_raceline = [Raceline.x_m(2) - Raceline.x_m(1), ...
                          Raceline.y_m(2) - Raceline.y_m(1)];
    direction_raceline = direction_raceline ./ sqrt(sum(direction_raceline.^2));
    normalvec_direction_raceline = cross([direction_raceline, 0], [0,0,1]);
    normalvec_direction_raceline = normalvec_direction_raceline(1:2);
    x0_vehiclepose_park = zeros(int_count_vehicles, 3);
    
    for i=1:int_count_vehicles
        
        x0_vehiclepose_park(i, :) = ...
            [[Raceline.x_m(1), Raceline.y_m(1)] ...
             + normalvec_direction_raceline * s_distance_toraceline_m ...
             + -1 * s_distance_betweenvehicles_m * direction_raceline* (i - 1), ...
             Raceline.psi_rad(1) + pi/4];
            
    end

%% assign struct to data dictionary

    path2module = split(which(mfilename), "mod_control");
    path2racelines = strcat(path2module{1}, "mod_control/racelines/");
    filepath2racelineDD = strcat(path2racelines, DDname);

    try

        % open default data dictionary
        if DDname == "raceline.sldd"
            datadict = Simulink.data.dictionary.open(DDname);
        
        % create new data dictionary with custom name
        else
            
            % check if data dictionary with custom name already exists
            if exist(filepath2racelineDD, "file")
                i = 1;

                while exist(filepath2racelineDD, "file")
                    DDname_temp = split(DDname,'.sldd');
                    DDname_temp = DDname_temp(1);
                    
                    filepath2racelineDD = strcat(path2racelines, DDname_temp, num2str(i),'.sldd');
                    
                    i = i + 1;

                end
                
                warning(strcat('data dictionary already exists. New name is ', filepath2racelineDD));

            end
            
            datadict = Simulink.data.dictionary.create(filepath2racelineDD);

        end
        
        % delete old data dictionary content
        dDataSectObj = getSection(datadict,'Design Data');
        list_struct_entries = find(dDataSectObj, 'DataSource', DDname);

        % delete all data dict entries of type struct
        for i=1:length(list_struct_entries)
            deleteEntry(dDataSectObj, list_struct_entries(i).Name)
        end

        % write raceline, start position and raceline file info to data dictionary
        disp(strcat('Write raceline "', filename_raceline, '" to raceline data dictionary...'));

        addEntry(dDataSectObj, "Raceline", Raceline)
        
        % add starting and parking position to data dictionary
        
        % TODO: Replace
        addEntry(dDataSectObj,'x0_vehiclepose_stm',...
        [Raceline.x_m(1),Raceline.y_m(1),Raceline.psi_rad(1)]);
    
        addEntry(dDataSectObj,'x0_vehiclepose_dtm',...
        [Raceline.x_m(1), -1 * Raceline.y_m(1), 0, 0, 0, normalizeAngle((-pi/2) - Raceline.psi_rad(1))]);

        for i=1:int_count_vehicles
            addEntry(dDataSectObj, strcat("x0_vehiclepos_start_veh", num2str(i)), x0_vehiclepose_start(i, :))
         	addEntry(dDataSectObj, strcat("x0_vehiclepos_park_veh", num2str(i)), x0_vehiclepose_park(i, :))
        end
        
        % add name of current raceline
        addEntry(dDataSectObj, 'current_raceline', filename_raceline)

        % save and close data dictionary
        datadict.saveChanges()
        datadict.close(); 
        
        % activate raceline (if other mode was used before)DDObj = ...
        DDObj = Simulink.data.dictionary.open('TrajectoryPlanningEmulation.sldd');
        dataSectObj = getSection(DDObj, 'Design Data');
        switchObj = getEntry(dataSectObj, 'P_VDC_TrajEmulation_mode');
        setValue(switchObj, 0);
        % save & close
        saveChanges(DDObj);
        close(DDObj);
            
    catch e
        warning(['Something went wrong during configuration of ' ...
                'raceline data dictionary']);
        disp('******************************************');
        disp('Exception: ');
        disp(getReport(e))
        disp('******************************************');
        
    end
    
end
