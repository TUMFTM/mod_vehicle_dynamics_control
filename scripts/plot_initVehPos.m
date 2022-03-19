%%
% script to plot the inital vehicle start and park position stored in the
% raceline.sldd
%
% Author: Leonhard Hermansdorfer
%
% Note: given raceline name should match the raceline data which has been
% loaded into the raceline.sldd. Otherwise, the plot won't match.

%% User Input

% raceline to plot
raceline = 'LucasOil';
  

%% Load vehicle positions from data dict

filepath2raceline = which(horzcat(raceline, '.csv'));

csv_data = dlmread(filepath2raceline,';',3);
x_m = csv_data(:, 2);
y_m = csv_data(:, 3);


DDname = "raceline.sldd";
datadict = Simulink.data.dictionary.open(DDname);
dDataSectObj = getSection(datadict,'Design Data');

list_struct_entries = find(dDataSectObj, 'DataSource', DDname);

pos_park = [];
pos_start = [];

for i=1:length(list_struct_entries)
    
    temp = getEntry(dDataSectObj, list_struct_entries(i).Name);
    if contains(temp.Name, 'vehiclepos')
        if contains(temp.Name, 'start')
            pos_start = [pos_start; getValue(temp)];

        elseif contains(temp.Name, 'park')
            pos_park = [pos_park; getValue(temp)];
        end
    end
end


%% plot

figure;
hold on, grid on;

plot(x_m, y_m)

for i=1:size(pos_start,1)
    scatter(pos_start(i, 1), pos_start(i, 2))
    scatter(pos_park(i, 1), pos_park(i, 2), '+')
end

axis equal
xlabel('x-coordinate in m')
ylabel('y-coordinate in m')
