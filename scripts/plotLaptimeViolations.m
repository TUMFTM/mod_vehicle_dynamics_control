function plotLaptimeViolations(files, laps, labels, max_data_points)
% files: .mat files in a dict. like [controller1 datapoint 1, controller1
% datapoint 2, ...., controller2 datapoint 1,...., controllerN datapoint
% max_data_points]
% max_data_points: number of data points per controller
for i = 1:1:length(files) 
    data{i} = load(files{i}); 
end
LineWidth = 1; 
% check if labels are given, if not use file names
if(nargin <= 2) 
    labels = files; 
end
k = length(files)/max_data_points;
lap_times = zeros(k,max_data_points);
acc_viol = zeros(k,max_data_points);
dy_viol = zeros(k,max_data_points);
sum_viol = zeros(k,max_data_points);
dax = zeros(k,max_data_points);    

% find start and end indices
for i = 1:1:length(files) 
    idx_start{i} = find((data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(1)), 1, 'first'); 
    idx_end{i} = find((data{i}.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(2)), 1, 'last'); 
end
    
for i = 1:1:k
    for j = 1:1:max_data_points
        lap_times(i,j) = data{(i-1)*max_data_points+j}.debug.debug_mvdc_trajectory_driver_perf_LapTime_s.Data(idx_end{(i-1)*max_data_points+j});
        acc_viol(i,j) = data{(i-1)*max_data_points+j}.debug.debug_mvdc_trajectory_driver_perf_TireConstViolations.Data(idx_end{(i-1)*max_data_points+j}-1);
        dy_viol(i,j) = data{(i-1)*max_data_points+j}.debug.debug_mvdc_trajectory_driver_perf_LatConstViolations.Data(idx_end{(i-1)*max_data_points+j}-1);
        if plot_K_opt
            dax(i,j) = data{(i-1)*max_data_points+j}.debug_slow.debug_slow_M_tilde.Data(500,1);            
        end
    end
end
sum_viol = dy_viol + acc_viol;
   
figure; box on, hold on, grid on;
for i = 1:1:k
    plot(lap_times(i,:),sum_viol(i,:),'LineWidth', LineWidth); 
end
xlabel('Lap time in s'); ylabel('Number of const. violations');
legend(labels{1:end}); 

