function plotDisturbanceComparison(files, laps, labels, max_data_points)
% files: .mat files in a dict. like [controller1 datapoint 1, controller1
% datapoint 2, ...., controller2 datapoint 1,...., controllerN datapoint
% max_data_points]
% max_data_points: number of data points per controller

LineWidth = 1; 

k = length(files)/max_data_points;
lap_times_flat = zeros(length(files), 1);
acc_viol_flat = zeros(length(files), 1);
dy_viol_flat = zeros(length(files), 1);
sum_viol_flat = zeros(length(files), 1);
dax_flat = zeros(length(files), 1);  
lap_times = zeros(k, max_data_points);
acc_viol = zeros(k, max_data_points);
dy_viol = zeros(k, max_data_points);
sum_viol = zeros(k, max_data_points);
dax = zeros(k,max_data_points);     

for i = 1:1:length(files) 
    data = load(files{i});     
    idx_start = find((data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(1)), 1, 'first'); 
    idx_end = find((data.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == laps(2)), 1, 'last'); 
    if(isempty(idx_end))
        disp(['No lap found for dataset ' files{i}]); 
        lap_times_flat(i) = NaN; 
        acc_viol_flat(i) = NaN; 
        dy_viol_flat(i) = NaN; 
        dax_flat(i) = NaN; 
    else
        lap_times_flat(i) = data.debug.debug_mvdc_trajectory_driver_perf_LapTime_s.Data(idx_end);
        acc_viol_flat(i) = data.debug.debug_mvdc_trajectory_driver_perf_TireConstViolations.Data(idx_end);
        dy_viol_flat(i) = data.debug.debug_mvdc_trajectory_driver_perf_LatConstViolations.Data(idx_end);
        dax_flat(i) = data.debug_slow.debug_slow_M_tilde.Data(500,1);         
    end
end

% resort to generate appropriate plots
for i = 1:1:k
    for j = 1:1:max_data_points
        lap_times(i,j) = lap_times_flat((i-1)*max_data_points+j);
        acc_viol(i,j) = acc_viol_flat((i-1)*max_data_points+j);
        dy_viol(i,j) = dy_viol_flat((i-1)*max_data_points+j);
        dax(i,j) = dax_flat((i-1)*max_data_points+j);            
    end
end

figure; 
subplot(3, 1, 1); box on, hold on, grid on;
for i = 1:1:k
    plot(dax(i,:), dy_viol(i,:),'LineWidth', LineWidth); 
end
xlabel('dax in mps2'); ylabel('violated dy constraints');
legend(labels{1:end}); 

subplot(3, 1, 2); box on, hold on, grid on;
for i = 1:1:k
    plot(dax(i,:), acc_viol(i,:),'LineWidth', LineWidth); 
end
xlabel('dax in mps2'); ylabel('violated a constraints');    

subplot(3, 1, 3); box on, hold on, grid on;
for i = 1:1:k
    plot(dax(i,:), lap_times(i,:),'LineWidth', LineWidth); 
end
xlabel('dax in mps2'); ylabel('Lap time in s');