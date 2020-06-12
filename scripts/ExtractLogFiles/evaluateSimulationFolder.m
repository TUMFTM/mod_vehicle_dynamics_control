function evaluateSimulationFolder()
% compares several simulation runs regarding the most important KPIs
data_folder = uigetdir; 
datasets = dir(data_folder); 
% iterate via all folders
% the first two items are . and .. so take care! 
for i = 1:1:(numel(datasets)-2)
  folderpath = [data_folder '\' datasets(i+2).name]; 
  % store dataset names to use as x values later 
  dataset_names{i} = datasets(i+2).name;
  % get filename of debug file
  filename = dir(folderpath); 
  results = load([folderpath '\' filename(3).name]); 
  % check if debug data is in there
  try
    if(isfield(results, 'debug'))
      % get the tracking performance 
      [nLaps, errors] = analyzeTrackingPerformance(results.debug, false, false); 
      % get the second last error points (best full lap) 
      if(nLaps < 1) 
        disp('Not enough laps to judge performance'); 
        % not enough laps so put NaN
        rms_errors(:, i) = NaN(6, 1); 
        peak_errors(:, i) = NaN(6, 1); 
      else
        rms_errors(:, i) = [errors.eTraj_lat_m_RMS(nLaps);...
          errors.eTraj_heading_rad_RMS(nLaps);...
          errors.eTraj_v_mps_RMS(nLaps);...
          errors.eTraj_kappa_radpm_RMS(nLaps);...
          errors.eControl_v_mps_RMS(nLaps);...
          errors.eControl_kappa_radpm_RMS(nLaps);];
        peak_errors(:, i) = [errors.eTraj_lat_m_Peak(nLaps);...
          errors.eTraj_heading_rad_Peak(nLaps);...
          errors.eTraj_v_mps_Peak(nLaps);...
          errors.eTraj_kappa_radpm_Peak(nLaps);...
          errors.eControl_v_mps_Peak(nLaps);...
          errors.eControl_kappa_radpm_Peak(nLaps);];
        disp(['RMS errors: ' num2str(rms_errors(:, i)')]);
        disp(['Peak errors: ' num2str(peak_errors(:, i)')]);
      end
    else
      % if no valid debug dataset is found, insert NaN values to the results 
      rms_errors(:, i) = NaN(6, 1); 
      peak_errors(:, i) = NaN(6, 1); 
    end
  catch
    % if something goes wrong insert NaN values to the results 
      rms_errors(:, i) = NaN(6, 1); 
      peak_errors(:, i) = NaN(6, 1); 
  end 
end
% visualize results 
figure; 
% specify y axis labels
ylabels = {{'Lateral', 'error in m'},...
  {'Heading', 'error in rad'},...
  {'Velocity trajectory', 'error in mps'},...
  {'Curvature trajectory', 'error in radpm'},...
  {'Velocity control', 'error in mps'},...
  {'Kappa control', 'error in radpm'}};
for i = 1:1:6
  % plot one performance indicator via all folders 
  subplot(3, 2, i); 
  bar(rms_errors(i, :)); 
  hold on; grid on; 
  ylabel(ylabels{i}); 
  set(gca, 'xticklabels', dataset_names); 
end
figure; 
for i = 1:1:6
  % plot one performance indicator via all folders 
  subplot(3, 2, i); 
  bar(peak_errors(i, :)); 
  hold on; grid on; 
  ylabel(ylabels{i}); 
  set(gca, 'xticklabels', dataset_names); 
end

