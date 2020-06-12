function [nLaps, CP] = analyzeTrackingPerformance(debug, plotActive, leaveOutFirstAndLastLap) 
% searches for the RMS and peak error values in the control errors logged by the vehicle. 
% these are displayed and compared over multiple laps. 
%
%%%%%%% Inputs %%%%%%%%
% debug: contains the standard TUM roborace debug structure
% plotActive: if this is set to true, the performance analysis is plotted 
% leaveOutFirstAndLastLap: if this is set to true, the plot does not consider the first and the last 
% lap (unreasonable values sometimes) 
%%%%%%% Outputs %%%%%%%
% nLaps: number of laps
% CP: structure with all rms and peak control errors 

% find index of last data points in the lap 
deltaLapCnt = diff(debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
lastPoint_idx_raw = find(deltaLapCnt); 
% cut of first point
lastPoint_idx = lastPoint_idx_raw(2:end);
nLaps = numel(lastPoint_idx);
disp([num2str(nLaps) ' Lap(s) found']);  

% get control errors
CP.eTraj_lat_m_RMS = debug.debug_mvdc_trajectory_driver_perf_eRMS_lateral_m.Data(lastPoint_idx); 
CP.eTraj_lat_m_Peak = debug.debug_mvdc_trajectory_driver_perf_ePeak_lateral_m.Data(lastPoint_idx); 
CP.eTraj_heading_rad_RMS = debug.debug_mvdc_trajectory_driver_perf_eRMS_heading_rad.Data(lastPoint_idx); 
CP.eTraj_heading_rad_Peak = debug.debug_mvdc_trajectory_driver_perf_ePeak_heading_rad.Data(lastPoint_idx); 
CP.eTraj_v_mps_RMS = debug.debug_mvdc_trajectory_driver_perf_eRMS_v_mps.Data(lastPoint_idx); 
CP.eTraj_v_mps_Peak = debug.debug_mvdc_trajectory_driver_perf_ePeak_v_mps.Data(lastPoint_idx); 
CP.eTraj_kappa_radpm_RMS = debug.debug_mvdc_trajectory_driver_perf_eRMS_kappa_radpm.Data(lastPoint_idx); 
CP.eTraj_kappa_radpm_Peak = debug.debug_mvdc_trajectory_driver_perf_ePeak_kappa_radpm.Data(lastPoint_idx); 

% if the plot is requested
if(plotActive)
  % check if the first and last lap shall be omitted
  if(leaveOutFirstAndLastLap)
    % if there are more than two laps, omit the outer
    if(nLaps > 2)
      plot_idx = 2:1:(nLaps-1); 
    else
      % else plot both
      plot_idx = 1:2; 
    end
  else
    % if the flag is not set, just display all
    plot_idx = 1:1:nLaps;
  end
  % define plot x labels
  labels = {'RMS','Peak'};
  % visualize errors via laps 
  figure; 
  subplot(3, 2, 1); 
  bar([CP.eTraj_lat_m_RMS(plot_idx)'; CP.eTraj_lat_m_Peak(plot_idx)']); 
  ylabel({'Lateral', 'error in m'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
  subplot(3, 2, 2); 
  bar([CP.eTraj_heading_rad_RMS(plot_idx)'; CP.eTraj_heading_rad_Peak(plot_idx)']); 
  ylabel({'Heading', 'error in rad'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
  subplot(3, 2, 3); 
  bar([CP.eTraj_v_mps_RMS(plot_idx)'; CP.eTraj_v_mps_Peak(plot_idx)']); 
  ylabel({'Velocity trajectory', 'error in mps'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
  subplot(3, 2, 4); 
  bar([CP.eTraj_kappa_radpm_RMS(plot_idx)'; CP.eTraj_kappa_radpm_Peak(plot_idx)']); 
  ylabel({'Curvature trajectory', 'error in radpm'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
  subplot(3, 2, 5); 
  bar([CP.eControl_v_mps_RMS(plot_idx)'; CP.eControl_v_mps_Peak(plot_idx)']); 
  ylabel({'Velocity control', 'error in mps'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
  subplot(3, 2, 6); 
  bar([CP.eControl_kappa_radpm_RMS(plot_idx)'; CP.eControl_kappa_radpm_Peak(plot_idx)']); 
  ylabel({'Kappa control', 'error in radpm'}); 
  set(gca, 'xticklabels', labels);
  grid on; 
end
