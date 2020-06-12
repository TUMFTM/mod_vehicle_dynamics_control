function result = analyzeLocalizationQuality(debug, ground_truth, PerformanceSpec, visualize)

% Author: Alexander Wischnewski     Date: 24-02-2019
% 
% Description: 
%   scans the debug data and ground truth and evalutes the localization
%   quality
% 
% Input: 
%   debug:                  Debug structure of the vehicle
%   ground_truth:           Physical ground truth data from simulation
%   PerformanceSpec:        Structure with fields: 
%       sDevMax_m               Maximum deviation position in longitudinal direction
%       dDevMax_m               Maximum deviation position in lateral direction
%       psiDevMax_rad           Maximum deviation heading
%       vxDevMax_mps            Maximum deviation longitudinal velocity
%       vyDevMax_mps            Maximum deviation lateral velocity
%       dPsiDevMax_radps        Maximum deviation yaw rate
%   visualize:              Set to true if debug plots shall be done  
% 
% Outputs: 
%   result:       1 if the localization quality is sufficient

result = 1; 

disp('------------------------------');
disp('Check Localization Quality: ');
disp('------------------------------');
% check whether sensor fusion is accurate enough during the whole dataset  
% after it was valid for the first time 
idxStart = find(uint16(debug.debug_mvdc_state_estimation_debug_StateEstimate_SE_Status.Data) == 2, 1); 
if(isempty(idxStart))
    disp('No valid state estimate was found in the dataset at all'); 
    result = 0; 
    return 
end
tStart = debug.debug_mvdc_state_estimation_debug_StateEstimate_SE_Status.Time(idxStart); 
disp(['Localization valid after ' num2str(tStart) 's']); 

% localization quality has to be assessed dependent on the current vehicle
% orientation, as the quality requirements in lateral are way higher than
% in longitudinal

t_vec = tStart:0.04:ground_truth.SimRealState_x_m.Time(end); 
x_m_gt = resample(ground_truth.SimRealState_x_m, t_vec);
y_m_gt = resample(ground_truth.SimRealState_y_m, t_vec);
x_m_se = resample(debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m, t_vec);
y_m_se = resample(debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m, t_vec);
psi_rad_gt = -1*(resample(ground_truth.SimRealState_psi_rad, t_vec) + pi/2);
dx = x_m_se.Data - x_m_gt.Data; 
dy = y_m_se.Data - y_m_gt.Data; 
p_s = zeros(length(dx), 1); 
p_d = zeros(length(dx), 1); 
% transform localization deviation to vehicle coordinate system 
for i = 1:1:length(dx)
    p_s(i) = cos(psi_rad_gt.Data(i))*dx(i) - sin(psi_rad_gt.Data(i))*dy(i); 
    p_d(i) = sin(psi_rad_gt.Data(i))*dx(i) + cos(psi_rad_gt.Data(i))*dy(i); 
end
s_m = timeseries(p_s, t_vec); 
d_m = timeseries(p_d, t_vec); 
zero_ts = timeseries(zeros(length(dx), 1), t_vec); 

if(compareSignals(zero_ts, s_m,...
        tStart, 'Longitudinal localization error', PerformanceSpec.sDevMax_m, false, visualize) == 0)
    result = 0; 
end

if(compareSignals(zero_ts, d_m,...
        tStart, 'Lateral localization error', PerformanceSpec.dDevMax_m, false, visualize) == 0)
    result = 0; 
end

if(compareSignals(ground_truth.SimRealState_psi_rad, debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_psi_rad,...
        tStart, 'Heading', PerformanceSpec.psiDevMax_rad, true, visualize) == 0)
    result = 0; 
end

if(compareSignals(ground_truth.SimRealState_vx_mps, debug.debug_mvdc_state_estimation_debug_StateEstimate_vx_mps,...
        tStart, 'Longitudinal velocity', PerformanceSpec.vxDevMax_mps, false, visualize) == 0)
    result = 0; 
end

if(compareSignals(ground_truth.SimRealState_vy_mps, debug.debug_mvdc_state_estimation_debug_StateEstimate_vy_mps,...
        tStart, 'Lateral velocity', PerformanceSpec.vyDevMax_mps, false, visualize) == 0)
    result = 0; 
end

if(compareSignals(ground_truth.SimRealState_dPsi_radps, debug.debug_mvdc_state_estimation_debug_StateEstimate_dPsi_radps,...
        tStart, 'Yaw rate', PerformanceSpec.dPsiDevMax_radps, false, visualize) == 0)
    result = 0; 
end

if(result == 1)
   disp(' ');
   disp('All localization checks passed.');
end

end

function dev_ok = compareSignals(signal1, signal2, tStart, name, maxDev, normalize, visualize)
    s1 = resample(signal1, tStart:0.004:signal1.Time(end));
    s2 = resample(signal2, tStart:0.004:signal1.Time(end));
    s1Dev = s1.Data-s2.Data; 
    if(normalize)
        s1Dev = normalizeAngle(s1Dev); 
    end
    s1Dev = abs(s1Dev); 
    disp(' ');
    disp(name); 
    display(['Maximum deviation: ' num2str(max(s1Dev))]); 
    if(~(max(s1Dev) < maxDev))
        disp('Deviation too high'); 
        dev_ok = 0; 
    else
        disp('Deviation within spec'); 
        dev_ok = 1; 
    end
    if(visualize)
        figure; 
        subplot(2, 1, 1); 
        if(normalize)
            plot(signal1.Time, normalizeAngle(signal1.Data)); 
        else
            plot(signal1); 
        end
        hold on; grid on; 
        if(normalize)
            plot(signal2.Time, normalizeAngle(signal2.Data)); 
        else
            plot(signal2); 
        end
        title(name); 
        subplot(2, 1, 2); 
        plot(s1.Time, s1Dev); 
        grid on; hold on; 
        plot([s1.Time(1), s1.Time(end)], [maxDev, maxDev], '--')
        ylabel('Deviation'); 
        xlabel('Time in s'); 
    end
end