function [] = analyzeParVariation(logprefix, signalname, signalname_ref)
%
% Author: Thomas Herrmann <thomas.herrmann@tum.de>     Date: 22-10-2020
% 
% Description:
%   extracts log data from all Simulink simulation logs and displays in one
%   plot. Purpose of this script is the tuning of the TMPC algorithm.
% 
% Input:
%   logprefix:      Prefix of all log files from simulation
%   signalname:     name of signal to be compared
%   signalname_ref: name of reference signal
%
%% Algorithm

% Get all log files with specific prefix
F = dir([logprefix, '*.*']);
% Allocate legend
lgd = struct();
F_data_struct = struct();

figure; hold on; grid on;
legend('-DynamicLegend');


for i = 1:length(F)
    % Get data from log file
    data_tmp = load(F(i).name);
    
    % get file identifier with specific parameter set
    fname = strrep(...
        strrep(F(i).name,[logprefix, '_'],''), ...
        '.mat', '');
    
    % append to struct for later use
    F_data_struct.(fname) = getfield(data_tmp.debug, signalname);
    
    % plot different paramter set signals over s coordinate
    plot(data_tmp.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data, ...
        F_data_struct.(fname).Data, ...
        'DisplayName', fname);
    
    set(0, 'DefaultLegendInterpreter', 'none')
    
end

signal_ref = getfield(data_tmp.debug, signalname_ref);
plot(data_tmp.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_s_glob_m.Data, ...
    signal_ref.Data, ...
    'DisplayName', 'ref');
set(0, 'DefaultLegendInterpreter', 'none')
xlabel('s in m'); ylabel('v in mps');

end