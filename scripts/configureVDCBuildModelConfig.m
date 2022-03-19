function configureVDCBuildModelConfig(config)
%
% Author: Alexander Wischnewski     Date: 21-12-2018
% 
% Description: 
%   configures the models in this repository to use a specific build config
% 
% Input: 
%   config: Config name string, e.g. 'Speedgoat', 'RasperryPi' or GRT

% configure model simulation configs
try 
    modelConfigDD = ...
    Simulink.data.dictionary.open('modelconfig.sldd');
    ConfigurationsDD = getSection(modelConfigDD,'Configurations');
    buildConfigEntry = getEntry(ConfigurationsDD,'Active');
    newConfigEntry = getEntry(ConfigurationsDD, config);
    newConfig = getValue(newConfigEntry); 
    newConfig.Name = 'Active'; 
    setValue(buildConfigEntry, newConfig);
    % reconfigure sample rate 
    DesignDD = getSection(modelConfigDD,'Design Data');
    SampleRateEntry = getEntry(DesignDD,'tS');
    SampleRateTarget = getEntry(DesignDD,['tS_' config]);
    setValue(SampleRateEntry, getValue(SampleRateTarget)); 
catch 
    disp('Something went wrong during build config setup'); 
end
