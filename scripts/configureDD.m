function configureDD(DDname, vehicle)
% configures a component to use the parametrization for a specific vehicle
% 
% DDname: The main component data dictionary which shall be configured 
% vehicle: The vehicle code which should be used (e.g. db, rc, ...)

% check if DDname is missing data dictionary extension 
if(isempty(strfind(DDname, '.sldd')))
  DDname = [DDname '.sldd']; 
end

% open main data dictionary for component which should be configured
DD = Simulink.data.dictionary.open(DDname);
% find all references which link to a vehicle specific DD 
idxConfigureCells = strfind(DD.DataSources, DDname);
% remove these references 
for i = 1:1:length(DD.DataSources)
  % if the reference does not contain a vehicle specific DD continue with next 
  if(isempty(idxConfigureCells{i}))
    continue; 
  end
  % remove the vehicle specific DD 
  removeDataSource(DD, DD.DataSources{i});
end
% add the new vehicle specific reference
addDataSource(DD, [vehicle '_' DDname]); 
% save and close data dictionary 
DD.saveChanges()
DD.close()
end