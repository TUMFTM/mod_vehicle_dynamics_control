function varargout = trajectorycontrolvisualizer(varargin)
% TRAJECTORYCONTROLVISUALIZER MATLAB code for trajectorycontrolvisualizer.fig
%      TRAJECTORYCONTROLVISUALIZER, by itself, creates a new TRAJECTORYCONTROLVISUALIZER or raises the existing
%      singleton*.
%
%      H = TRAJECTORYCONTROLVISUALIZER returns the handle to a new TRAJECTORYCONTROLVISUALIZER or the handle to
%      the existing singleton*.
%
%      TRAJECTORYCONTROLVISUALIZER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRAJECTORYCONTROLVISUALIZER.M with the given input arguments.
%
%      TRAJECTORYCONTROLVISUALIZER('Property','Value',...) creates a new TRAJECTORYCONTROLVISUALIZER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trajectorycontrolvisualizer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trajectorycontrolvisualizer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trajectorycontrolvisualizer

% Last Modified by GUIDE v2.5 10-Mar-2019 09:23:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trajectorycontrolvisualizer_OpeningFcn, ...
                   'gui_OutputFcn',  @trajectorycontrolvisualizer_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before trajectorycontrolvisualizer is made visible.
function trajectorycontrolvisualizer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trajectorycontrolvisualizer (see VARARGIN)

% Choose default command line output for trajectorycontrolvisualizer
handles.output = hObject;

axes(handles.axes7)
matlabImage = imread('TUM-Logo-40.png');
image(matlabImage)
axis off
axis image

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trajectorycontrolvisualizer wait for user response (see UIRESUME)
% uiwait(handles.main);


% --- Outputs from this function are returned to the command line.
function varargout = trajectorycontrolvisualizer_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function SubsetPointer_Callback(hObject, eventdata, handles)
% hObject    handle to SubsetPointer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% write slider value into edit field 

UpdateMarker_Callback(hObject, eventdata, handles, ceil(handles.SubsetPointer.Value));

% --- Executes during object creation, after setting all properties.
function SubsetPointer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SubsetPointer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% update Marker callback 
function UpdateMarker_Callback(hObject, eventdata, handles, newValue)

    handles = guidata(hObject); 
    % if simple mode is not active update the trajectory pointers as planned 
    if(~handles.SimpleMode)
      % check bounds
      newValue = max(min(newValue, handles.SubsetPointer.Max), handles.SubsetPointer.Min);
      % find idx which to move marker to 
      targetidx = find(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data ==  newValue, 1);
      % move marker 
      handles.PlotMarkerLap.XData = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time(targetidx);
      handles.PlotMarkerLap.YData = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data(targetidx);
      handles.PlotMarkerLapIdx = newValue; 
      % plot actual beginning time of lap 
      handles.actualtimestamp_string.String = [num2str(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time(targetidx)) ' s'];
      % update edit field and slider 
      handles.SubsetPointerEdit.String = newValue; 
      handles.SubsetPointer.Value = newValue; 
    end
    % check if simple mode is active, if this is the case, take the first and last time sample
    % as boundaries 
    try
      if(handles.SimpleMode) 
        handles.tStart_simple = handles.debug.debug_Time_s.Data(1); 
        handles.tEnd_simple = handles.debug.debug_Time_s.Data(end); 
        handles.actualtimestamp_string.String = ''; 
      else
        % get start and stop time of visualization 
        % take lap navigation 
        idx_start = find((handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == handles.PlotMarkerLapIdx), 1, 'first'); 
        idx_end = find((handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data == handles.PlotMarkerLapIdx), 1, 'last'); 
        % check if only one lap is available and prevent idx_end from being out of range
        if(idx_end > handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Length)
          idx_end = idx_end - 1; 
        end
        handles.tStart_lap = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time(idx_start); 
        handles.tEnd_lap = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time(idx_end);     
        [idxStart,idxEnd] = find_ts_idx(handles.debug.debug_mvdc_trajectory_driver_perf_LapTime_s,handles.tStart_lap,handles.tEnd_lap); 
        handles.laptime.String = [num2str(handles.debug.debug_mvdc_trajectory_driver_perf_LapTime_s.Data(idxEnd)) ' s']; 
      end  
    catch 
      disp('Error during calculation of time limits'); 
    end

    % store to guidata
    guidata(hObject, handles); 
    updateMainGUIMap(hObject, eventdata, handles); 

function updateMainGUIMap(hObject, eventdata, handles)

handles = guidata(hObject); 
% update map plot 
cla(handles.mapView); 
if(handles.SimpleMode)
    % find valid data points 
    idx_valid = (handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Data == 2);
    idx_start = find(idx_valid, 1, 'first'); 
    idx_end = find(idx_valid, 1, 'last');  
    % only plot valid data
    tMinValid = handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Time(idx_start); 
    tMaxValid = handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Time(idx_end); 
    % create temporary timeseries which exluces non valid matched path points 
    actualPathPoint_x_temp = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_x_m; 
    actualPathPoint_y_temp = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_y_m; 
    % rework actual path point data 
    if(~isempty(tMinValid)) 
      actualPathPoint_x_temp.Data(1:(idx_start-1)) = NaN; 
      actualPathPoint_y_temp.Data(1:(idx_start-1)) = NaN; 
    end
    if(~isempty(tMaxValid))
      actualPathPoint_x_temp.Data((idx_end+1):end) = NaN; 
      actualPathPoint_y_temp.Data((idx_end+1):end) = NaN; 
    end

      mapViewplot = xy_axes({handles.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m,...
            actualPathPoint_x_temp},...
            {handles.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m,...
            actualPathPoint_y_temp},...
            {'Actual', 'Path'}, 'East - Coordinate in m', 'North - Coordinate in m', true, true); 
      mapViewplot.plot_all(handles.mapView, handles.tStart_simple, handles.tEnd_simple); 
      updateErrors(hObject, eventdata, handles, handles.tStart_simple, handles.tEnd_simple); 
else
    % find valid data points 
    idx_valid = (handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Data == 2);
    idx_start = find(idx_valid, 1, 'first'); 
    idx_end = find(idx_valid, 1, 'last');  
    % only plot valid data
    tMinValid = handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Time(idx_start); 
    tMaxValid = handles.debug.debug_mvdc_trajectory_driver_debug_PathMatchingStatus.Time(idx_end); 
    % create temporary timeseries which exluces non valid matched path points 
    actualPathPoint_x_temp = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_x_m; 
    actualPathPoint_y_temp = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_y_m; 
    % rework actual path point data 
    if(~isempty(tMinValid)) 
      actualPathPoint_x_temp.Data(1:(idx_start-1)) = NaN; 
      actualPathPoint_y_temp.Data(1:(idx_start-1)) = NaN; 
    end
    if(~isempty(tMaxValid))
      actualPathPoint_x_temp.Data((idx_end+1):end) = NaN; 
      actualPathPoint_y_temp.Data((idx_end+1):end) = NaN; 
    end
    % one is lap marker 
    mapViewplot = xy_axes({handles.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_x_m,...
          actualPathPoint_x_temp},...
          {handles.debug.debug_mvdc_state_estimation_debug_StateEstimate_Pos_y_m,...
          actualPathPoint_y_temp},...
          {'Actual', 'Path'}, 'East - Coordinate in m', 'North - Coordinate in m', true, true); 
    mapViewplot.plot_all(handles.mapView, handles.tStart_lap, handles.tEnd_lap);
    updateErrors(hObject, eventdata, handles, handles.tStart_lap, handles.tEnd_lap); 
end

% store to guidata
guidata(hObject, handles); 

function updateErrors(hObject, eventdata, handles, tStart, tEnd)
    % update error values in main GUI 
    [idxStart,idxEnd] = find_ts_idx(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_lateral_m,tStart,tEnd); 
    handles.laterrrms.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_lateral_m.Data(idxEnd));
    handles.laterrpeak.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_ePeak_lateral_m.Data(idxEnd));
    [idxStart,idxEnd] = find_ts_idx(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_heading_rad,tStart,tEnd); 
    handles.headerrrms.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_heading_rad.Data(idxEnd));
    handles.headerrpeak.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_ePeak_heading_rad.Data(idxEnd));
    [idxStart,idxEnd] = find_ts_idx(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_v_mps,tStart,tEnd); 
    handles.velerrrms.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_v_mps.Data(idxEnd));
    handles.velerrpeak.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_ePeak_v_mps.Data(idxEnd));
    [idxStart,idxEnd] = find_ts_idx(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_kappa_radpm,tStart,tEnd); 
    handles.curverrrms.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_eRMS_kappa_radpm.Data(idxEnd));
    handles.curverrpeak.String = num2str(handles.debug.debug_mvdc_trajectory_driver_perf_ePeak_kappa_radpm.Data(idxEnd));
        
% --- Executes on button press in loaddataset.
function loaddataset_Callback(hObject, eventdata, handles)
% hObject    handle to loaddataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

  % delete data set
  if (isfield(handles,'debug'))
    handles=rmfield(handles,'debug');
  end
  if (isfield(handles,'real_physics'))
    handles=rmfield(handles,'real_physics');
  end
  if (isfield(handles,'sim_physics'))
    handles=rmfield(handles,'sim_physics');
  end
  
  % load dataset
    [FileName,PathName] = uigetfile('*.mat','Select the debug data file');
    if((FileName == 0))
      disp('No file selected'); 
      return
    end
    
    disp('Loading file');
    data = load([PathName FileName]);
  
	if(isfield(data, 'debug'))
    disp('Dataset debug found'); 
    handles.debug = data.debug;
    handles.datasetname.String = FileName; 
    else
    disp('No data found in file'); 
    return; 
    end
  
    if(isfield(data, 'real_physics'))
    disp('Dataset real_physics found'); 
    handles.real_physics = data.real_physics;
    else
    disp('Dataset does not contain real_physics structure');
    end
    
    if(isfield(data, 'sim_physics'))
    disp('Dataset sim_physics found'); 
    handles.sim_physics = data.sim_physics;
    else
    disp('Dataset does not contain sim_physics structure');
    end

% check if there are open windows (if yes, close) 
closevisz_Callback(hObject, eventdata, handles);
if(any(diff(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_TrajCnt.Data)~=0))
  % check if trajectory infromation are available
  handles.SimpleMode = 0; 
  handles.fulllapmode_pb.Enable = 'On'; 
  handles.fulllapmode_pb.String = 'Full Dataset Mode';
else
  handles.SimpleMode = 1; 
  handles.fulllapmode_pb.Enable = 'Off'; 
  handles.fulllapmode_pb.String = 'Single Lap Mode';
end
% reset markers 
handles.PlotMarkerLapIdx = 0;
handles.PlotMarkerType = 0; 
% store to guidata
guidata(hObject, handles); 
setupMode(hObject, eventdata, handles); 
handles = guidata(hObject); 
UpdateMarker_Callback(hObject, eventdata, handles, ceil(handles.SubsetPointer.Value)); 
handles = guidata(hObject); 
% view all main gui plots
handles.mapView.Visible = 'On';
% store to guidata
guidata(hObject, handles); 
  
disp(['Loading dataset ' FileName ' finished']); 

function setupMode(hObject, eventdata, handles) 

  handles = guidata(hObject); 
  % check if target path information are available 
  if(~handles.SimpleMode)
    % setup navigation interface with lap and trajectory count 
    LapCounterVisual = ty_axes({handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt},{'Lap number'});
    cla(handles.LapCounter); 
    LapCounterVisual.plot_all(handles.LapCounter);

    if(~isfield(handles, 'PlotMarkerLapIdx'))
      handles.PlotMarkerLapIdx = handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data(1); 
    end
    hold(handles.LapCounter, 'on'); 
    if(~exist('handles.PlotMarkerLap'))
      % check if marker needs to be created 
      handles.PlotMarkerLap = plot(handles.LapCounter,...
        handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Time(1),...
        handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data(1),'ro', 'MarkerFaceColor', 'r');
    else
      handles.PlotMarkerLap.Visible = 'Off'; 
    end
    handles.LapCounter.Visible = 'On';
    handles.SubsetPointer.Enable = 'On'; 
    handles.SubsetPointerEdit.Enable = 'On'; 
    handles.switchdatabase.Enable = 'On'; 
    handles.text4.Enable ='On'; 
    handles.text5.Enable ='On'; 
    handles.text8.Enable = 'On';
    handles.foresight_count.Enable = 'On';
    handles.foresight_count.String ='0';
    
        % check if there is only one lap to display
      if(max(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data)...
          == min(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data))
        % disable slider 
        handles.SubsetPointer.Enable = 'off';
        handles.PlotMarkerLapIdx = min(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
        handles.SubsetPointer.Min = min(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
        handles.SubsetPointer.Max = min(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data)+1; 
      else
        % update slider min max 
        handles.SubsetPointer.Min = min(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
        handles.SubsetPointer.Max = max(handles.debug.debug_mvdc_path_matching_debug_ActualTrajPoint_LapCnt.Data); 
        % enable slider 
        handles.SubsetPointer.Enable = 'on';
      end
      handles.PlotMarkerLap.Visible = 'On'; 
      handles.SubsetPointer.Value = handles.PlotMarkerLapIdx;
      handles.SubsetPointerEdit.String = num2str(handles.PlotMarkerLapIdx);
      % clear it up 
      handles.SubsetPointer.SliderStep = [1/(handles.SubsetPointer.Max-handles.SubsetPointer.Min), 10/(handles.SubsetPointer.Max-handles.SubsetPointer.Min+1)]; 
      handles.SubsetPointerEdit.String = num2str(handles.SubsetPointer.Value);
  else
    % if they do not exist, go back to simple mode and disable everything
    handles.LapCounter.Visible = 'Off';
    cla(handles.LapCounter); 
    handles.SubsetPointer.Enable = 'Off'; 
    handles.SubsetPointerEdit.Enable = 'Off'; 
    handles.switchdatabase.Enable = 'Off'; 
    handles.text4.Enable ='Off'; 
    handles.text5.Enable ='Off'; 
    handles.text8.Enable = 'Off';
    handles.foresight_count.Enable = 'Off';
    handles.foresight_count.String ='';
    handles.tStart_simple = handles.debug.debug_Time_s.Data(1); 
    handles.tEnd_simple = handles.debug.debug_Time_s.Data(end); 
  end

  guidata(hObject, handles); 

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in updatevisz.
function updatevisz_Callback(hObject, eventdata, handles)
% hObject    handle to updatevisz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  
% check if there are datasets
if(~isfield(handles, 'debug') && ~isfield(handles, 'real_physics') && ~isfield(handles, 'sim_physics'))
  disp('No data loaded'); 
  return; 
end

% check analysis struct
str=get(handles.analysisstruct,'String');
val=get(handles.analysisstruct,'Value');

switch str{val}
    case 'none'
        disp('No visualization options chosen. Select analysis struct first.')
        analysismode='none';
    case 'debug'
        if isfield(handles,'debug')
            analysisdata=handles.debug;
            analysismode='debug';
        else
            analysismode='notavailable';
        end
    case 'real_physics'
        if isfield(handles,'real_physics')
            analysisdata=handles.real_physics';
            analysismode='real_physics';
        else
            analysismode='notavailable';
        end
    case 'sim_physics'
        if isfield(handles,'sim_physics')
            analysisdata=handles.sim_physics;
            analysismode='sim_physics';
        else
            analysismode='notavailable';
        end
end

switch analysismode
    case 'none'
    case 'notavailable'
        
        disp('Required analysis struct not available in dataset')
        
    case {'debug','real_physics','sim_physics'}
            
        % read out right start and stop times 
        if(handles.SimpleMode)
          tStart = handles.tStart_simple; 
          tEnd = handles.tEnd_simple; 
        else
          tStart = handles.tStart_lap; 
          tEnd = handles.tEnd_lap; 
        end
        
        % check if selection of visualization options is empty
        numberoffigures=0;
        for i = 1:1:length(handles.plotIDs)
            temp=numberoffigures;
            numberoffigures= handles.cb_array{i}.Value + temp;
        end
        if numberoffigures==0
            disp('No visualization options chosen')
        end
        
        % check if the figure handles do not exist or have been destroyed and are not 
        % a valid figure handle anymore. Further, the figure needs to be activated. 
        for i = 1:1:length(handles.plotIDs)
            % if the corresponding checkbox is pressed and there is no valid figure
            % handle, create one. 
            if(handles.cb_array{i}.Value == 1)
                if(isempty(handles.fig_array{i}))
                    handles.fig_array{i} = figure('Name', handles.plotIDs{i});
                elseif(~isvalid(handles.fig_array{i}))
                    handles.fig_array{i} = figure('Name', handles.plotIDs{i});
                end        
                try
                    % get plot configuration
                    myjson=fileread([handles.plotConfig_path '/' handles.plotIDs{i} '.json']); 
                    myjson_parsed = jsondecode(myjson); 
                    visz_plotGroup(createPlotGroup(analysisdata, analysismode, myjson_parsed), ...
                      handles.fig_array{i}, tStart, tEnd); 
                catch
                   disp(['Error while plotting ' handles.plotIDs{i}]); 
                end
            end
        end
        
end

guidata(hObject, handles); 


function SubsetPointerEdit_Callback(hObject, eventdata, handles)
% hObject    handle to SubsetPointerEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SubsetPointerEdit as text
%        str2double(get(hObject,'String')) returns contents of SubsetPointerEdit as a double

% ensure that only integer values are entered 
UpdateMarker_Callback(hObject, eventdata, handles, floor(str2double(handles.SubsetPointerEdit.String)));

% --- Executes during object creation, after setting all properties.
function SubsetPointerEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SubsetPointerEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in closevisz.
function closevisz_Callback(hObject, eventdata, handles)
% hObject    handle to closevisz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% this function closes all open windows, in case their figure handle exists
% and it is a valid handle. This is necessary to prevent the programm from crashing. 
if isfield(handles,'plotIDs')
for i = 1:1:length(handles.plotIDs)
    if(~isempty(handles.fig_array{i}) && isvalid(handles.fig_array{i}))
        close(handles.fig_array{i}); 
    end
end
end
disp('Closed figures'); 


% --- Executes on button press in fulllapmode_pb.
function fulllapmode_pb_Callback(hObject, eventdata, handles)
% hObject    handle to fulllapmode_pb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.SimpleMode == 0) 
  handles.fulllapmode_pb.String = 'Single Lap Mode'; 
  handles.SimpleMode = 1;   
  % store to guidata
  guidata(hObject, handles); 
  setupMode(hObject, eventdata, handles); 
  handles = guidata(hObject); 
  updateMainGUIMap(hObject, eventdata, handles); 
  handles = guidata(hObject); 
  handles.laptime.String = ''; 
else
  handles.fulllapmode_pb.String = 'Full Dataset Mode'; 
  handles.SimpleMode = 0;   
  % store to guidata
  guidata(hObject, handles); 
  setupMode(hObject, eventdata, handles); 
  handles = guidata(hObject); 
  UpdateMarker_Callback(hObject, eventdata, handles, floor(str2double(handles.SubsetPointerEdit.String))); 
  handles = guidata(hObject); 
  updateMainGUIMap(hObject, eventdata, handles); 
end
handles = guidata(hObject); 


% --- Executes on selection change in analysisstruct.
function analysisstruct_Callback(hObject, eventdata, handles)
% hObject    handle to analysisstruct (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns analysisstruct contents as cell array
%        contents{get(hObject,'Value')} returns selected item from analysisstruct

% check analysis struct
str=get(handles.analysisstruct,'String');
val=get(handles.analysisstruct,'Value');

switch str{val}
    case 'none'
        set(handles.uibuttongroup2,'Title','No visualization options available');
    case 'debug'
        set(handles.uibuttongroup2,'Title','Choose debug visualizations to show');
    case 'real_physics'
        set(handles.uibuttongroup2,'Title','Choose real_physics visualizations to show');
    case 'sim_physics'
        set(handles.uibuttongroup2,'Title','Choose sim_physics visualizations to show');     
end

% Close open figures
if isfield(handles,{'plotIDs','cb_array','fig_array'}) 
for i = 1:1:length(handles.plotIDs)
    if(~isempty(handles.fig_array{i}) && isvalid(handles.fig_array{i}))
        close(handles.fig_array{i}); 
    end
end
end
disp('Closed figures');

% Remove check boxes from current handles object
if isfield(handles,{'cb_array'})
for i=1:numel(handles.cb_array)
    delete(handles.cb_array{i})
end
end

% Remove variables from handles object
if isfield(handles,{'plotIDs','cb_array','fig_array'})
handles=rmfield(handles,{'plotIDs','cb_array','fig_array'});
end

% Force graphical update of GUI 
drawnow 

% Load new visualization options upon selection of Analysis struct 
switch str{val}

    case 'none'
        
    case {'debug','real_physics','sim_physics'}    
    
        % Initialise new plot interface
        nVert = 5; 
        nHor = 5; 
        p0_hor = 70; 
        p0_vert = 115; 
        space_hor = 20; 
        space_vert = 7; 
        width = 150; 
        height = 15; 

        % dialog to select folder with config 
        handles.plotConfig_path = uigetdir(pwd, strcat('Select ',str{val},' visualization config folder')); 
        % load config 
        plotConfig_temp = dir(handles.plotConfig_path);
        % the file list returns also the current and upper folder handles as
        % virtual files, therefore to files must be removed
        plotConfig = plotConfig_temp(3:end); 
        nPlots = length(plotConfig); 

        % create checkboxes to use 
        for i = 0:1:(nPlots-1)
           % determine position for checkbox, starting with index at 0
           xPos = floor(i/nVert);
           yPos = mod(i,nVert); 
           % calculate coordinates 
           cb_pos = [p0_hor+xPos*(space_hor+width), p0_vert-yPos*(space_vert+height), width, height];
           % get file name
           plotName_raw = plotConfig(i+1).name; 
           % remove file ending
           plotName = plotName_raw(1:(end-5));
           % create array with plot IDs (the file names from the jsons) 
           handles.plotIDs{i+1} = plotName; 
           handles.cb_array{i+1} =  uicontrol('Style', 'checkbox', 'Position',cb_pos, 'String', plotName);
           handles.fig_array{i+1} = []; 
        end

end

guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function analysisstruct_CreateFcn(hObject, eventdata, handles)
% hObject    handle to analysisstruct (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
