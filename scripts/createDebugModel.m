function createDebugModel(target, bus, outfile)

% Author: Alexander Wischnewski     Date: 21-12-2018
% 
% Description: 
%   creates special models which allow logging of complex bus structures on
%   the real time targets. These can be copied and then integrated into the
%   actual software model. 
% 
% Input: 
%   target:     Target code string, e.g. 'Speedgoat' or 'GRT' 
%   bus:        Bus structure to be used for logging
%   outfile:    If given, the resulting data structure description is written 
%               to a file. This can be used for data conversion later
%               on. 

    % open template new model (depending on target) 
    switch target
        case 'Speedgoat'
            sys = 'mloc_debug_signal_conversion_sg'; 
        case 'ERT'
            sys = 'mloc_debug_signal_conversion_temp'; 
    end
    open_system(sys); 
    %% parse names of debug signals from data dictionary 
    % get debug bus signal from data dictionary 
    TUMInterfacesDD = Simulink.data.dictionary.open('TUM_interfaces.sldd'); 
    dDataSectObj = getSection(TUMInterfacesDD,'Design Data');
    debugBus = getValue(getEntry(dDataSectObj, bus));

    % set the first bus element as the starting point of the signal structure 
    SignalStructure = [{debugBus.Elements.Name}', {debugBus.Elements.DataType}', {debugBus.Elements.Dimensions}']; 
    Finished = false; 
    % index of next parsed signal
    actualParseIdx = 1; 
    while(~Finished)
      % check if next signal is a bus 
      if(strcmp(SignalStructure{actualParseIdx, 2}(1:4), 'Bus:'))
        nextBusName = SignalStructure{actualParseIdx, 2}(6:end); 
        previousSignalName = SignalStructure{actualParseIdx, 1};
        % replace the actual signal with the content of the sub bus 
        nextBus = getValue(getEntry(dDataSectObj, nextBusName));
        % save remaining structure of bus elements
        remainingSignalStructure = SignalStructure((actualParseIdx+1):end, :); 
        % remove signals in signal structure 
        SignalStructure((actualParseIdx):end, :) = [];  
        % fill up with signals of sub bus
        for i = 1:1:length(nextBus.Elements)
          SignalStructure(actualParseIdx+(i-1), :) = ...
            {[previousSignalName '.' nextBus.Elements(i).Name], nextBus.Elements(i).DataType, nextBus.Elements(i).Dimensions}; 
        end
        % add old signals 
        SignalStructure = [SignalStructure;...
          remainingSignalStructure]; 
      else
        actualParseIdx = actualParseIdx+1; 
        % check if the last element is reached and it is not a bus element 
        if(actualParseIdx == length(SignalStructure) &&...
            ~strcmp(SignalStructure{actualParseIdx, 2}(1:4), 'Bus:'))
          Finished = true; 
        end
      end
    end

    %% design blocks 
    block_height = 10; 
    block_width = 30; 
    hor_offset = 200; 
    vert_offset = 30; 

    % add bus selector block 
    % generate bus output string 
    bus_outputstring = [SignalStructure{1, 1}]; 
    for i = 2:1:length(SignalStructure) 
      bus_outputstring = [bus_outputstring ',' SignalStructure{i, 1}]; 
    end
    % add bus selector 
    add_block('built-in/BusSelector', [sys '/BusOutput'], 'OutputSignals', bus_outputstring, 'Position', [-500, -vert_offset/2, -490, length(SignalStructure)*vert_offset+block_height-vert_offset/2]); 
    % add mux for file scope
    switch target
        case 'Speedgoat'
            numMuxSignalIns = length(SignalStructure)+1; 
        case 'ERT'
            numMuxSignalIns = length(SignalStructure); 
    end
    add_block('built-in/Mux', [sys '/Mux'], 'Inputs', num2str(numMuxSignalIns), 'DisplayOption', 'bar', 'Position', [500, -vert_offset/2, 500+10, numMuxSignalIns*vert_offset+block_height-vert_offset/2]);
    % add system inport 
    add_block('built-in/Inport', [sys '/debugIn'], 'Position', [-700, 0, -670, 15], 'BackgroundColor', 'green', 'OutDataTypeStr', ['Bus: ' bus]); 
    % connect it to bus selector 
    add_line(sys, 'debugIn/1', 'BusOutput/1'); 

    % iterate via all signals
    for i = 1:1:length(SignalStructure)
      % create data type conversion blocks 
      if(strcmp(target, 'Speedgoat'))
        add_block('built-in/DataTypeConversion', [sys '/Convert' num2str(i)], 'DataType', 'single', 'Position', [0, (i-1)*vert_offset, block_width, (i-1)*vert_offset+block_height], 'BackgroundColor', 'yellow'); 
      else
        add_block('built-in/DataTypeConversion', [sys '/Convert' num2str(i)], 'DataType', 'double', 'Position', [0, (i-1)*vert_offset, block_width, (i-1)*vert_offset+block_height], 'BackgroundColor', 'yellow'); 
      end
      % create signal conversion blocks with signal name necessary for debug structure 
      signal_description{i} = strrep(SignalStructure{i, 1}, '.', '_');
      add_block('built-in/SignalConversion', [sys '/debug_' signal_description{i}], 'Position', [hor_offset, (i-1)*vert_offset, hor_offset+block_width, (i-1)*vert_offset+block_height], 'BackgroundColor', 'yellow'); 
      % connect them 
      add_line(sys, ['Convert' num2str(i) '/1'], ['debug_' signal_description{i} '/1'], 'autorouting', 'on');
      add_line(sys, ['BusOutput/' num2str(i)], ['Convert' num2str(i) '/1']); 
      add_line(sys, ['debug_' signal_description{i} '/1'], ['Mux/' num2str(i)]); 
    end
    % add Task Execution time block as last signal if target is speedgoat 
    if(strcmp(target, 'Speedgoat')) 
        % create data type conversion blocks 
        add_block('built-in/DataTypeConversion', [sys '/Convert' num2str(length(SignalStructure)+1)], 'DataType', 'single', 'Position', [0, length(SignalStructure)*vert_offset, block_width, length(SignalStructure)*vert_offset+block_height], 'BackgroundColor', 'yellow'); 
        % create signal conversion blocks with signal name necessary for debug structure 
        add_block('built-in/SignalConversion', [sys '/debug_TET_s'], 'Position', [hor_offset, length(SignalStructure)*vert_offset, hor_offset+block_width, length(SignalStructure)*vert_offset+block_height], 'BackgroundColor', 'yellow'); 
        % create TET block 
        add_block('slrtlib/Target Management/Execution Parameters/Task Execution Time ',[sys '/TET'],'Position',[-150, length(SignalStructure)*vert_offset, block_width-150, length(SignalStructure)*vert_offset+block_height], 'BackgroundColor', 'yellow');
        % connect them 
        add_line(sys, ['Convert' num2str(length(SignalStructure)+1) '/1'], ['debug_TET_s/1'], 'autorouting', 'on');
        add_line(sys, ['debug_TET_s/1'], ['Mux/' num2str(length(SignalStructure)+1)]); 
        add_line(sys, ['TET/1'], ['Convert' num2str(length(SignalStructure)+1) '/1']);
        signal_description{length(signal_description)+1} = 'debug_TET_s'; 
        SignalStructure{length(signal_description), 1} = 'TET_s'; 
        SignalStructure{length(signal_description), 2} = 'double'; 
        SignalStructure{length(signal_description), 3} = [1]; 
        set_param([sys '/TET'], 'ts', 'tS')
    end

    switch target
        % add logging scope in case of speedgoat
        case 'Speedgoat'
            filescope = add_block('slrtlib/Displays and Logging/Scope ',  [sys '/LoggingScope'], 'Position', [570,  length(SignalStructure)*vert_offset/2-30, 650, length(SignalStructure)*vert_offset/2+30]);
            % set scope number depending on bus given (if not none use standard)
            switch bus
                case 'mvdc_tube_mpc_debug' 
                    set_param(filescope,'scopeno','7'); 
                case 'mvdc_vehicle_dynamics_control_debug' 
                    set_param(filescope,'scopeno','8'); 
            end
            set_param(filescope,'scopetype','File'); 
            set_param(filescope,'nosamples','1000'); 
            set_param(filescope,'filename', ['C:\LOGS\' outfile '_001.dat']);
            set_param(filescope,'maxwritefilesize','512*2048^2'); 
            set_param(filescope,'writesize','10*512'); 
            set_param(filescope,'mode','Commit'); 
            set_param(filescope,'AutoRestart','On'); 
            add_line(sys, 'Mux/1', 'LoggingScope/1'); 
            add_block('simulink/Ports & Subsystems/Enable', [sys '/Enable'], 'Position', [-600, 0, -580, 20]); 
        % add subsystem output in case of ERT target and generate additional signal description file
        case 'ERT'  
            add_block('simulink/Sinks/Out1',  [sys '/DebugOut'], 'Position', [570,  length(SignalStructure)*vert_offset/2-30, 650, length(SignalStructure)*vert_offset/2+30]);
            add_line(sys, 'Mux/1', 'DebugOut/1'); 
            % output the description file 
            fid = fopen([outfile '.txt'], 'w');
            for row = 1:length(signal_description)
                % check dimension of signal 
                nDim = SignalStructure{row, 3};
                if(nDim > 1) 
                    for loc_idx = 1:1:nDim
                        fprintf(fid, 'debug_%s_%d\n', signal_description{row}, loc_idx);
                    end
                else
                    fprintf(fid, 'debug_%s\n', signal_description{row});
                end
            end
            fclose(fid);
    end
        
    % save model to new file
    projectRoot = evalin('base', 'projectRoot_vdc');
    save_system(sys, [projectRoot '/system/models/debug_conversion_' outfile '.slx']); 