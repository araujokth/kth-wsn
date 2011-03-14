function varargout = simulink_gui(varargin)


% SIMULINK_GUI M-file for simulink_gui.fig
%      SIMULINK_GUI, by itself, creates a new SIMULINK_GUI or raises the existing
%      singleton*.
%
%      H = SIMULINK_GUI returns the handle to a new SIMULINK_GUI or the handle to
%      the existing singleton*.
%
%      SIMULINK_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULINK_GUI.M with the given input arguments.
%
%      SIMULINK_GUI('Property','Value',...) creates a new SIMULINK_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simulink_gui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stoppb.  All inputs are passed to simulink_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simulink_gui

% Last Modified by GUIDE v2.5 10-Feb-2011 19:37:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simulink_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @simulink_gui_OutputFcn, ...
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




% --- Executes just before simulink_gui is made visible.
function simulink_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simulink_gui (see VARARGIN)
    
% Choose default command line output for simulink_gui
handles.output = hObject;
modelName = 'sensor_block_pulse';
handles.sfPort = 9002;
handles.usbPort = 0;

% Do some simple error checking on the input
if ~localValidateInputs(modelName)
    estr = sprintf('The model %s.mdl cannot be found.',modelName);
    errordlg(estr,'Model not found error','modal');
    return
end
% load the model
handles = localLoadModel(modelName, handles);
       
%% Configure the Figure
hf = findall(0, 'tag', 'figure1');

set(hf, 'Toolbar','none',...
        'MenuBar','none',...
        'IntegerHandle','off',...
        'Units','normalized',...
        'NumberTitle','off',...
        'HandleVisibility','callback',...
        'Name',sprintf('Custom UI for controlling and showing %s.mdl',modelName));
    

guidata(hObject,handles)
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');

% UIWAIT makes simulink_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = simulink_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



%
% Function to load model and get certain of its parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = localLoadModel(modelName, handles)

% Load the simulink model
if ~modelIsLoaded(modelName)
    load_system(modelName);
end
% Create some application data storing various
% pieces of information about the model's original state.
% These will be used to "reset" the model to its original state when
% the UI is closed.
handles.modelName = modelName;
handles.smoothVector = [1 2 3 0 5 6 7 8];


% List the blocks that are to have listeners applied
handles.viewing = struct(...
    'blockName','',...
    'blockHandle',[],...
    'blockEvent','',...
    'blockFcn',[]);

handles.commComponents = '';        % var to indicate which sensor we are analysing

%
% Every block has a name
handles.viewing(1).blockName = sprintf('%s/xc_value',handles.modelName);
handles.viewing(2).blockName = sprintf('%s/theta_value',handles.modelName);

handles.viewing(3).blockName = sprintf('%s/delay',handles.modelName);
handles.viewing(4).blockName = sprintf('%s/errors',handles.modelName);
handles.viewing(5).blockName = sprintf('%s/reliability',handles.modelName);

handles = localCreateFigures(handles);

for i=1:length(handles.viewing)
    % That block has a handle
    % (This will be used in the graphics drawing callback, and is done here
    % as it should speed things up rather than searching for the handle
    % during every event callback.)
    handles.viewing(i).blockHandle = get_param(handles.viewing(i).blockName,'Handle');
    % List the block event to be listened for
    handles.viewing(i).blockEvent = 'PostOutputs'; 
    handles.viewing(i).blockFcn = @localEventListener;
end

% List the function to be called
% (These must be subfunctions within this mfile).

% Save some of the models original info that this UI may change
% (and needs to change back again when the simulation stops)
handles.originalStopTime = get_param(handles.modelName,'Stoptime');
handles.originalMode =  get_param(handles.modelName,'SimulationMode');
handles.originalStartFcn = get_param(handles.modelName,'StartFcn');

out = handles;
% end localLoadModel(modelName, handles)


function out = localCreateFigures(handles)

handles.plotAxes(1) = handles.plotAxesXc;
handles.plotAxes(2) = handles.plotAxesTheta;

handles.plotAxes(3) = handles.plotAxesDelay;
handles.plotAxes(4) = handles.plotAxesPacketLoss;
handles.plotAxes(5) = handles.plotAxesReliability;


for i=1:5
    xlabel(handles.plotAxes(i), 'Time'); 
    set(handles.plotAxes(i), ...
        'HandleVisibility','callback',...
        'Unit','normalized',...
        'Xlim',[0 500]);
    if i == 4
        legend( handles.plotAxes(i),'asdfasdf', 'asdfadf', 'asdfasdf' );
    end
end

ylabel(handles.plotAxesXc, 'x_c [mm]');
ylabel(handles.plotAxesTheta, '\theta [degrees]'); 
        
ylabel(handles.plotAxesDelay, 'Delay [ms]');
ylabel(handles.plotAxesPacketLoss, 'Packet Loss');
    set(handles.plotAxes(i), 'Ylim',[-0.1 1.1]);
ylabel(handles.plotAxesReliability, 'Reliability');
    set(handles.plotAxes(i), 'Ylim',[-0.1 1.1]);
    
nlines = length(handles.viewing);
hl = nan(1,nlines);
colourOrder = get(handles.plotAxes(1),'ColorOrder');

for idx=1:nlines
    axes = idx; colors = idx;
%     if axes >= 4 && axes <= 6 
%         axes = 4; colors = idx - axes;
%     elseif axes >= 6 axes = 5; colors = idx - axes;end
    
    hl(idx) = line('Parent',handles.plotAxes(axes),...
        'XData',[],...
        'YData',[],...
        'Color',colourOrder(mod(colors + idx - axes -1,size(colourOrder,1))+1,:),...
        'LineWidth', 2, ...
        'EraseMode','xor',...
        'Tag',sprintf('signalLine%d',idx));

end
handles.lineHandles = hl;

out = handles;
% end localCreateFigures(modelName, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% L I S T E N E R   M A N A G E R
%
%
%
% Callback Function for adding an event listener to the gain block
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function localAddEventListener

% get the application data
handles = guidata(gcbo);

% execute any original startFcn that the model may have had
if ~isempty(handles.originalStartFcn)
    evalin('Base',handles.originalStartFcn);
end

% Add the listener(s)
% For this example all events call into the same function
% handles.eventHandle = cell(1,2);
%  handles.eventHandle{1} = ...
%         add_exec_event_listener(handles.viewing(3).blockName,...
%         handles.viewing(3).blockEvent, handles.viewing(3).blockFcn);
%      handles.eventHandle{2} = ...
%         add_exec_event_listener(handles.viewing(4).blockName,...
%         handles.viewing(4).blockEvent, handles.viewing(4).blockFcn);
    
handles.eventHandle = cell(1,length(handles.viewing));
for idx = 1:length(handles.viewing)
    handles.eventHandle{idx} = ...
        add_exec_event_listener(handles.viewing(idx).blockName,...
        handles.viewing(idx).blockEvent, handles.viewing(idx).blockFcn);
end

% store the changed app data
guidata(gcbo,handles);


%
% Callback Function for executing the event listener on the gain block
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localEventListener(block, eventdata) %#ok

% Note: this callback is called by all the block listeners.  No effort has
% been made to time synchronise the data from each signal.  Rather it is
% assumed that since each block calls this function at every time step and
% hence the time synchronisation will come "for free".  This may not be the
% case for other models and additional code may be required for them to
% work/display data correctly.

% get the application data
%hf = findall(0,'tag',mfilename);
hf = findall(0, 'tag', 'figure1');
handles = guidata(hf);

% Get the handle to the line that currently needs updating
idx = [handles.viewing.blockHandle]==block.BlockHandle;

sTime = block.CurrentTime;
data = block.InputPort(1).Data;
% idxLine = find(idx, 1);
% if isequal(handles.commComponents, '') && (idx(4) || idx(5))
%     % update the 3 lines (xc, theta, actuator)
%     idxLine = idxLine;
%     for i=idxLine:idxLine
%         thisLineHandle = handles.lineHandles(i);
%         
%         xdata = get(thisLineHandle,'XData');
%         ydata = get(thisLineHandle,'YData');
%         
%         if length(xdata) < 201
%             newXData = [xdata sTime];
%             newYData = [ydata data(i-idxLine+1)];
%         else
%             newXData = [xdata(2:end) sTime];
%             newYData = [ydata(2:end) data(i-idxLine+1)];
%         end
%         set(thisLineHandle,...
%             'XData',newXData,...
%             'YData',newYData);
%     end
% else
    % Get the simulation time and the block data
    thisLineHandle = handles.lineHandles(idx);
        
    % Get the data currently being displayed on the axis
    xdata = get(thisLineHandle,'XData');
    ydata = get(thisLineHandle,'YData');
        
    if length(xdata) < 201
        newXData = [xdata sTime];
        newYData = [ydata data];
    else
        newXData = [xdata(2:end) sTime];
        newYData = [ydata(2:end) data];
    end
    set(thisLineHandle,...
        'XData',newXData,...
        'YData',newYData);
%end

% The axes limits may also need changing
%newXLim = [sTime-1 sTime];
newXLim = [max(0,sTime-1) max(1,sTime)];

set(handles.plotAxes(idx),'Xlim',newXLim);



%
% Callback Function for removing the event listener from the gain block
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localRemoveEventListener

% get the application data
handles = guidata(gcbo);

% return the startFcn to its original value
set_param(handles.modelName,'StartFcn',handles.originalStartFcn);

% delete the listener(s)
for idx = 1:length(handles.eventHandle)
    if ishandle(handles.eventHandle{idx})
        delete(handles.eventHandle{idx});
    end
end
% remove this field from the app data structure
handles = rmfield(handles,'eventHandle');
%save the changes
guidata(gcbo,handles);

%
%
% E N D   L I S T E N E R   M A N A G E R
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% G U I   C A L L B A C K S
%
%
%
% Callback Function for Start button
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localStartPressed(hObject, eventdata, ad)
% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% ad    structure with handles and user data (see GUIDATA)

% Load the model if required (it may have been closed manually).
if ~modelIsLoaded(ad.modelName)
    load_system(ad.modelName);
end

% toggle the buttons
% Turn off the Start button
set(ad.startpb,'Enable','off');
% Turn on the stoppb button
set(ad.stoppb,'Enable','on');
set(ad.pausepb,'Enable','on');

% Run serial forwarder
unix(sprintf('sf %d /dev/ttyUSB%d 115200 &', ad.sfPort, ad.usbPort));

%str2num(get(handles.sfport_v,'String')), ...
%str2num(get(handles.usbport_v,'String')) ));

%options = simset('SrcWorkspace','current');

% set the stoppb time to inf
set_param(ad.modelName,'StopTime','inf');
% set the simulation mode to normal
%set_param(ad.modelName,'SimulationMode','normal');
%set_param(ad.modelName,'Solver','FixedStepDiscrete','FixedStep', '0.01');
set_param(ad.modelName,'StartFcn','localAddEventListener');
% start the model
set_param(ad.modelName,'SimulationCommand','start');

% reset the line(s)
for idx = 1:length(ad.lineHandles)
    set(ad.lineHandles(idx),...
        'XData',[],...
        'YData',[]);
end


%
% Callback Function for Stop button
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localStopPressed(hObject, eventdata, ad)
% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% ad    structure with handles and user data (see GUIDATA)

% stop the model
set_param(ad.modelName,'SimulationCommand','stop');


% set model properties back to their original values
set_param(ad.modelName,'Stoptime',ad.originalStopTime);
set_param(ad.modelName,'SimulationMode',ad.originalMode);

% reset the line(s)
for idx = 1:length(ad.lineHandles)
    set(ad.lineHandles(idx),...
        'XData',[],...
        'YData',[]);
end

% toggle the buttons
% Turn on the Start button
set(ad.startpb,'Enable','on');
% Turn off the stoppb button
set(ad.stoppb,'Enable','off');
set(ad.pausepb,'Enable','off');



% Remove the listener on the Gain block in the model's StartFcn
localRemoveEventListener;
unix('killall sf');



% --- Executes during object creation, after setting all properties.
function imageCreateFcn(hObject, eventdata, handles)
% hObject    handle to image (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate image
axes(hObject);
imshow('kth_cmyk.png');


% --- Executes on button press in pausepb.
function pausepbPressed(hObject, eventdata, ad)
% hObject    handle to pausepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_param(ad.modelName,'SimulationCommand','pause');

% Turn on the Start button
set(ad.startpb,'Enable','on');
% Turn off the stoppb button
set(ad.stoppb,'Enable','on');


% --------------------------------------------------------------------
function helpmenu_Callback(hObject, eventdata, handles)
% hObject    handle to helpmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




%
% Callback Function for viewing an about box
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function localAboutPulldown(hObject, eventdata, handles)
% hObject    handle to about (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Create an about box
str = {'This GUI was written by Aitor Hernandez {aitorhh@kth.se}\n'; ...
    'Version 1.0 \n';...
    '2011 - KTH | Automatic Control'};
msgbox(str,'About Box','Help','modal')



% --- Executes on selection change in selector.
function localSelectorChanged(hObject, eventdata, handles)
% hObject    handle to selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns selector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from selector
%gets the selected option
idx = [3 4 5]; 
%removeListeners(handles, idx);  %remove listener
%resetLines(handles, idx);       %reset the lines in the plots

switch get(handles.selector,'Value')   
    case 1
        component = '';

    case 2
        component = 'xc_';
    case 3
        %remove listener
        component = 'theta_';
    case 4
        component = 'actuator_';
end

if ~isequal(component, handles.commComponents)
    handles.commComponents = component;
    
    handles.viewing(3).blockName = sprintf('%s/%sdelay', ...
        handles.modelName, component);
    handles.viewing(4).blockName = sprintf('%s/%serrors',...
        handles.modelName, component);
    handles.viewing(5).blockName = sprintf('%s/%sreliability',...
        handles.modelName, component);
    for i=idx
        if ishandle(handles.eventHandle{i})
            delete(handles.eventHandle{i});
        end
        set(handles.lineHandles(i),...
            'XData',[],...
            'YData',[]);
        handles.viewing(i).blockHandle = get_param(handles.viewing(i).blockName,'Handle');
        % List the block event to be listened for
        handles.viewing(i).blockEvent = 'PostOutputs';
        handles.viewing(i).blockFcn = @localEventListener;
        
        handles.eventHandle{i} = ...
            add_exec_event_listener(handles.viewing(i).blockName,...
            handles.viewing(i).blockEvent, handles.viewing(i).blockFcn);
    end
    
end
guidata(gcbo,handles);


% --- Executes during object creation, after setting all properties.
function localSelectorCreateFcn(hObject, eventdata, handles)
% hObject    handle to selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%
% Callback Function for deleting the UI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localCloseRequestFcn(hObject,eventdata) %#ok

% get the application data
ad = guidata(hObject);

% Can only close the UI if the model has been stopped
% Can only stop the model is it hasn't already been unloaded (perhaps
% manually).
if modelIsLoaded(ad.modelName)
    switch get_param(ad.modelName,'SimulationStatus');
        case 'stopped'
            % Reset the gain to its original value
            set_param(ad.tuning.blockName,ad.tuning.blockProp,ad.originalGainValue);
            % close the Simulink model
            close_system(ad.modelName,0);
            % destroy the window
            delete(gcbo);
        otherwise
            errordlg('The model must be stopped before the UI is closed',...
                'UI Close error','modal');
    end
else
    % destroy the window
    delete(gcbo);
end


%
% E N D   G U I   C A L L B A C K S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% O T H E R   T O O L S
%
%
% Function to check that model is still loaded
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function modelLoaded = modelIsLoaded(modelName)

try
    modelLoaded = ...
        ~isempty(find_system('Type','block_diagram','Name',modelName));
catch ME %#ok
    % Return false if the model can't be found
    modelLoaded = false;
end



function modelExists = localValidateInputs(modelName)

num = exist(modelName,'file');
if num == 4
    modelExists = true;
else
    modelExists = false;
end

%
% E N D   O T H E R   T O O L S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
