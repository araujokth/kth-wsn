function varargout = waterTanksSelfTriggered_gui(varargin)


% waterTanksSelfTriggered_gui M-file for waterTanksSelfTriggered_gui.fig
%      waterTanksSelfTriggered_gui, by itself, creates a new waterTanksSelfTriggered_gui or raises the existing
%      singleton*.
%
%      H = waterTanksSelfTriggered_gui returns the handle to a new waterTanksSelfTriggered_gui or the handle to
%      the existing singleton*.
%
%      waterTanksSelfTriggered_gui('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in waterTanksSelfTriggered_gui.M with the given input arguments.
%
%      waterTanksSelfTriggered_gui('Property','Value',...) creates a new waterTanksSelfTriggered_gui or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before waterTanksSelfTriggered_gui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stoppb.  All inputs are passed to waterTanksSelfTriggered_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help waterTanksSelfTriggered_gui

% Last Modified by GUIDE v2.5 22-Jun-2011 15:14:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @waterTanksSelfTriggered_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @waterTanksSelfTriggered_gui_OutputFcn, ...
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




% --- Executes just before waterTanksSelfTriggered_gui is made visible.
function waterTanksSelfTriggered_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% Initialization of the WTSelf to detect if we need to use the default or
% the new values for the compilation
global WTSelf;
    
% Choose default command line output for waterTanksSelfTriggered_gui
handles.output = hObject;

% Save where are the variables for the new scneario
WTSelf.gui.numWaterTanks = handles.numWaterTanksST;
WTSelf.gui.numSensors = handles.numSensors;
WTSelf.gui.channel = handles.channel;
WTSelf.gui.panID = handles.panID;
WTSelf.gui.offsetWT = handles.offsetMotelist;

guidata(hObject,handles)
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');
%set(handles.showPlots,'Enable','off');
set(handles.saveButton,'Enable','off');
set(handles.controllerMenu,'Enable','inactive');
set(handles.lowerTankLevel,'Enable','off');
set(handles.constantTxt,'Enable','off');

axes(handles.imageKth);
imshow('kth_cmyk.png');

waterTanksSelfTriggered('init');


% UIWAIT makes waterTanksSelfTriggered_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = waterTanksSelfTriggered_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% G U I   C A L L B A C K S
%
%
%
% Callback Function for Start button
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in startpb.
function startpb_Callback(hObject, eventdata, handles)
% hObject    handle to startpb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
waterTanksSelfTriggered('init');
waterTanksSelfTriggered('startP');

% toggle the buttons
% Turn off the Start button
set(handles.startpb,'Enable','off');
% Turn on the stoppb button
set(handles.stoppb,'Enable','on');
set(handles.pausepb,'Enable','on');
set(handles.controllermenu,'Enable','on');

set(handles.saveButton,'Enable','on');


guidata(hObject,handles)


%
% Callback Function for Stop button
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function stoppb_Callback(hObject, eventdata, handles)
% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% ad    structure with handles and user data (see GUIDATA)

% toggle the buttons
% Turn on the Start button
set(handles.startpb,'Enable','on');
set(handles.showPlots,'Enable','on');
set(handles.lowerTankLevel,'Enable','off');

% Turn off the stoppb button
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');

% Disable Consntant

%stop(handles.t);
% mat_file = sprintf('%s/data_%s.m',WTAPP.OUTPUTS_PATH , ...
%     datestr(d.date, 'yymmdd_HHMMSS'));
waterTanksSelfTriggered('stopP');
waterTanksSelfTriggered('disconnectP');


% --- Executes during object creation, after setting all properties.
function imageKth_CreateFcn(hObject, eventdata, handles)
% Hint: place code in OpeningFcn to populate image


% --- Executes on button press in pausepb.
function pausepb_Callback(hObject, eventdata, handles)
% hObject    handle to pausepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

waterTanksSelfTriggered('stopP');

set(handles.controllerMenu, 'Value', 4);
% Turn on the Start button
set(handles.startpb,'Enable','on');
% Turn off the stoppb button
set(handles.stoppb,'Enable','on');


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


%
% Callback Function for deleting the UI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localCloseRequestFcn(hObject,eventdata) %#ok

% get the application data
ad = guidata(hObject);

waterTanksSelfTriggered('stopP');
waterTanksSelfTriggered('disconnectP');

% Can only close the UI if the model has been stopped
% Can only stop the model is it hasn't already been unloaded (perhaps
% manually).
% Remove the listener on the Gain block in the model's StartFcn
% destroy the window
delete(gcbo);

% --- Executes during object creation, after setting all properties.
function constantTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to constantTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in compileButton.
function compileButton_Callback(hObject, eventdata, handles)
% hObject    handle to compileButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
waterTanksSelfTriggered('compile');


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
waterTanksSelfTriggered('saveInformation');

function showPlots_Callback(hObject, eventdata, handles)
waterTanksSelfTriggered('showPlots', get(handles.showPlotsController,'Value'));



%
% E N D   G U I   C A L L B A C K S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% O T H E R   T O O L S
%


%
% E N D   O T H E R   T O O L S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% --- Executes on button press in controllerMenu.
function controllerMenu_Callback(hObject, eventdata, handles)
global WTModeling;
global WTSelf;

WTModeling.uConstant = str2double(get(handles.constantTxt,'String'));
waterTanksSelfTriggered('changeController', get(hObject,'Value'));

if get(hObject,'Value') ~= WTSelf.CONSTANT_CONTROLLER
    set(handles.constantTxt,'Enable','off');
else
        set(handles.constantTxt,'Enable','on');

end

function controllerMenu_CreateFcn(hObject, eventdata, handles)

function constantTxt_Callback(hObject, eventdata, handles)
global WTModeling;
WTModeling.uConstant = str2double(get(hObject,'String'));


% --- Executes on key press with focus on constantTxt and none of its controls.
function constantTxt_KeyPressFcn(hObject, eventdata, handles)
global WTModeling;
WTModeling.uConstant = str2double(get(hObject,'String'));

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
waterTanksSelfTriggered('resetIntegral');



% --- Executes on selection change in showPlotsController.
function showPlotsController_Callback(hObject, eventdata, handles)
% hObject    handle to showPlotsController (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns showPlotsController contents as cell array
%        contents{get(hObject,'Value')} returns selected item from
%        showPlotsController


function lowerTankLevel_Callback(hObject, eventdata, handles)
% hObject    handle to lowerTankLevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lowerTankLevel as text
%        str2double(get(hObject,'String')) returns contents of lowerTankLevel as a double


% --- Executes during object creation, after setting all properties.
function lowerTankLevel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lowerTankLevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function showPlotsController_CreateFcn(hObject, eventdata, handles)
% hObject    handle to showPlotsController (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% C O M P I L A T I O N    O P T I O N S
%

function numWaterTanksST_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function numWaterTanksST_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function numSensors_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function numSensors_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function channel_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function channelTxt_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function panID_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function panID_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function offsetMotelist_Callback(hObject, eventdata, handles)
% hObject    handle to offsetMotelist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of offsetMotelist as text
%        str2double(get(hObject,'String')) returns contents of offsetMotelist as a double


% --- Executes during object creation, after setting all properties.
function offsetMotelist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to offsetMotelist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% 
% E N D    C O M P I L A T I O N    O P T I O N S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes during object creation, after setting all properties.
function channel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when uipanel16 is resized.
function uipanel16_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to uipanel16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function numWaterTanksET_Callback(hObject, eventdata, handles)
% hObject    handle to numWaterTanksET (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of numWaterTanksET as text
%        str2double(get(hObject,'String')) returns contents of numWaterTanksET as a double


% --- Executes during object creation, after setting all properties.
function numWaterTanksET_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numWaterTanksET (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
