%
% Copyright (c) 2011, KTH Royal Institute of Technology
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without modification,
% are permitted provided that the following conditions are met:
%
%  - Redistributions of source code must retain the above copyright notice, this list
% 	  of conditions and the following disclaimer.
%
%  - Redistributions in binary form must reproduce the above copyright notice, this
%    list of conditions and the following disclaimer in the documentation and/or other
%	  materials provided with the distribution.
%
%  - Neither the name of the KTH Royal Institute of Technology nor the names of its
%    contributors may be used to endorse or promote products derived from this software
%    without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
% INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
% WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
% OF SUCH DAMAGE.
%

% @author Aitor Hernandez <aitorhh@kth.se>
% 
% @version  $Revision: 1.0 Date: 2011/06/15 $ 
%

function varargout = testSerialComm_gui(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @testSerialComm_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @testSerialComm_gui_OutputFcn, ...
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




% --- Executes just before testSerialComm_gui is made visible.
function testSerialComm_gui_OpeningFcn(hObject, eventdata, handles, varargin)
global TestSerial;
    
% Choose default command line output for testSerialComm_gui
handles.output = hObject;

guidata(hObject,handles)
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');
set(handles.showPlots,'Enable','off');
set(handles.constantTxt,'Enable','off');
set(handles.compileButton,'Enable','off');
set(handles.saveButton,'Enable','off');
set(handles.lowerTankLevel,'Enable','off');

axes(handles.imageKth);
imshow('kth_cmyk.png');

testSerialComm('init');

% Set the sent value and received in the TestSerial struct
TestSerial.sentValue = handles.sentValue;
TestSerial.receivedValue = handles.receivedValue;

% UIWAIT makes testSerialComm_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


function varargout = testSerialComm_gui_OutputFcn(hObject, eventdata, handles) 


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
testSerialComm('startP', get(handles.tutorialItem,'Value'));

% toggle the buttons
% Turn off the Start button
set(handles.startpb,'Enable','off');
% Turn on the stoppb button
set(handles.stoppb,'Enable','on');
set(handles.pausepb,'Enable','on');
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
testSerialComm('stopP');
testSerialComm('disconnectP');


% --- Executes during object creation, after setting all properties.
function imageKth_CreateFcn(hObject, eventdata, handles)
% Hint: place code in OpeningFcn to populate image



% --- Executes on button press in pausepb.
function pausepb_Callback(hObject, eventdata, handles)
% hObject    handle to pausepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

testSerialComm('stopP');

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

testSerialComm('stopP');
testSerialComm('disconnectP');

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
testSerialComm('compile');


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
testSerialComm('saveInformation');

function showPlots_Callback(hObject, eventdata, handles)
testSerialComm('showPlots');



%
% E N D   G U I   C A L L B A C K S
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% T U T O R I A L    F U N C T I O N S
%

% --- Executes on selection change in tutorialItem.
function tutorialItem_Callback(hObject, eventdata, handles)
testSerialComm('stopP');
testSerialComm('startP', get(hObject,'Value'));



% --- Executes during object creation, after setting all properties.
function tutorialItem_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tutorialItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%
% E N D   T U T O R I A L   T O O L S
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
testSerialComm('changeController', get(hObject,'Value'));

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
