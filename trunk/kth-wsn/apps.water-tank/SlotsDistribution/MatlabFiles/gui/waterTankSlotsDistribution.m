function varargout = waterTankSlotsDistribution(varargin)


% WATERTANKSLOTSDISTRIBUTION M-file for waterTankSlotsDistribution.fig
%      WATERTANKSLOTSDISTRIBUTION, by itself, creates a new WATERTANKSLOTSDISTRIBUTION or raises the existing
%      singleton*.
%
%      H = WATERTANKSLOTSDISTRIBUTION returns the handle to a new WATERTANKSLOTSDISTRIBUTION or the handle to
%      the existing singleton*.
%
%      WATERTANKSLOTSDISTRIBUTION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WATERTANKSLOTSDISTRIBUTION.M with the given input arguments.
%
%      WATERTANKSLOTSDISTRIBUTION('Property','Value',...) creates a new WATERTANKSLOTSDISTRIBUTION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simulink_gui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stoppb.  All inputs are passed to waterTankSlotsDistribution_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help waterTankSlotsDistribution

% Last Modified by GUIDE v2.5 08-Jun-2011 11:25:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @waterTankSlotsDistribution_OpeningFcn, ...
                   'gui_OutputFcn',  @waterTankSlotsDistribution_OutputFcn, ...
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




% --- Executes just before waterTankSlotsDistribution is made visible.
function waterTankSlotsDistribution_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to waterTankSlotsDistribution (see VARARGIN)
    
% Choose default command line output for waterTankSlotsDistribution
handles.output = hObject;
handles.sfPort = 9002;
handles.usbPort = 0;

handles.hl = line('Parent',handles.u,...
        'XData',[],...
        'YData',[],...
        'LineWidth', 2, ...
        'EraseMode','xor');
    
guidata(hObject,handles)
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');
set(handles.selector,'Value', 1);

axes(handles.imageKth);
imshow('kth_cmyk.png');




% UIWAIT makes waterTankSlotsDistribution wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = waterTankSlotsDistribution_OutputFcn(hObject, eventdata, handles) 
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
function localStartPressed(hObject, eventdata, handles)
global WTPLOTS;

% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% ad    structure with handles and user data (see GUIDATA)

handles.t = timer('Name', 'Timer','TimerFcn',{'waterTankApp', 'plotResultsWTPLOTS'},'ExecutionMode','fixedRate','Period',1);
%start(handles.t);
unix(sprintf('sf %d /dev/ttyUSB%d 115200 &', handles.sfPort, handles.usbPort))


WTPLOTS.uAxes = handles.u;
WTPLOTS.upperTankAxes = handles.upperTank;
WTPLOTS.lowerTankAxes = handles.lowerTank;

waterTankApp('init');
waterTankApp('startP');

% toggle the buttons
% Turn off the Start button
set(handles.startpb,'Enable','off');
% Turn on the stoppb button
set(handles.stoppb,'Enable','on');
set(handles.pausepb,'Enable','on');
guidata(hObject,handles)





%
% Callback Function for Stop button
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function localStopPressed(hObject, eventdata, handles)
% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% ad    structure with handles and user data (see GUIDATA)

% toggle the buttons
% Turn on the Start button
set(handles.startpb,'Enable','on');
% Turn off the stoppb button
set(handles.stoppb,'Enable','off');
set(handles.pausepb,'Enable','off');

set(handles.selector,'Value', 1);
% Disable Consntant
set(handles.updatepb,'Enable','off');
set(handles.constantTxt,'Enable','off');

%stop(handles.t);
% mat_file = sprintf('%s/data_%s.m',WTAPP.OUTPUTS_PATH , ...
%     datestr(d.date, 'yymmdd_HHMMSS'));
waterTankApp('stopP');
waterTankApp('disconnectP');

unix('killall sf');


% --- Executes during object creation, after setting all properties.
function imageCreateFcn(hObject, eventdata, handles)
% hObject    handle to image (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate image


% --- Executes on button press in pausepb.
function pausepbPressed(hObject, eventdata, ad)
% hObject    handle to pausepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

waterTankApp('stopP');


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
switch get(hObject,'Value')
    case 2
        set(handles.updatepb,'Enable','on');
        set(handles.constantTxt,'Enable','on');
    otherwise
        set(handles.updatepb,'Enable','off');
        set(handles.constantTxt,'Enable','off');
end


waterTankApp('changeController', get(handles.selector,'Value'), ...
    str2double(get(handles.constantTxt,'String')));

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

stop(ad.t);
waterTankApp('stopP');
waterTankApp('disconnectP');

unix('killall sf');
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


% --- Executes on button press in updatepb.
function updatepb_Callback(hObject, eventdata, handles)
% hObject    handle to updatepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

waterTankApp('changeController', get(handles.selector,'Value'), ...
    str2double(get(handles.constantTxt,'String')));

% --- Executes on button press in updatePlots.
function updatePlots_Callback(hObject, eventdata, handles)
% hObject    handle to updatePlots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% disp(handles.u);
% disp(handles.upperTank)
% disp(handles.lowerTank)
waterTankApp('plotResultsWTPLOTS', handles.u, handles.upperTank, handles.lowerTank);


function plotExternalFigures
global WTPLOTS;

plotsMED();

figure;
subplot(4,1,1)
plot(WTPLOTS.u);
subplot(4,1, 2)
hold on,
% line([0 length(WTPLOTS.u)], [10 10]),
plot(WTPLOTS.tankLevel1),
hold off
subplot(4,1, 3)
hold on,
% line([0 length(WTPLOTS.u)], [10 10]),
plot(WTPLOTS.tankLevel2);
hold off
subplot(4,1, 4)
plot(WTPLOTS.integrals);
legend('wt1','wt2','wt3','wt4','wt5','wt6','wt7','wt8','wt9','wt10');



function plotsMED()
global WTCONTROL;
global WTAPP;

 m_file = sprintf('%s/cdc_%s.mat',WTAPP.OUTPUTS_PATH , ...
             datestr(date, 'yymmddHHMM'));
 save(m_file, 'WTCONTROL');
        
%save cdc.mat;
load( m_file)
%load cdc_1103220000_final.mat;
cost = WTCONTROL.cost;
data = WTCONTROL.data;
init = WTCONTROL.init;
ErrorLog = WTCONTROL.ErrorLog;
sysd = WTCONTROL.sysd;
Xlog = WTCONTROL.Xlog;
Ylog = WTCONTROL.Ylog;
Ulog = WTCONTROL.Ulog;
Hlog = WTCONTROL.Hlog;
Wlog = WTCONTROL.Wlog;
Vlog = WTCONTROL.Vlog;
Qlog = WTCONTROL.Qlog;
Plog = WTCONTROL.Plog;
P = WTCONTROL.P;
mlog = WTCONTROL.mlog;
m = WTCONTROL.m;
k = WTCONTROL.k;
LAMBDAlog = WTCONTROL.LAMBDAlog;
sys_trans = WTCONTROL.sys_trans;
sim_time = size(Xlog,2);
N = data.nSensors;

sim_time = data.simTIME;
Xref = 10;

times = data.h:data.h:data.h*sim_time; 

%offset_bottom = kron(data.offset(2:2:data.nSensors),ones(1,length(times)));

%close all
% Plot traces and envelope
figure;
range = sqrt(cost.c{1}); % assumes all are controlled to the same range
%plot(times,Ylog(2:2:data.nSensors,:)-offset_bottom);
plot(times,Ylog(2:2:data.nSensors,:));
hold on
plot([times(1),times(sim_time)],[Xref-range, Xref-range],'--k')
plot([times(1),times(sim_time)],[Xref+range, Xref+range],'--k')
%plot(times,eta(1,:),'.k')
%plot(times,eta(2,:),'.k')
axis([data.h sim_time*data.h 0 25]);
xlabel('seconds')
ylabel('centimeters')
title('Bottom tank water level vs. time')
legend('wt1','wt2','wt3','wt4','wt5','wt6','wt7','wt8');
% 
% figure(1)
% 
% legend('wt1','wt2','wt3','wt4','wt5','wt6','wt7','wt8');



figure;
range = cost.c{1}; % assumes all are controlled to the same range
plot(times,Ylog(1:2:data.nSensors,:))
hold on
plot([data.h,sim_time*data.h],[Xref-range, Xref-range],'--k')
plot([data.h,sim_time*data.h],[Xref+range, Xref+range],'--k')
axis([data.h sim_time*data.h 0 25]);
xlabel('seconds')
ylabel('centimeters')
title('Top tank water level vs. time')

figure;
plot(times,Ulog(1:data.nActuators,1:sim_time))
axis([data.h sim_time*data.h 0 25]);
xlabel('seconds')
ylabel('volts')
title('Controller input vs. time')

figure;
bar(sum(Qlog,2)/sim_time)
axis([0,21,0,1])
xlabel('sensor number')
ylabel('probability of selection')
title('Probability of selection vs. sensor')


vals = zeros(size(Qlog,1)*size(Qlog,2),2);
k = 0;
for i = 1:size(Qlog,1)
    for j = 1:size(Qlog,2)
        if (Qlog(i,j) == 1)
            k = k + 1;
            vals(k,1) = i;
            vals(k,2) = j;
        end
    end
end

figure;
%plot(vec(kron(times,ones(1,20))),vec(Qlog),'k.')
plot(vals(1:k,2),vals(1:k,1),'k.')
axis([0, length(times),0,21])
xlabel('seconds')
ylabel('sensor number')
title('Sensor schedule vs. time')

figure;
plot(times,ErrorLog/25)
xlabel('seconds')
ylabel('Error percentage')
title('Error percentage vs. time')




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


% --- Executes on button press in externalPlots.
function externalPlots_Callback(hObject, eventdata, handles)
% hObject    handle to externalPlots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
plotExternalFigures();


% --- Executes on button press in calibrationResults.
function calibrationResults_Callback(hObject, eventdata, handles)
waterTankApp('calibration');

