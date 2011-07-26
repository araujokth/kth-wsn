function waterTankApp(varargin)
global WTAPP
global WTPLOTS;
global WTCONTROL;


%This function is the interface to control the matlab WTAPP application

%
% The following block is the standard matlab/TinyOS app.
% Functions specific to this application are below
%
if nargin>0 & ischar(varargin{1})
    % the user or timer is calling one of the functions below
    feval(varargin{1},varargin{2:end});
elseif nargin == 3 && strcmp(varargin{3},'plotResultsWTPLOTS')
    feval(varargin{3});
elseif nargin==0
    usage;
end


function usage
fprintf('USAGE:\n\twaterTankApp(''init'')\n\twaterTankApp(''startP'')\n\twaterTankApp(''stopP'')\n\twaterTankApp(''disconnectP'')\n\tetc.\n')

%
% StdControl:
%   init
%   reinit
%   start
%   restart
%   stop
%

function init(varargin)
% create a global structure to hold persistent state for this application
global WTAPP
global WTPLOTS;
global WTCALIBRATION;

% import all necessary java packages
import net.tinyos.*
import net.tinyos.message.*

%restart all the variables
WTAPP.LIMIT = 28;
WTAPP.MATRIX_SIZE = 100000;
WTAPP.CALIBRATION = 87.3932;
WTAPP.UPDATE_INTERVAL = 1;
WTAPP.NUMBER_WT = 8;
WTAPP.DEBUG = false;
WTAPP.REF_LEVEL = 10;

WTAPP.T_SYMBOL = 1/(2*32768);
WTAPP.MISSING_VALUE = double(65534 / WTAPP.CALIBRATION);
WTAPP.BEACON_OREDER = 6;
WTAPP.SAMPLING_INTERVAL = power(2,WTAPP.BEACON_OREDER)*60*16*WTAPP.T_SYMBOL * 2;

WTAPP.constant = 0;
WTAPP.controller = 0; % 0 : Null controller
% 1 : Constant controller
% 2 : Jose's controller
% 3 : Jim's controller
WTAPP.lastSampleTime = clock;
WTAPP.tbp = [];

WTAPP.offsets = [];
WTAPP.u = zeros(WTAPP.NUMBER_WT, 1);
WTAPP.tankLevel1 = zeros(WTAPP.NUMBER_WT, 1);
WTAPP.tankLevel2 = zeros(WTAPP.NUMBER_WT, 1);
WTAPP.ACTUATOR_ID = [2 6 10 14 18];

% convert to wtID
WTAPP.ACTUATOR_WT = WTAPP.ACTUATOR_ID/2;
WTAPP.OUTPUTS_PATH = '/home/kthwsn/workspace/apps.water-tank/SlotsDistribution/MatlabSimulnk/mat';
WTPLOTS.tankLevel1 = zeros(1, WTAPP.NUMBER_WT);
WTPLOTS.tankLevel2 = zeros( 1, WTAPP.NUMBER_WT);
WTPLOTS.integrals = zeros(1, WTAPP.NUMBER_WT);
WTPLOTS.u = zeros(1, WTAPP.NUMBER_WT);
WTPLOTS.instant = 1;

% calibration
WTCALIBRATION.U_CONSTANTS= [0 2 3 4 5 6];
WTCALIBRATION.WINDOW_LENGTH = 10; % 10 smaples = 1.8*20 = 18s
WTCALIBRATION.steadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS), WTCALIBRATION.WINDOW_LENGTH); %% save the data of the steady state
WTCALIBRATION.std = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS)); %% save the data of the steady state
WTCALIBRATION.idx = [1 , 1 , 1 ]; % idxVoltage: pointer to the voltage constants
                                  % idxWindow: pointer to the inititial
                                  %            value of the new window
                                  % idxSteadyState
WTCALIBRATION.EPSILON = 0.15;
WTCALIBRATION.state = 1; % 1: tracking steadyState
                         % 2: saving steadyState Values
                         % 3: switch to new voltage
                         % 4: end of calibration


%% LQR Controller
load /home/kthwsn/workspace/apps.water-tank/SlotsDistribution/MatlabSimulnk/Kvalue_watertanks_JIM.mat
WTAPP.K_jose = double(K);

%% Jim's Controller
initializeJimsConstants();

% connect to the network
connect('sf@localhost:9002');

% instantiate a timer
%    WTAPP.t = timer('Name', 'Timer','TimerFcn',@timerFired,'ExecutionMode','fixedRate','Period',5);
%    WTAPP.counter = 0;

function startP
global WTAPP

% start the timer. Just to debug
%start(WTAPP.t);
% register as a listener to BlinkToRadioMsg objects
WTAPP.integrals = zeros(WTAPP.NUMBER_WT, 1);
receive(@sensorsMsgReceived, SensorMatrixMsg);

function stopP
global WTAPP
% stop the timer
% stop(WTAPP.t)
% unregister as a listener to WTAPPMsg objects
stopReceiving(@sensorsMsgReceived, SensorMatrixMsg);


function disconnectP
global COMM

for i=1:size(COMM.connectionName)
    disconnect(COMM.connectionName{i});
end


function changeController(varargin)
global WTAPP;
initializeJimsConstants();
WTAPP.integrals = zeros(WTAPP.NUMBER_WT, 1);
WTAPP.offsets = [];
WTAPP.controller = varargin{1};
WTAPP.constant = varargin{2};

function plotResultsWTPLOTS(varargin)
global WTPLOTS;
global WTCONTROL;
global WTAPP;

% for i=1:WTAPP.NUMBER_WT;
%        WTPLOTS.tankLevel2(i) = polyval(WTCONTROL.coeffs2{i}, WTPLOTS.tankLevel2(i) , 1);
% end

axes(WTPLOTS.uAxes);
plot(WTPLOTS.u);
axes(WTPLOTS.upperTankAxes);
plot(WTPLOTS.tankLevel1);
axes(WTPLOTS.lowerTankAxes);
plot(WTPLOTS.tankLevel2);




function initializeJimsConstants()
global WTCONTROL;
global WTAPP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% DEFINE CONSTANTS %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data.h=3; % sampling period
% data.simTIME=80; % N seconds: total sim time
% data.sizeWINDOW=1; % steps of data that we implement (DO NOT CHANGE!)
% data.sizeHORIZON=2;  % Size for the horizon window (DO NOT CHANGE!)
% data.nSensors=20;
% data.nTracking=10;
% data.nActuators=10; % used to be 5

data.h=WTAPP.SAMPLING_INTERVAL; % sampling period
data.simTIME=600; % N seconds: total sim time
data.calibrate = 1; % number of time steps to calibrate the observations
data.sizeWINDOW=1; % steps of data that we implement (DO NOT CHANGE!)
data.sizeHORIZON=3;  % Size for the horizon window (DO NOT CHANGE!)
data.nSensors=WTAPP.NUMBER_WT*2;
data.nTracking=WTAPP.NUMBER_WT;
data.nActuators=WTAPP.NUMBER_WT; % used to be 5
data.offset = 0; % to be determined automatically.


data.Rw=0.1; % process noise variance
data.Wm=0; % mean of the process noise
data.Rv=0.25; % measurement noise variance
data.lossP=1; % select loss probability
data.P=1;   % inital state variance

% SELECT THE INITIAL STATE
X=zeros(data.nSensors,1);
Xref=WTAPP.REF_LEVEL; % set point selection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xlog=zeros(size(X,1),data.simTIME);
Ylog=zeros(data.nSensors,data.simTIME);
Ulog=zeros(data.nActuators,data.simTIME);
Wlog=zeros(data.nActuators,data.simTIME);
Vlog=zeros(data.nSensors,data.simTIME);
Hlog=zeros(data.nSensors,data.simTIME);

Plog=cell(data.simTIME);
mlog=zeros(size(X,1),data.simTIME);
LAMBDAlog=zeros(data.nSensors,data.simTIME);
Qlog=zeros(data.nSensors,data.simTIME);
ErrorLog = zeros(data.simTIME,1);
k = 250;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'mat/coefs.mat'
WTCONTROL.coeffs1 = coeffs1;
WTCONTROL.coeffs2 = coeffs2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% call for A,B,C,D, E(matrix for noise, continuous), Ed(matrix for noise,
% discrete)
%[A,B,C,D,E,Ed]=matrices_comp(data.nSensors,data.nActuators,data.h);
[A,B,C,D]=matrices_comp_new_setup(data.nSensors,data.nActuators,data.h);

A=A(1:data.nSensors,1:data.nSensors);
B=B(1:data.nSensors,1:data.nActuators);
C=C(1:data.nSensors,1:data.nSensors);
D=D(1:data.nSensors,1:data.nActuators);

% sampling at h
sys=ss(A,B,C,D);
sysd=c2d(sys,data.h);
sys_out.A = sysd.a; % need to fix the error checking in transition_matrices
sys_out.B = sysd.b; % so that this step isn't needed, making the code more
sys_out.C = sysd.c; % matlab friendly
sys_out.E = sysd.b;


sys_trans.A = cell(data.sizeHORIZON);
sys_trans.B = cell(data.sizeHORIZON);
sys_trans.C = cell(data.sizeHORIZON);
sys_trans.E = cell(data.sizeHORIZON);
for j = 1:data.sizeHORIZON
    sys_trans.A{j} = sys_out.A;
    sys_trans.B{j} = sys_out.B;
    sys_trans.C{j} = sys_out.C;
    sys_trans.E{j} = sys_out.E;%init.Wm = [-1.8;-.5;-1;-.5;-.8;-.5;-.5;-.5];
end
%% Run Algorithm


%======================================================
% DEFINE THE COST FUNCTIONS
% 1) Assume we want the lower tanks to be bounded between 5 and 15 centimeters
% 2) Assume we
% Define cost constraints (assume no quadratic weighting)
cost.num = zeros(data.sizeHORIZON,1);
cost.Fu = eye(data.sizeHORIZON*(data.nActuators)) ...
    - kron(diag(ones(data.sizeHORIZON-1,1),1),-eye(data.nActuators));% ... % ...% diagonal weights
% + .5*kron(diag(ones(data.sizeHORIZON-1,1),-1),-eye(data.nActuators));% diagonal weights
%cost.Fu = 10*cost.Fu;

%cost.Fy = eye(data.sizeHORIZON*data.nSensors);
cost.Fy = 100*eye(data.nSensors);

%m = ones(size(X))*Xref;
m = X;
P = eye(size(A))*data.P;

init.W = eye(data.nActuators)*data.Rw;
%init.Wm = ones(data.nActuators,1)*data.Wm;
%init.Wm = [-1.8;-.5;-1;-.5;-.8;-.5;-.7;-.5];
init.Wm = zeros(data.nActuators,1);
%init.Wm = [-1;0;0;0;0;0;0;0];

init.V = eye(data.nSensors)*data.Rv;
init.Lambda = eye(data.nSensors)*data.lossP;
init.h = data.h;
init.m = cell(data.sizeHORIZON,1);
init.P = cell(data.sizeHORIZON,1);
for i = 1:data.sizeHORIZON
    init.m{i} = m;
    init.P{i} = P;
end
init.M = data.sizeHORIZON;

fprintf('\nInitialization done ... please wait\n');


%% Write in the global variable
WTCONTROL.cost = cost; 
WTCONTROL.data = data; 
WTCONTROL.init = init;
WTCONTROL.ErrorLog = ErrorLog;
WTCONTROL.sysd = sysd;
WTCONTROL.Xlog = Xlog;
WTCONTROL.Ylog = Ylog;
WTCONTROL.Ulog = Ulog;
WTCONTROL.Hlog = Hlog;
WTCONTROL.Wlog = Wlog;
WTCONTROL.Vlog = Vlog;
WTCONTROL.Qlog = Qlog;
WTCONTROL.Plog = Plog;
WTCONTROL.P = P;
WTCONTROL.mlog = mlog;
WTCONTROL.LAMBDAlog = LAMBDAlog;
WTCONTROL.m = m;
WTCONTROL.k = k;
WTCONTROL.sys_trans = sys_trans;


function calibration

global WTAPP;
global WTCALIBRATION;
global WTCONTROL;

files = dir(WTAPP.OUTPUTS_PATH);
nFiles = size(files,1)-2;

%%save all the steady states
steadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS), WTCALIBRATION.WINDOW_LENGTH);

for i=1:1
    clear WTCALIBRATION;

    m_file = sprintf('%s/%s',WTAPP.OUTPUTS_PATH , files(i+2).name);
    load(m_file);
    %for j=1:length(WTCALIBRATION.U_CONSTANTS)
        steadyState =  WTCALIBRATION.steadyState;
    %end
        
end
stdSteadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS));
for i=1:length(WTCALIBRATION.U_CONSTANTS)
    for j=1:WTAPP.NUMBER_WT
        %tmp = reshape(steadyState(2, j, i, : ), nFiles, 1);
        stdSteadyState(1, j, i) = var(steadyState(1, j, i, : ));
        stdSteadyState(2, j, i) = var(steadyState(2, j, i, : ));
        meanSteadtState(1, j, i) = mean(steadyState(1, j, i, : ));
        meanSteadtState(2, j, i) = mean(steadyState(2, j, i, : ));
    end
end
hf = figure;
colourOrder = ['b' 'g' 'r' 'c' 'm' 'y' 'k'  'b'];
coeffs1 = cell(WTAPP.NUMBER_WT,1);
coeffs2 = cell(WTAPP.NUMBER_WT,1);

    hold on;
    fprintf('Cubic aproximation coeffs:\n');

    A = WTCONTROL.sysd.A
    B = WTCONTROL.sysd.B
    C = WTCONTROL.sysd.C
        
    % Calculate modeled water levels
    y = zeros(2*WTAPP.NUMBER_WT,length(WTCALIBRATION.U_CONSTANTS));    
    for j = 1:length(WTCALIBRATION.U_CONSTANTS);
        u = ones(WTAPP.NUMBER_WT,1)*WTCALIBRATION.U_CONSTANTS(j);
        x = zeros(WTAPP.NUMBER_WT*2,1);
        for i = 1:100
            x = A*x + B*u;
        end
        y(:,j) = x; %C*x; % tune to state
    end
    y
    for j=1:WTAPP.NUMBER_WT
      %  errorbar(WTCALIBRATION.U_CONSTANTS, meanSteadtState(2, j,
      %  :),stdSteadyState(2, j, :) , colourOrder(j));
      %plot(WTCALIBRATION.U_CONSTANTS', reshape(meanSteadtState(2, j, :), size(meanSteadtState,3), 1)', colourOrder(j));
       
        % Find coefficients for polynomial (order = 3)
        
        y1 = y(2*j-1,:)'; 
        y2 = y(2*j,:)';
        
        fitResults1 = polyfit(reshape(meanSteadtState(1, j, :), size(meanSteadtState,3)', 1), y1, 3);
        fitResults2 = polyfit(reshape(meanSteadtState(2, j, :), size(meanSteadtState,3)', 1), y2, 3);

        coeffs1{j} = fitResults1;
        %fprintf('Upper tank [WT %d] %s \n',j, getEquationString(4, coeffs1{j}, 4));
        
        coeffs2{j} = fitResults2;
       % fprintf('Lower tank [WT %d] %s \n',j, getEquationString(4, coeffs2{j}, 4));

        % Evaluate polynomial
        yplot1 = polyval(fitResults2, reshape(meanSteadtState(2, j, :), size(meanSteadtState,3)', 1));
        plot(reshape(meanSteadtState(2, j, :), size(meanSteadtState,3)', 1), yplot1, colourOrder(j));
        axis([0 25 0 25])
    end
    legend('wt1','wt2', 'wt3', 'wt4', 'wt5', 'wt6', 'wt7', 'wt8');
    hold off;
    
    save 'mat/coefs.mat' coeffs1 coeffs2;
    
    x = [1 10.5 1 8 1 7.5 1 8.5 1 8.5 1 9 1 8.5 1 10.5 ]';
    
    km = inv(B'*B)*B'*(eye(WTAPP.NUMBER_WT*2) - A)*x;
    

function calibrationFiles

global WTAPP;
global WTCALIBRATION;


files = dir(WTAPP.OUTPUTS_PATH);
nFiles = size(files,1)-2;

%%save all the steady states
steadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS), nFiles);

for i=1:nFiles
    clear WTCALIBRATION;

    m_file = sprintf('%s/%s',WTAPP.OUTPUTS_PATH , files(i+2).name);
    load(m_file);
    clear WTCALIBATION.std
    %for j=1:length(WTCALIBRATION.U_CONSTANTS)
        steadyState(:,:,:,i) =  WTCALIBRATION.steadyState;
    %end
        
end
stdSteadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS));
for i=1:length(WTCALIBRATION.U_CONSTANTS)
    for j=1:WTAPP.NUMBER_WT
        %tmp = reshape(steadyState(2, j, i, : ), nFiles, 1);
        stdSteadyState(1, j, i) = std(steadyState(1, j, i, : ));
        stdSteadyState(2, j, i) = std(steadyState(2, j, i, : ));
        meanSteadtState(1, j, i) = mean(steadyState(1, j, i, : ));
        meanSteadtState(2, j, i) = mean(steadyState(2, j, i, : ));
    end
end
hf = figure;
colourOrder = ['b' 'g' 'r' 'c' 'm' 'y' 'k'  'b'];
        hold on;

    for j=1:WTAPP.NUMBER_WT
        errorbar(WTCALIBRATION.U_CONSTANTS, meanSteadtState(2, j, :),stdSteadyState(2, j, :) , colourOrder(j));
    end
    hold off;
    
    
    
    
%-------------------------------------------------------------------------%
function [s1] = getEquationString(fittype1, coeffs1, digits1)
%GETEQUATIONSTRING(FITTYPE1,COEFFS1,DIGITS1,AXESH1)
%  Get show equation string
%  FITTYPE1:  type of fit
%  COEFFS1:  coefficients
%  DIGITS1:  number of significant digits
%  AXESH1:  axes
if isequal(fittype1, 0)
    s1 = 'Cubic spline interpolant';
elseif isequal(fittype1, 1)
    s1 = 'Shape-preserving interpolant';
else
    op = '+-';
    format1 = ['%s %0.',num2str(digits1),'g*x^{%s} %s'];
    format2 = ['%s %0.',num2str(digits1),'g'];
    fit =  fittype1 - 1;
    s1 = sprintf('y =');
    if abs(coeffs1(1) < 0)
        s1 = [s1 ' -'];
    end
    for i = 1:fit
        sl = length(s1);
        if ~isequal(coeffs1(i),0) % if exactly zero, skip it
            s1 = sprintf(format1,s1,abs(coeffs1(i)),num2str(fit+1-i), op((coeffs1(i+1)<0)+1));
        end
        if (i==fit) && ~isequal(coeffs1(i),0)
            s1(end-5:end-2) = []; % change x^1 to x.
        end

    end
     if ~isequal(coeffs1(fit+1),0)
        sl = length(s1);
        s1 = sprintf(format2,s1,abs(coeffs1(fit+1)));
    end
end

