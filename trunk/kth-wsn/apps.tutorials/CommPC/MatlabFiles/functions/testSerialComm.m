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
 
%
% This applications includes all the functions that we need to comunicate
% between the PC - mote and mote - PC.
%
% Not all the functions are implemented, it only includes the basic
% functionalities. This is just the squeleton for some of them.
% 
% @param varargin   contains the function that we need to execute
%                   {init, startP, stopP, compileP, disconnectP, 
%                    changeController, showPlots}
function testSerialComm(varargin)

%This function is the interface to control the matlab TestSerial application

%
% The following block is the standard matlab/TinyOS app.
% Functions specific to this application are below
%
if nargin>0 & ischar(varargin{1})
    % the user or timer is calling one of the functions below
    feval(varargin{1},varargin{2:end});
else nargin==0
    usage;
end


function usage
fprintf('USAGE:\n\twaterTanksSelfTriggered(''init'')\n\twaterTanksSelfTriggered(''startP'')\n\twaterTanksSelfTriggered(''stopP'')\n\twaterTanksSelfTriggered(''disconnectP'')\n\tetc.\n')

% To COMPILE all the motes. We use bash or php scripts to program all the
% motes at the same time. (It is not mandatory)
function compile(varargin)
disp('--- Script to program the motes and generate the motelist ---');
% Here you could call your script to program all the motes automatically
% unix('moteSelfTriggered');
disp('-------------------------------------------------------------');


% To INITialize all the variables that we need to configure the
% communications, controller, or any kind of data.
function init(varargin)
% create a global structure to hold persistent state for this application
global TestSerial

% import all necessary java packages
import net.tinyos.*
import net.tinyos.message.*

%% SerialForwarder CONSTANTS
TestSerial.usbBasePort = 0;
TestSerial.connectionPortInit = 9002;   % Port where the BaseStation is connected
                                        % It is really interesting to use
                                        % an automatic way to detect the
                                        % port where the mote is connected
                                        % @see PHP scripts
TestSerial.connectionName = 'sf@localhost:';      % Connection name for the SerialForwarder
TestSerial.msgClass = TestSerialCommMsg;          % JavaClass for the packet we want 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Controller CONSTANTS
% Some examples, in case we need to change the controller
%TestSerial.PERIODIC_CONTROLLER = 1;
%TestSerial.APERIODIC_CONTROLLER = 2;
%TestSerial.controller = TestSerial.PERIODIC_CONTROLLER;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Tutorial CONSTANTS
% Some examples, in case we need to change the controller
TestSerial.TEST_READ_COMM = 1;
TestSerial.TEST_WRITE_COMM = 2;
TestSerial.TEST_SERIAL_COMM = 3;
TestSerial.TEST_HIL = 4;
TestSerial.tutorialApp = TestSerial.TEST_READ_COMM;

TestSerial.message = TestSerial.msgClass;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Logger CONSTANTS
% Set the variables that we want to log
TestSerial.logger.serialOutputFileName = '';
TestSerial.logger.nSamples = 1;
TestSerial.logger.counterReceived = [];
TestSerial.logger.counterSent = [];

% Here we should write the variables that we need to plot the resultss
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Do we need to have something periodic?
% We create a timer with a fixedRate of 6 seconds
% Be aware that when you use start(TestSerial.t) the callback function is
% called as well. 
TestSerial.t = timer('Name', 'Timer', ...
                 'TimerFcn',@timerFired, ...
                 'ExecutionMode','fixedRate', ...
                 'Period',1);
             
% It is a good solution to create an state var
TestSerial.INITIALIZATION = 0;
TestSerial.RUNNING = 1;
TestSerial.state = TestSerial.INITIALIZATION;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% To START the connections with the SerialForwarder and start the listener
% for the selecting message type
function startP(option)
global TestSerial
global COMM

% Set the tutorial we want to run
TestSerial.tutorialApp = option;

if TestSerial.tutorialApp == TestSerial.TEST_HIL
    % For the Hardware in Loop we need two motes, one receiving data,
    % the computer simulating the model of the system, and another mote
    % sending the data
    
    %% Connections for reception/send with one mote
    % Execute the SerialForwarder in the specified port
    sfStr = sprintf('sf %d /dev/ttyUSB%d 115200 &', ...
        TestSerial.connectionPortInit, TestSerial.usbBasePort);
    unix(sfStr);
    
    connectionName =  sprintf('%s%d', TestSerial.connectionName, ...
        TestSerial.connectionPortInit);
    
    % Build the connections array
    connect(connectionName);
    
    % Set the listener for the received messages with the class (TestSerial.msgClass)
    % in the connection TestSerial.connectionName
    % The function readFromBaseStation is fired on the reception of each
    % message
    receive(@readFromBaseStation, TestSerial.msgClass, connectionName);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Connections for send data with the second mote
    % Execute the SerialForwarder in the specified port
    sfStr = sprintf('sf %d /dev/ttyUSB%d 115200 &', ...
        TestSerial.connectionPortInit + 1, TestSerial.usbBasePort + 1);
    unix(sfStr);
    
    connectionName =  sprintf('%s%d', TestSerial.connectionName, ...
        TestSerial.connectionPortInit + 1);
    
    % Build the connections array
    connect(connectionName);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sendPacket( 0, TestSerial.message.get_counter()+1, 1 );

else

    %% Connections for reception/send with one mote
    % Execute the SerialForwarder in the specified port
    sfStr = sprintf('sf %d /dev/ttyUSB%d 115200 &', ...
        TestSerial.connectionPortInit, TestSerial.usbBasePort);
    unix(sfStr);

    connectionName =  sprintf('%s%d', TestSerial.connectionName, ...
        TestSerial.connectionPortInit);

    % Build the connections array
    connect(connectionName);

    if ~(TestSerial.tutorialApp == TestSerial.TEST_WRITE_COMM)
        % Set the listener for the received messages with the class (TestSerial.msgClass)
        % in the connection TestSerial.connectionName
        % The function readFromBaseStation is fired on the reception of each
        % message
        receive(@readFromBaseStation, TestSerial.msgClass, connectionName);
    end
    
    if ~(TestSerial.tutorialApp == TestSerial.TEST_READ_COMM)
        % Be aware that when you call start(TestSerial.t) the callback function is
        % called as well.
        start(TestSerial.t);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% If we want to check for how long the app is running
tic


% To STOP the listenets
function stopP
global TestSerial
if isempty(TestSerial)
    %% Connections for reception/send with one mote
    connectionName =  sprintf('%s%d', TestSerial.connectionName, ...
            TestSerial.connectionPortInit);

    % Stop the listener, we shall not receive any more functions calls
    stopReceiving(@readFromBaseStation, TestSerial.msgClass, connectionName);

    % stop the timer
    stop(TestSerial.t);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% To DISCONNECT the connections with the SerialForwarder
function disconnectP
global COMM
% Kill the SerialForwarder (all the SerialForwarders running stop)
unix('killall sf');
% We disconnect all the connections
for i=1:size(COMM.connectionName,2)
    disconnect(COMM.connectionName{1});
end




%%
% This functions are not used in this template but could be useful
% to keep the structure

% To change the controller based on the Drop-down menu from the GUI
function changeController(option)

% Save the TestSerial structure to plots results latel on
function saveInformation
global TestSerial
c = clock;
m_file = sprintf('mat/data_%s_%02.f%02.f%02.f.mat', ...
    datestr(date, 'yymmdd'), c(4), c(5), c(6) );

save(m_file, 'TestSerial');

function showPlots
d = dir('mat/data_*');
if isempty(d)
    return;
end
% We open the oldest file
file = d(end).name;

% Function to plot the results
plotGlobal(['mat/' file]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




