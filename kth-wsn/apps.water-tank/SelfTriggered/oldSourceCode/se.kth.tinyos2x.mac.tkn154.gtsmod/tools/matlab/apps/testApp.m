function testApp(varargin)
%This function is the interface to control the matlab testApp application

%
% The following block is the standard matlab/TinyOS app.
% Functions specific to this application are below
%

if nargin>0 & ischar(varargin{1})
  % the user or timer is calling one of the functions below
  feval(varargin{1},varargin{2:end});
  
elseif nargin==0 
  usage;
end


function usage
fprintf('USAGE:\n\ttestApp(''init'')\n\ttestApp(''startP'')\n\ttestApp(''stopP'')\n\ttestApp(''disconnectP'')\n\tetc.\n')


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
    global TESTAPP

    % import all necessary java packages
    import net.tinyos.*
    import net.tinyos.message.*

    % connect to the network
    connect('sf@localhost:9002');
    %connect('serial@/dev/ttyUSB0:tmote');

    % instantiate a timer
    TESTAPP.t = timer('Name', 'Timer','TimerFcn',@timerFired,'ExecutionMode','fixedRate','Period',2);
    TESTAPP.counter = 0;
    
    % set the msg class
    TESTAPP.MsgClass = SensorsMsg;


function startP
    global TESTAPP;
    % start the timer. Just to debug
    %start(TESTAPP.t);
    % register as a listener to BlinkToRadioMsg objects
    receive(@testAppMessageReceived, TESTAPP.MsgClass);


function stopP
    global TESTAPP
    % stop the timer
    stop(TESTAPP.t)
    % unregister as a listener to  TESTAPP.MsgClass objects
    stopReceiving(@testAppMessageReceived, TESTAPP.MsgClass);
    
function disconnectP
    global COMM
    for i=1:size(COMM.connectionName)
        disconnect(COMM.connectionName{i});
    end

