function receive(functionName, message, varargin)
%RECEIVE
%This function registers a function to receive all incoming messages of
%a message type through certain PC connections.  If no connections
%are specified, all existing connections are used with that message
%type.  The connections are specified by their phoenixSource name.
%
%Usage: 
%       receive(function, message);
%       receive(function, message, connectionName);
%       receive(function, message, connectionName, connectionName, ...);
%
% function: the string or function handle of a function, eg. 'myfunction'
% message: a subclass of net.tinyos.message.Message
% port: a phoenix name, eg. 'network@localhost:9000' or a serial port, eg. 'serial@COM1'

global COMM

if nargin<3 %this is a global receive; first add to globals, then
            %add to all existing connections
  TF = strcmpi(COMM.globalFunction, functionName) & strcmpi(COMM.globalMessageName, message.getClass.toString.toCharArray');
  if any(TF) %if this is already a global receive
    return
  else
    COMM.globalFunction{end+1}=functionName;
    COMM.globalMessageType{end+1}=message;
    COMM.globalMessageName{end+1}=message.getClass.toString.toCharArray;
  end
  if ~isempty(COMM.connectionName)
    receive(functionName, message, COMM.connectionName{:});
  end
  return
end

if nargin > 3  %there are more than one connection; treat each one individually
  for i=1:length(varargin)
    receive(functionName, message, varargin{i})
  end
  return
end

connectionName=varargin{1};

if nargin<2 usage; return; end
if ~ischar(functionName)
    functionName = functions(functionName);
    functionName = functionName.function; 
end
if ~ischar(connectionName) | ~isjava(message)
    usage;
    return;
end

moteIF=connect(connectionName); %make sure we are connected
moteIF=moteIF{1};

%check for doubles
TF = strcmpi(COMM.function, functionName) & strcmpi(COMM.messageName, message.getClass.toString.toCharArray');
moteIFs = [COMM.sourceMoteIF{TF}];
for i=1:length(moteIFs)
    if moteIFs(i)==moteIF return; end
end
    
mml = net.tinyos.matlab.MatlabMessageListener;
mml.registerMessageListener(functionName);
mml.setConnectionName(connectionName);
moteIF.registerListener(message, mml);

COMM.function{end+1}=functionName;
COMM.messageType{end+1}=message;
COMM.messageName{end+1}=message.getClass.toString.toCharArray';
COMM.sourceMoteIF{end+1}=moteIF;
COMM.sourceName{end+1}=connectionName;
COMM.messageListener{end+1}=mml;



function usage
disp('Usage: ')
disp('       receive(function, message);')
disp('       receive(function, message, connectionName);')
disp('       receive(function, message, connectionName, connectionName, ...);')
disp(' ')
disp(' function: the string or function handle of a function, eg. ''myfunction''')
disp(' message: a subclass of net.tinyos.message.Message')
disp(' port: a phoenix name, eg. ''network@localhost:9000'' or a serial port, eg. ''serial@COM1''')

