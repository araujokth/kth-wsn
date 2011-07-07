function stopReceiving(functionName, message, varargin)
%STOPRECEIVING
%This function deregisters a function for receiving incoming messages of
%a message type through certain PC ports.  If no ports are specified, all existing ports are used.
%
%Usage: 
%       stopReceiving(function, message);
%       stopReceiving(function, message, connectionName);
%       stopReceiving(function, message, connectionName, connectionName, ...);
%
% function: the string or function handle of a function, eg. 'myfunction'
% message: a subclass of net.tinyos.message.Message
% port: a phoenix name, eg. 'network@localhost:9000' or a serial port, eg. 'serial@COM1'

global COMM

if nargin<3 %stop listening globally, and remove from all existing connections
  TF = strcmpi(COMM.globalFunction, functionName) & strcmpi(COMM.globalMessageName, message.getClass.toString.toCharArray');
  if any(TF) %if this is a global receive
    COMM.globalFunction=COMM.globalFunction{~TF};
    COMM.globalMessageType=COMM.globalMessageType{~TF};
    COMM.globalMessageName=COMM.globalMessageName{~TF};
  end
  stopReceiving(functionName, message, COMM.connectionName{:});
  return
end

if nargin > 3  %there are more than one connection; treat each one individually
  for i=1:length(varagin)
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

if ~ischar(connectionName) | ~isjava(message) usage; return; end
  
TF = strcmpi(COMM.function, functionName) & strcmpi(COMM.messageName, message.getClass.toString.toCharArray') & strcmpi(COMM.sourceName,connectionName);
for i=find(TF)
    COMM.sourceMoteIF{i}.deregisterListener(message,COMM.messageListener{i});
end
COMM.sourceMoteIF = {COMM.sourceMoteIF{~TF}};
COMM.sourceName = {COMM.sourceName{~TF}};
COMM.function = {COMM.function{~TF}};
COMM.messageType = {COMM.messageType{~TF}};
COMM.messageName = {COMM.messageName{~TF}};
COMM.messageListener = {COMM.messageListener{~TF}};


function usage
disp('Usage: ')
disp('       stopReceiving(function, message);')
disp('       stopReceiving(function, message, connectionName);')
disp('       stopReceiving(function, message, connectionName, connectionName, ...);')
disp(' ')
disp(' function: the string or function handle of a function, eg. ''myfunction''')
disp(' message: a subclass of net.tinyos.message.Message')
disp(' port: a phoenix name, eg. ''network@localhost:9000'' or a serial port, eg. ''serial@COM1''')
