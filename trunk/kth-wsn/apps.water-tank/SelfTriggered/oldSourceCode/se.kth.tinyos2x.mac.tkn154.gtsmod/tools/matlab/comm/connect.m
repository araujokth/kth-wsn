function moteIF = connect(varargin)
%CONNECT
%
%this function connects the current matlab environment to a phoenix
%source.  The return value is the moteIF that it is using to make
%the connection. This function is idempotent, so if you call it
%multiple times on the same connectionName, it will not actually
%try to make the connection more than once.  The parameters can be
%any legal parameters that can be used to build a phoenix source.
%
%usage: connect('network@localhost:9000')
%       connect('serial@COM1')
%       connect('serial@COM1:mica2dot')
%       connect('network@localhost:9000', 'network@c62b27d:10002', 'serial@COM1:mica2dot', 'serial@COM2',...)

global COMM
global DEBUG

if isempty(COMM)
    defineComm
end

if nargin > 1
  for i=1:nargin
    moteIF{i}=connect(varargin{i});
  end
  return
end

if nargin==0 usage; return; end
connectionName=varargin{1};
if ~ischar(connectionName) usage; return; end

TF=strcmp(COMM.connectionName, connectionName);
if any(TF)
  moteIF={COMM.moteIF{TF}};
  return
end

try
    messenger = net.tinyos.matlab.MatlabMessenger;
    messenger.displayMessages(DEBUG);
    phoenixSource = net.tinyos.packet.BuildSource.makePhoenix(connectionName,messenger);
    phoenixSource.setResurrection; %make it not kill matlab on errors, but try to restart
    moteIF = net.tinyos.message.MoteIF(messenger); 
catch
    moteIF=[];
    disp(['Could not connect: ' connectionName])
    return
end
COMM.connectionName{end+1}=connectionName;
COMM.moteIF{end+1}=moteIF;
for i = 1:length(COMM.globalFunction) %register all global listeners
  receive( COMM.globalFunction{i}, BaseStation,  connectionName);
end

    
function usage	
disp('usage: connect(''network@localhost:9000'')')
disp('       connect(''serial@COM1'')')
disp('       connect(''serial@COM1:mica2dot'')')
disp('       connect(''network@localhost:9000'', ''network@c62b27d:10002'', ''serial@COM1:mica2dot'', ''serial@COM2'',...)')
