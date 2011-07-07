function disconnect(varargin)
%DISCONNECT
%
%this function disconnects the current matlab environment from a phoenix
%source.  If the connection does not exist, nothing is done.  The
%parameters must match the exact name that was used to create the
%phoenix source, so serial@COM1 will not terminate serial@COM1:mica2dot.
%
%usage: disconnect('network@localhost:9000')
%       disconnect('serial@COM1')
%       disconnect('serial@COM1:mica2dot')
%       disconnect('network@localhost:9000', 'network@c62b27d:10002', 'serial@COM1:mica2dot', 'serial@COM2',...)

global COMM

if isempty(COMM)
  return
end

disp('WARNING: Due to an error in the 1.3.1 java.net.socket.close() implementation, the disconnect function is not guaranteed to work.')

if nargin > 1
  for i=1:nargin
    disconnect(varargin{i})
  end
  return
end

if nargin==0 usage; return; end
connectionName=varargin{1};
if ~ischar(connectionName) usage; return; end

mIFtoKill=strcmpi(COMM.connectionName, connectionName); %shoul be only one
if any(mIFtoKill)
  packetsToDeregister=strcmpi(COMM.sourceName, connectionName);
  while(any(packetsToDeregister))
    i=find(packetsToDeregister); %first, delete all the matlab message listeners from this moteIF
    stopReceiving(COMM.function{i(1)}, COMM.messageType{i(1)}, COMM.sourceName{i(1)});
    packetsToDeregister=strcmpi(COMM.sourceName, connectionName);
  end

  %then, stop and delete the moteIF itself
  phoenixSource = COMM.moteIF{mIFtoKill}.getSource;
  shutdown(phoenixSource);
  COMM.moteIF={COMM.moteIF{~mIFtoKill}};
  COMM.connectionName={COMM.connectionName{~mIFtoKill}};
end
  
    
function usage	
disp('usage: disconnect(''network@localhost:9000'')')
disp('       disconnect(''serial@COM1'')')
disp('       disconnect(''serial@COM1:mica2dot'')')
disp('       disconnect(''network@localhost:9000'', ''network@c62b27d:10002'', ''serial@COM1:mica2dot'', ''serial@COM2'',...)')
