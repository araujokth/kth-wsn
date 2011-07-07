function send(address, message, varargin)
%SEND
%This function sends a message to a certain address through certain PC
%ports.  If no ports are specified, all existing ports are used.
%
%Usage: 
%       send(address, message, port);
%
% address: any number between 0-65535
% message: a subclass of net.tinyos.message.Message
% port: a socket, eg. 'localhost:9000' or a serial port, eg. 'COM1'

global COMM

if nargin<3
    send(address, message, COMM.connectionName{:});
    return
end

if nargin>3
    for i=1:length(varargin)
        send(address, message, varargin{i});
    end
    return
end

connectionName=varargin{1};

if nargin<2 usage; return; end
if ~isnumeric(address) | ~ischar(connectionName) | ~isjava(message) usage; return; end

moteIF=connect(connectionName);
moteIF{1}.send(address, message);
    
      
function usage
disp('Usage: ')
disp('       send(address, message);')
disp('       send(address, message, connectionName);')
disp('       send(address, message, connectionName, connectionName, ...);')
disp(' ')
disp(' address: the value the message should be address to, eg. COMM.TOS_BCAST_ADDR')
disp(' message: a subclass of net.tinyos.message.Message')
disp(' connectionName: a phoenix name, eg. ''network@localhost:9000'' or a serial port, eg. ''serial@COM1''')

