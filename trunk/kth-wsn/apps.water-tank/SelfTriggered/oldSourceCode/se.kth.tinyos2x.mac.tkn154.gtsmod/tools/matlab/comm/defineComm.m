function defineComm(groupID, packetLength)
%DEFINECOMM
%
%usage: defineCOMM(groupID, packetLength, baud)

global COMM


if nargin>0 & ~isempty(groupID) 
    COMM.GROUP_ID=groupID;
elseif ~isfield(COMM,'GROUP_ID') 
    COMM.groupID=hex2dec('22'); 
end

if nargin>1 & ~isempty(packetLength) 
    COMM.packetLength=packetLength;
else
    COMM.packetLength=29;
end

disp(['groupID = ' dec2hex(COMM.GROUP_ID)])

%in this data structure, each moteIF is associated with a
%connectionName.  moteIF holds a complete list of all moteIFs
COMM.connectionName={};
COMM.moteIF={};

%each time a function or message type is added to a moteIF, it is
%stored here, with the messageListener that was used to register it
%with each moteIF and with the moteIF that it was registered with.
%Therefore, each function/mesage pair will appear once for each
%moteIF that it is registered with.
COMM.function={};
COMM.messageType={};
COMM.messageName={};
COMM.messageListener={};
COMM.sourceMoteIF={};
COMM.sourceName={};

%these are the message listeners of all connections, that should always
%be added to new connections as they are opened
COMM.globalFunction={};
COMM.globalMessageType={};
COMM.globalMessageName={};
