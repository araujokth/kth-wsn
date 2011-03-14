%Function con = SF_CONNECT('hostname',port)
%
%This function opens a TCP connection with the SerialForwarder. The connexion
%identifier (con) is returned.
%
%Use SF_DISCONNECT(con) for close the connexion
%
%
%See also sf_disconnect, pnet, read_message
%
%Daniel Perez Huertas, Oct2010


function con = sf_connect(hostname,port)

global CON_ID

if nargin == 0
    hostname = 'localhost';
    port = 9002;
end

con = pnet('tcpconnect',hostname,port);

if nargin == 0
    CON_ID = con;
end

if con < 0
    disp('Connexion error');
    pnet(con,'close');
    return
end

handshake_sent = cast([85,32],'char'); %85->Cookie, 32->Version

pnet(con,'write',handshake_sent);

pause(0.1);

handshake_replied = pnet(con,'read',2,'char');

if ~strcmp(handshake_sent,handshake_replied)
    disp('Warning: Handshake error')
end
    
pause(0.1);

return