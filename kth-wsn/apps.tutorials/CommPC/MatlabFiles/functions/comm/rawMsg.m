function msg = rawMsg( am, data_bytes )
%RAWMSG  Create a TOSMsg with the given AM and data_bytes as body
%
%   msg = rawMsg( am, data_bytes );
%
%   Example: Create message to turn on motes:
%      msg = rawMsg( 249, [1 0] );
%
%   Example: Oh, you want to haul off and send it, do you? fine:
%      msg = send( 65535, rawMsg( 249, [1 0] ), 'localhost:9000' );

if nargin ~= 2
  error 'wrong number of arguments. usage: rawMsg(am,bytes)';
end

data_bytes = min( data_bytes, 255 );
data_bytes(data_bytes>127) = data_bytes(data_bytes>127) - 256;

msg = net.tinyos.message.TOSMsg;
msg.set_type( am );
msg.set_data( int8(data_bytes) );
msg.set_length( length(data_bytes) );

