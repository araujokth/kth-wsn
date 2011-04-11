% function data = read_data2(con)

% length_message = pnet(con,'read',1,'uint8');
% 
% message(1,:) = pnet(con,'read',length_message,'uint8');
% 
% data = message(length_message-message(6)+1:end);

function [updated, out] = read_data(con)

length_message = pnet(con,'read',1,'uint8');

message = pnet(con,'read',length_message,'uint8');

data = message(1:end);
data = cast(data,'uint8');

if size(data,2)>0
    updated = 1;
    out = data;
else
    updated = 0;
    out = 0;
end

