function send_data(con,data)

%Source:

%http://www.tinyos.net/tinyos-2.x/doc/html/tep113.html
%http://www.capsule.hu/local/sensor/documentation/deciphering_tinyOS_serial_packets.pdf
%http://www.mail-archive.com/tinyos-help@millennium.berkeley.edu/msg24829.html
%http://tinyos.net/tinyos-2.x/doc/txt/tep113.txt

data_length = length(data);

pnet(con,'write',cast([data_length+8,0,255,255,255,255,data_length,0,6,data],'char'));
