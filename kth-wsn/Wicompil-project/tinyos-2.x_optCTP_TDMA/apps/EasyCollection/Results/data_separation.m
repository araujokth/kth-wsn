
DATA = dlmread('test_5/sink.txt')

len = size(DATA, 1);
Motes = zeros(round(len/4)+10, 5, 4);
pktcnt = ones(1, 4);

for i=1:len
   index = DATA(i, 2) - 5;
   for j=1:5
       Motes(pktcnt(index), j, index) = DATA(i, j);
   end
   pktcnt(index) = pktcnt(index) + 1;
end

dlmwrite('test_5/pkts_from_Mote6.txt', Motes(:,:,1), 'delimiter', ' ', 'precision', '%d');
dlmwrite('test_5/pkts_from_Mote7.txt', Motes(:,:,2), 'delimiter', ' ', 'precision', '%d');
dlmwrite('test_5/pkts_from_Mote8.txt', Motes(:,:,3), 'delimiter', ' ', 'precision', '%d');
dlmwrite('test_5/pkts_from_Mote9.txt', Motes(:,:,4), 'delimiter', ' ', 'precision', '%d');