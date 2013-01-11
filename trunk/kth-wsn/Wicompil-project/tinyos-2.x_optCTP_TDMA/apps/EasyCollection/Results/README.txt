data_separation.m -- separate reception timestamps into four files, according to the mote id

packets_count.m -- count the number of DATA & BEACON packets during the rerouting time

counting.m -- a function for counting

> in each folder:

Mote*_*.txt -- timestamp & type(D or B) of packet transmission, 'R':Relay, 'S':Sensor

Sink.txt -- info of received packets [timestamp, mote id, value(seqno), times of being forwarded, elapsed time since last reception]

Time_reRouting.txt -- timestamps of the link-broken and link-reestablished, breaking 3 times

pkts_from_Mote*.txt -- timestamps of packet reception for individual sensor mote

packet_counting_during_rerouting.txt -- number of packets received during re-routing time
	-> 8 rows: mote 2 to 9
	-> for each row: T_broken_1, T_found_1, T_broken_2, T_found_2, T_broken_3, T_found_3