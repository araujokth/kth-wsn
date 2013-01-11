@author: Altamash Ahmed <khan6@kth.se>
@author: Ziyang Li <lziyang@kth.se>
@title: README file for the applications to test TDMA-CTP code
@date: 2013/01

--------------------------------------------------
DESCRIPTION
--------------------------------------------------
  This application uses the TDMA-CTP (developed by Altamash Ahmed) as the routing-layer protocol. The purpose of this application is to test the performance of the network with TDMA-CTP code. Specifically, we are interested in the delay caused by re-routing after the network topology is changed.

  Since we mainly focus on the network performance rather than the sensor values, we modify the original Water Tank application code to let the sensor motes only send out the sequence number of the packet, which increments by 1 after each transmission. Of course, the "sensor value" could be changed to carry other kinds of information.

  To run this application, we don't need any external sensor or device, but only WSN motes as "sensors", "relays" and "sink" of the network.


--------------------------------------------------
BEFORE USE
--------------------------------------------------
  Several GTS & TDMA-CTP code files include header file from the application folder, currently we specify a relative path that points to app/EasyCollection/Sink/EColl.h. So if the location of this application and/or header file is changed (e.g. SenseCollection app which collect data from the humidity sensor), the path for including the header file should be modified accordingly, it's in following files:
> tos/lib/net/ctp_tdma/CtpForwardingEngineP.nc
> tos/lib/net/le_tdma/LinkEstimatorP.nc
> tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154-sink/BeaconTransmitP.nc


--------------------------------------------------
CONFIGURATION
--------------------------------------------------
I. Network Topology
  The current code establishes a ring-topology network, which is illustrated by the following diagram:
	     --	[6] --
          --	 |     --
       --	 |	  --
     --		[2]	    --
   --		 |	      --
  --		 |	       --
[9]-----[5]-----[1]-----[3]-----[7]
  --		 |	       --
   --		 |	      --
     --		[4]	    --
       --	 |	  --
          --	 |     --
	     --	[8] --

a) Sensor Motes: the four motes on the outer ring [6], [7], [8], [9] are the sensor motes. Apart from sending out own data value (seqno), each of these four motes could also be responsible for forwarding data from neighbor sensor motes, if it's selected as the parent mote.

b) Relay Motes: the four motes on the inner ring [2], [3], [4], [5] are the relay motes, which are supposed to forward the data from the sensor mote connected to each of them, i.e. [6], [7], [8], [9], respectively.

(c) Sink Mote: mote [1] acts as the sink of the network. It is responsible for establishing the network at the beginning, and after the network is established, it starts to collect sensor data through relay motes [2], [3], [4], [5].

  To change the network topology, one needs to modify code pieces in two files:
1) tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154-sink/TKN154ActiveMessageP.nc
  In the "setDefaultGtsDescriptor" function, allocate a time slot for each mote (sink not included). Note that sensor motes should be allocated "early" time slots (small slot numbers), while relay motes should be given "late" time slots (large slot numbers). And it's better to leave the latest time slot unused (e.g. allocate it to a unused mote id). Besides, time slots should be allocated continuously, i.e. do not skip slot, otherwise the network may not run in the expected way.

2) tos/lib/net/ctp_tdma/CtpRoutingEngineP.nc
  In the "BeaconReceive.receive" function, specify the connections between motes according to the network topology. Basically, if the beacon a mote receives is not from its neighbor (by neighbor, we mean the mote on its route to the sink), we just ignore it; otherwise, we update the neighbor table of this mote.
  In the ring topology, relay motes only have one neighbor: sink mote, while sensor motes have three neighbors: one relay mote and two other sensor motes next to it.

******************
TWO-PATH TOPOLOGY: 
      -- [8] -- [6] -- [4] -- [2] --
    --				    --
[10]				      [1]
    --				    --
      -- [9] -- [7] -- [5] -- [3] --
  In this simple topology, mote [10] and mote [1] is the sensor and sink respectively. Mote [8], [6], [4], [2] and mote [9], [7], [5], [3] form two routes, either could forwards the data from the sensor to the sink.
  To establish a network with this topology:
    * replace file 1) with TKN154ActiveMessageP_TWO_PATH_TOPOLOGY.nc
    * replace file 2) with CtpRoutingEngineP_TWO_PATH_TOPOLOGY.nc
    * replace file tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154/TKN154ActiveMessageP.nc with TKN154ActiveMessageP_NO_BREAKING.nc
which are in the same folder as the original files.
******************

II. Radio and MAC parameters
  Main radio and MAC-layer parameters, such as radio channel, transmission power, beacon order and superframe order, are configured in the following file:
> tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154-sink/TKN154ActiveMessageP.nc

III. Link Breaking
  In order to efficiently test the time it takes sensor motes to re-route, we kick each relay mote out of the network at a unique time. For the ring topology, we break mote [5], [4] and [3] sequentially, with a 30-second interval between two breakings.

  Since each mote use a independent clock source and hence has a clock time that's different from that of other motes, we need to use a common event to indicate a absolute timepoint. In the current code, the reception of the first MAC beacon is chosen as this common event, because this event occurs at almost the same time for all the originator and relay motes in the network. Basically, it works in this way: mote [5], [4] and [3] are broken (actually switched to another radio channel) 60 sec, 90 sec and 120 sec respectively after they receive the first MAC beacon.

  This link-breaking scenario is implemented in three functions: "MLME_BEACON_NOTIFY.indication()", "TimerPhase()" and "reboot()", in the file:
> tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154/TKN154ActiveMessageP.nc
  And this scenario could be disabled by simply commenting out the command for starting the timer: "call TimerPhase.startOneShot(WaitTime);"
 

--------------------------------------------------
DEPLOYMENT & INSTALLATION
--------------------------------------------------
  To form the ring-topology network, we need nine WSN motes in total. It is better to arrange the motes according to their respective position in the network topology, as shown in the diagram ("Network Topology" section), so that the status (link-establishing, re-routing, etc.) of the network could be visually clear. All motes are connected to a USB hub with power supply. And for installation and testing purpose, the USB hub could be connected to a host computer.

  The installation of code on a mote requires the USB port number to which the mote is connected, but this port number may change every time the hub is (re-)connected to the host computer. So before installing the code, we associate each mote id with the serial number of that mote by using the command "motelist" in the terminal. Write the serial number of each mote into the bash script ".installMotes1" (for the Two-Path topology, use ".installMotes0"), then run this script to install codes on corresponding motes:
$ ./.installMotes1
  Note that this script file should be set executable for direct execution:
$ sudo chmod 777 ./.installMotes1

  To start running the experiment, first press the USER_BUTTON of the sink mote, wait a few seconds to have the network established, then press the USER_BUTTON of the sensor motes preferably in this order: [9], [8], [7], [6]. If all four "shortest" paths/links work, the green led on [5], [4], [3], [2] will blink; otherwise, some sensor may have selected a sensor next to it as its "parent" which will forward the data through its own link.

  To restart the experiment, reset the sink mote first, then reset other motes; otherwise, these motes will start to work again since the sink mote is still active.


--------------------------------------------------
RESULTS
--------------------------------------------------
I. Data Logging
  In this experiment, we are mainly interested in two values: re-routing time and number of packets transmitted during this period.
  To observe the test results, one could run a java program modified from a tinyos example. The program is in app/TestSerial/ folder, and to run the program:
$ java TestSerial -comm serial@/dev/ttyUSBx:telosb

  For the re-routing time, we just calculate every elapsed time between two consequtive reception of packet which is from the same originator. For example, we save the reception time of packet seqno=5 from mote 6, and when the next packet from mote 6 comes, we subtract the saved time from the new time and get the time interval between two reception. In the terminal for sink mote, this elapsed time will be printed out with the computer time, originator, data value (seqno), and number of hops. In the terminals for relay motes, every time a relay mote is "killed", the computer time will be printed out. Therefore, we could know the time when re-routing begins and the time when re-routing is done (the next packet is received by sink).
  The print-out for sink is in the "SubReceive.receive()" function in file:
> tos/lib/net/ctp_tdma/CtpForwardingEngineP.nc
  The print-out for relay mote is in the "TimerPhase.fired()" function in file:
> tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/mac/tkn154/TKN154ActiveMessageP.nc

  Besides, we are interested in the packets (data and CTP beacon) transmitted during the re-routing period. When a packet of either kind mentioned is sent out, we print out the computer time with the type of that packet (111:CTP beacon; 222:data). So having the beginning time and end time of every re-routing, we could count the number of packets transmitted during that period.
  The print-out for sending CTP beacon is in the "sendBeaconTask()" task in file:
> tos/lib/net/ctp_tdma/CtpRoutingEngineP.nc
  The print-out for sending data packet is in the "SubSend.sendDone()" function in file:
> tos/lib/net/ctp_tdma/CtpForwardingEngineP.nc

  Everytime the test is finished, we save the print-outs from terminals to text files. The information of the packets received by Sink is saved into file "Sink.txt" and the timestamps of the packets transmitted by originator/relay motes are saved in file "Mote*_S/R.txt". Additionally, the start and end time of three re-routings are written into file "Time_reRouting.txt". Results from our tests, together with Matlab scripts, could be found in the "Results" folder.

II. Results Interpretation
  Since the file "Sink.txt" contains the information of packets from all four originators, we run a small program "data_separation.m" to separate the data streams and write each stream into a file "pkts_from_Mote*.txt".
  Then we run another program "packets_count.m" to count the number of packets transmitted by each mote during the re-routing periods.
  More information about the use of data log and Matlab scripts is described in the README file in "Results" folder.


--------------------------------------------------
INTERFERENCE MOTE
--------------------------------------------------
  The purpose of a interference mote is to test the performance of the network when there is noise signal blocking the CTP beacon (control information). We want to know whether the re-routing time will increase or not with the presence of interference.
  Our current approach is to use another mote to continuously transmit carrier signal during the entire CAP period. This mote only have TKN154 MAC code, no routing-layer protocol, because it just need the MAC beacon to know the start and end of CAP period. 

I. Modification of the code for CC2420 chip
  In order to directly transmit carrier signal, we need to make a few modification to the codes for CC2420 chip.
> add DACTST interface and wire it to CC2420SpiWireC register in:
  tos/chips/cc2420/CC2420SpiC.nc
> connect DACTST interface of CC2420TransmitP to that of CC2420SpiC in:
  tos/chips/cc2420_tkn154/CC2420ControlTransmitC.nc
> add DACTST interface and CarrierTx functions in:
  tos/chips/cc2420_tkn154/CC2420TransmitP.nc
> create the interface file for CarrierTx functions:
  tinyos-2.x-contrib/kth/tkn154-gts/tos/lib/interfaces/CarrierTx.nc
  # Note that this file could also be put in any other "interfaces" folder that is included in the Makefile.include files.

II. Use of the Interference Mote
  Install the code on an isolated mote, and give it a random mote id that's not in the network. To turn on the interference, just press the USER BUTTON; and to turn off the interference, just press RESET.

