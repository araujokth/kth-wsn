@author Altamash Ahmed <khan6@kth.se>
@author Anser Ahmed <anserahmed87@gmail.com>
@author: Ziyang Li <lziyang@kth.se>
@title: README file for the Water Tank app on CSMA-CTP communication protocol
@date: 2013/01

DESCRIPTION
--------------------------------------------------
This application is for the Water Tank experiments, it uses the original CSMA-CTP as its routing layer protocol.

It adopts the two-path topology:
      -- [8] -- [6] -- [4] -- [2] --
    --				    --
[10]				      [1]
    --				    --
      -- [9] -- [7] -- [5] -- [3] --
  Mote [10] is the sensor which acquisites the water levels in both tanks, and mote [1] is the sink which will actuate based on the sensor values it receives through wireless communication. Mote [8], [6], [4], [2] and mote [9], [7], [5], [3] form two routes, either could forwards the data from the sensor to the sink.


DEPLOYMENT AND INSTALLATION
--------------------------------------------------
  To form the two-topology network, we need 10 WSN motes in total. It is better to arrange the motes according to their respective position in the network topology, as shown in the diagram in "DESCRIPTION", so that the status (link-establishing, re-routing, etc.) of the network could be visually clear. All motes could connected to a USB hub with power supply. When running the experiment, the relay motes could also be powered by two AA batteries instead.

  Use the command "motelist" in the terminal to get the serial number of each mote, then write these serial numbers in the bash script ".installMotes0". Run this script to install codes on corresponding motes.

  To start running the experiment, press the USER_BUTTON of the sensor mote and the sink mote. And to kill a link, just press the RESET_BUTTON of a relay mote and hold it for a few seconds. If one wants to evaluate the longest re-routing time, just kill the relay motes that's closest to the sink, i.e. mote 2 or 3.

  To restart the experiment, reset the sink mote first, then reset other motes; otherwise, these motes will start to work again since the sink mote is still active.

  To read data from the sink mote through serial port, one could use the serial forwarder command "sf 9002 /dev/ttyUSBx 115200" with either "sflisten localhost 9002" in the terminal, or the Labview VI in the /Labview_Interface folder.

  For more information about the Water Tank experiment, please refer to the documents in the kth-wsn repository: kth-wsn-read-only/trunk/kth-wsn/app.water-tank/


