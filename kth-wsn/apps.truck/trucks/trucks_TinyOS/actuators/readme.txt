TRUCK CONTROL
21 - 10 - 2011



serialforwarder:
------------------------------
It has to be programmed in the mote with nodeID = 1. Then, with the mote connected to the computer, it is necessary to open the connection with the serialforwarder in the computer. This can be donde as follows:

Windows:
1 - In cygwin type the command "motelist" to know in which port the mote is connected. In Windows it is always called "COMi", where "i" is a number.

2 - Type "sf 9003 COMi telosb". If you receive a warning message that means that you are connected with the mote and you can use the LabView programs.

Linux:
1 - Type in the terminal the command "motelist" to know in which port the mote is connected. In Linux it is alwais called "/dev/ttyUSBi", where "i" is a number.

2 - Type "sf 9003 /dev/ttyUSBi telosb". Now you can use the LabView programs.
