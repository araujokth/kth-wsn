TRUCK CONTROL
21-10-2011 KTH


The folder called "sensors" contains all the TinyOS code necessary for sensing (serialforwarder, line-following+tachometer and distance sensors). All the measurements are sent to the same serialforwarder mote, which has to be connected to the computer. All the sensor motes use the same radio channel, in this case the channel 21 (0x15).

The folder "actuators" contains the serialforwarder used for sending the control actions to all the trucks and the code for the motes that receive the control actions and apply them to the servos (motor+steering). They use the channel 22 (0x16).
