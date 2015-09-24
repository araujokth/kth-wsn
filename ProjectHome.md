# Introduction #

Welcome to our source code and documentation repository on Wireless Sensor and Actuator Networks!

In the Automatic Control Lab in the Electrical Engineering Department at KTH Royal Institute of Technology we have been performing extensive research in Networked Control Systems, Embedded Systems and Wireless Sensor and Actuator Networks.

We have been developing several applications using wireless sensors nodes as Telosb, Tmote Sky, CM5000 and Zolertia Z1 using mainly the TinyOS operating system. Also, we have been working in the implementation of the full IEEE 802.15.4 MAC protocol to be used in these applications. All the code developed and documentation is available here.

# Other WSN contributions #

Other code contributions can be found in the TinyOS Contribution folder for KTH. This includes the full impementation of the IEEE 802.15.4 MAC (2006),a modified version of this MAC, and the implementation of SPI drivers for telosb/sky nodes http://tinyos.cvs.sourceforge.net/viewvc/tinyos/tinyos-2.x-contrib/kth/index.html

If you came here looking for GISOO, our wireless cyber-physical system simulator, you can find it at GISOO's [google code project page](https://code.google.com/p/kth-gisoo/)

# Project structure #

In the section "Downloads" you can find all the tutorials we have available which explain you e.g. how to get started with TinyOS, how to perform communication between a wireless node and a computer and how to debug your WSN implementations. If you can't see any Downloads when you are there, click on See All Downloads so you can get access to all of the available downloads.

Section "Source" has software we have developed in our implementations at KTH and the software required to run some applications from our tutorials.

# Help and suggestions? #

Please let us know if you encounter any problems with our tutorials or if you have any suggestions.

General contact: araujo@kth.se

# The Team #

  * [José Araújo](http://people.kth.se/~araujo/) - PhD student - araujo@kth.se

  * João Pedro Alvito - Research engineer - jpfa@kth.se

  * [Karl Henrik Johansson](http://www.s3.kth.se/~kallej/) - Professor - kallej@kth.se


## Alumni ##

  * Pangun Park - PhD student - currently at ETRI, Korea

  * [Behdad Aminian](http://people.kth.se/~behdad/) - Research engineer - behdad@kth.se - currently at Ericsson AB, Sweden

  * Lin Wei - Research engineer

  * Ziyang Li - Research engineer

  * Mani Amoozadeh - Research engineer - currently PhD student at UC Davis, US

  * Anser Ahmed - Research engineer - currently at Ericsson, Sweden

  * Sara Khosravi - Research engineer

  * Altamash Khan - Research engineer - currently PhD student at LCN, KTH, Sweden

  * Navid Hasanzadeh - Research engineer - currently at CIT, Ireland

  * Aziz Khakulov - Research engineer - currently at Siemens Healthcare, Sweden

  * Alireza Ahmedi - Research engineer - currently at KTH Control AB, Sweden

  * João Faria - Research engineer - currently at TV Globo, Brazil

  * David Andreu - Research engineer

  * Aitor Hernandez - Research engineer - currently at Ericsson, Sweden

<a href='Hidden comment: 

----

= Source code =

The code in this server is divided basically by the process that they are managing.

== Inverted Pendulum ==
We have the code necessary for the motes, and the LabView interface to analize the communication performance.

== Water Tanks ==

In this case, the code is also divided by the different scenarios that we have for this process:

* Centralized Controller
* Self-Triggered Controller
* Dynamic Scheduler

== 3D Crane ==

Here, we have the code for the motes that we need to run the Event-Triggered 3D Crane. Moreover is needed other platforms (See documentation)

== Tutorials ==

The folder contains different tutorials and templates which could be used to start a certain application. It includes.

* Communication between the mote and PC (TinyOS, Matlab and LabView templates)
* Serial communication between motes

----

= Documents =

Before download the code, we recommend to take a look on the following documents. They are related to some of applications that could be found on this server, and give to the user a guidance.

(They are not available yet)

* Getting started with Tinyos at the Automatic Control

* Debugging

* Communication PC to mote

* Wireless Water Tanks. Different scenarios and controllers

* Wireless Crane. Event-Triggered Control

* Wireless Inverted Pendulum

'></a>