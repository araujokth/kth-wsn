$Id: README.txt,v 1.4 2006-12-12 18:22:52 vlahan Exp $

README for Printf

Author/Contact:

  tinyos-help@millennium.berkeley.edu

Description:

   This application is used to generate random number. 
   
   * The output random number range [0  65536]; 
   * You can define seed by call Seed.init(yourseed);(default seed is nodeID)  
   
   After starting the service, calls to the standard
   c-style printf command are made to print various strings of text
   over the serial line. The output can be displayed using the
   PrintfClient, for example using the following command line:

   java net.tinyos.tools.PrintfClient -comm serial@/dev/ttyUSB0:115200

   Successful execution of the application is indicated by repeated
   output of the following string sequence:
   
   Here is a Random: 618
   Here is a Random: 34065
   Here is a Random: 16931
   ...

Tools:

  net.tinyos.tools.PrintfClient

Known bugs/limitations:

  None.
