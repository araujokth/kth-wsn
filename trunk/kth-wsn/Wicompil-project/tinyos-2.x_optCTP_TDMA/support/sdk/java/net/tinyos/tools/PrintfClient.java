/*
 * Copyright (c) 2006 Washington University in St. Louis.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Kevin Klues (klueska@cs.wustl.edu)
 * @version $Revision: 1.3 $
 * @date $Date: 2010-06-29 22:07:42 $
 */

package net.tinyos.tools;

import net.tinyos.message.*;
import net.tinyos.packet.*;
import net.tinyos.util.*;

public class PrintfClient implements MessageListener {

  private MoteIF moteIF;
  static String zz="hhhh";
  public PrintfClient(MoteIF moteIF) {
    this.moteIF = moteIF;
    this.moteIF.registerListener(new PrintfMsg(), this);
  }

  public  void messageReceived(int to, Message message) {
    synchronized(zz){
    PrintfMsg msg = (PrintfMsg)message;
    long time = System.nanoTime();
    //System.out.print(t+":");

     for(int i=0; i<PrintfMsg.totalSize_buffer(); i++) {
       char nextChar = (char)(msg.getElement_buffer(i));
       if(nextChar != 0){
	 if(nextChar == 'T') { System.out.print(time)/1000000;	}
         else { System.out.print(nextChar); }
       }
     }

    }
  }
  
  private static void usage() {
    System.err.println("usage: PrintfClient [-comm <source>]");
  }
  
  public static void main(String[] args) throws Exception {
   String[]  source = {"serial@/dev/ttyUSB0:telosb" , "serial@/dev/ttyUSB1:telosb" , "serial@/dev/ttyUSB2:telosb" ,
                       "serial@/dev/ttyUSB3:telosb" , "serial@/dev/ttyUSB4:telosb" , "serial@/dev/ttyUSB5:telosb" ,"serial@/dev/ttyUSB6:telosb" , "serial@/dev/ttyUSB7:telosb","serial@/dev/ttyUSB8:telosb","serial@/dev/ttyUSB9:telosb","serial@/dev/ttyUSB10:telosb","serial@/dev/ttyUSB11:telosb","serial@/dev/ttyUSB12:telosb","serial@/dev/ttyUSB13:telosb","serial@/dev/ttyUSB14:telosb","serial@/dev/ttyUSB15:telosb","serial@/dev/ttyUSB16:telosb","serial@/dev/ttyUSB17:telosb","serial@/dev/ttyUSB18:telosb","serial@/dev/ttyUSB19:telosb","serial@/dev/ttyUSB20:telosb"}; 
   //String source = null;
   // if (args.length == 2) {
   //   if (!args[0].equals("-comm")) {
   //	       usage();
   //	       System.exit(1);
   //   }
   //   source = args[1];
   // }
    //String x = "serial@/dev/ttyUSB1:telosb";
    System.out.println("number " + Integer.parseInt(args[0]));
    PhoenixSource[] phoenix = new PhoenixSource[Integer.parseInt(args[0])];
    //PhoenixSource phoenix2;
    //if (source == null) {
    //     phoenix1 = BuildSource.makePhoenix(PrintStreamMessenger.err);
    //        }
    //else {
      for(int i=0; i<Integer.parseInt(args[0]) ; i++) 
      phoenix[i] = BuildSource.makePhoenix(source[Integer.parseInt(args[i+1])], PrintStreamMessenger.err);//source, PrintStreamMessenger.err);
      
    //}
    //System.out.print(phoenix1);
    MoteIF[] mif = new MoteIF[Integer.parseInt(args[0])];
    for(int i=0; i<Integer.parseInt(args[0]) ; i++)
    mif[i] = new MoteIF(phoenix[i]);
    
    PrintfClient[] client = new PrintfClient[Integer.parseInt(args[0])];
    for(int i=0; i<Integer.parseInt(args[0]) ;i++)
    client[i] = new PrintfClient(mif[i]);

  }
}
