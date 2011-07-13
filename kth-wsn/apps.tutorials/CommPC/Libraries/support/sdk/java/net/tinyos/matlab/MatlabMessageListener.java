// $Id: MatlabMessageListener.java,v 1.4 2004/03/07 08:39:18 kaminw Exp $

/* MatlabMessageListener.java
 *
 * "Copyright (c) 2001 and The Regents of the University 
 * of California.  All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * $\Id$
 */

/**
 * 
 *
 * @author <a href="mailto:kamin@cs.berkeley.edu">Kamin Whitehouse</a>
 */

package net.tinyos.matlab;

import net.tinyos.util.*;
import net.tinyos.message.*;
import java.util.*;
import java.lang.System.*;
import com.mathworks.jmi.*;

public class MatlabMessageListener implements MessageListener
{
    Vector listeners = null;
    MatlabControl matlab;
    String connectionName;
    boolean isOn;

    public MatlabMessageListener()
    {
	connectionName=null;
	isOn=true;
	try{
	    matlab=new MatlabControl();
	    this.listeners=new Vector();
	}
	catch(Exception e){}
    }

    public void messageReceived(int to, Message m){
	if(isOn){
	    if(connectionName==null){
		Object[] args = new Object[2];
		args[0]= new Integer(to);
		args[1]= m;
		for(int index=0;index<listeners.size();index++){
		    matlab.feval((String)listeners.elementAt(index), args);
		}
	    }
	    else{
		Object[] args = new Object[3];
		args[0]= new Integer(to);
		args[1]= m;
		args[2]= connectionName;
		for(int index=0;index<listeners.size();index++){
		    matlab.feval((String)listeners.elementAt(index), args);
		}
	    }
	}
    }

    public void registerMessageListener (String Listener ) 
    {
	listeners.addElement(Listener);
    }

    public void deregisterMessageListener (String Listener)
    {
	listeners.removeElement(Listener);
    }
    
    public void setConnectionName(String ConnectionName)
    {
	connectionName = ConnectionName;
    }
    
    public void start(){
	isOn=true;
    }

    public void stop(){
	isOn=false;
    }
}





