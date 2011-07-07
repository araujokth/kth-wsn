// $Id: MatlabMessenger.java,v 1.1 2004/03/07 08:40:17 kaminw Exp $

/* MatlabMessenger.java
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
 * this class is the messenger required by the phoenixSource java tool chain to print error messages to the screen
 *
 * @author <a href="mailto:kamin@cs.berkeley.edu">Kamin Whitehouse</a>
 */

package net.tinyos.matlab;

import net.tinyos.util.*;
import net.tinyos.message.*;
import net.tinyos.message.*;
import java.util.*;
import java.lang.System.*;
import com.mathworks.jmi.*;

public class MatlabMessenger implements Messenger
{
    MatlabControl matlab;
    boolean displayMessages;

    public MatlabMessenger()
    {
	try{
	    matlab=new MatlabControl();
	    displayMessages = false;
	}
	catch(Exception e){}
    }

    public void message(String s){
	Object[] args = new Object[1];
	args[0]= s;
	if(displayMessages){
	    matlab.feval(new String("disp"), args);
	}
    }

    public void displayMessages(boolean display)
    {
	displayMessages=display;
    }
}
