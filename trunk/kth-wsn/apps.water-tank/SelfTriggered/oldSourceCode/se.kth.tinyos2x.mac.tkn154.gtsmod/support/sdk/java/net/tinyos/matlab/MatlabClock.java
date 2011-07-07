// $Id: MatlabClock.java,v 1.3 2005/02/21 01:41:38 kaminw Exp $

/* @(#)MatlabClock.java
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

import javax.swing.*;
import java.io.*;
import java.awt.*;
import java.awt.event.*;
import java.net.*;
import java.util.*;
import java.lang.System.*;
import com.mathworks.jmi.*;

public class MatlabClock extends JFrame implements Runnable, AdjustmentListener, ActionListener
{
    //constructors
    String          listener          = null;
    Matlab            matlab;
    volatile boolean           active = true;
    volatile boolean           quit = false;
    Thread            clockThread = null;
    int sleepTime = 1000;
    static final String START = "|>";
    static final String STOP = "||";
    static int xPos = 0;
    static int yPos = 0;
    JButton startButton = null;
    JButton stopButton = null;
    JLabel sleepLabel = null;
    JTextField sleepValue = null;
    JLabel units = null;
    JScrollBar sleepSlider = null;

    int output_mask=1;
    public static final int GUI_OFF = 0;
    public static final int GUI_ON = 1;
    public static final int OUTPUT_OFF = 0;
    public static final int OUTPUT_COMMAND = 2;
    public static final int OUTPUT_RESPONSE = 4;
    

    /*    public MatlabClock (String Name, String Listener)
    {
	MatlabClock(name, listener, 1000, 1);
    }

    public MatlabClock (String Name, String Listener, int SleepTime)
    {
	MatlabClock(name, listener, SleepTime, 1);
	}*/

    public MatlabClock (String Name, String Listener, int SleepTime, int Display )
    {
	super(Name);
	try{
	    matlab=new Matlab();//this command links Java to Matlab
	    this.listener=Listener;
	    this.sleepTime=SleepTime;
	    this.clockThread = new Thread(this, this.paramString()+ " Socket Reader");
	    clockThread.start();
	    output_mask=Display;
	    if((output_mask & GUI_ON)>0){
		createGUI();
	    }
	}
	catch(Exception e){}
    }
    
    private void createGUI()
    {
	this.setSize(250, 150);
	this.setLocation(xPos,yPos);
	startButton = new JButton(START);
	startButton.addActionListener(this);
	stopButton = new JButton(STOP);
	stopButton.addActionListener(this);
	sleepLabel = new JLabel("   Frequency   ");
	units = new JLabel(" milliseconds      ");
	sleepValue = new JTextField(String.valueOf(sleepTime));
	sleepValue.setActionCommand("textField");
	sleepValue.addActionListener(this);
	sleepValue.setColumns(8);
	sleepSlider = new JScrollBar(Scrollbar.HORIZONTAL, sleepTime, 200, 10, 4200);
	sleepSlider.addAdjustmentListener(this);

	Panel labelPanel = new Panel();
	labelPanel.add(startButton);
	labelPanel.add(stopButton);
	labelPanel.add(sleepLabel);
	labelPanel.add(sleepValue);
	labelPanel.add(units);

	Panel sleepPanel = new Panel();
	sleepPanel.setLayout(new BorderLayout());
	sleepPanel.add("North", labelPanel);
	sleepPanel.add("South", sleepSlider);

	getContentPane().add(sleepPanel);
	
	this.addWindowListener(new WindowAdapter() { 
		public void windowClosing(WindowEvent e){close();}//System.exit(0);}	    
	    });
	this.pack();
	Dimension size = this.getSize();
	yPos+=size.height;
	this.show();
    }


    public void close()
    {
	this.dispose();
	quit=true;
    }

    public void setSleepTime(int SleepTime)
    {
	sleepTime=SleepTime;
    }

    synchronized public void executeNow( )
    {
	active=true;
	notify();
    }

    synchronized public void stop( )
    {
	active=false;
    }

    synchronized public void run(){
	java.lang.String matlabOutput;
	try{
	    while(true){
		wait(sleepTime);
		while(!active) {
		    wait();
		}
		if((output_mask & OUTPUT_COMMAND)>0)
		    System.out.println(listener);
		matlabOutput = matlab.eval(listener);
		if( ((output_mask & OUTPUT_RESPONSE)>0) && (matlabOutput!=null) && (matlabOutput.trim().length() !=0))
		    System.out.println(matlabOutput);
		if(quit) return;
	    }
	}
	catch(Exception e){}
    }

    public void actionPerformed(ActionEvent e)
    {
	try{
	    Object source = e.getSource();
	    if (e.getActionCommand().equals("textField")) {
		sleepTime = Integer.parseInt(sleepValue.getText());
		sleepSlider.setValue(sleepTime);
	    }
	    else if (((JButton)source).getName()==START) {
		executeNow();
		stopButton.setEnabled(true);
	    } else if (((JButton)source).getName()==STOP) {
		stop();
		stopButton.setEnabled(false);
	    }
	}catch(Exception f){}
	return;
    }

    public void adjustmentValueChanged(AdjustmentEvent e)
    {
	Object source = e.getSource();
	sleepTime = ((JScrollBar)source).getValue();
	sleepValue.setText(String.valueOf(sleepTime));
	return;
    }
    
    protected void finalize()
    {
    }
}






