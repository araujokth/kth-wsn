/*
 * Copyright (c) 2009, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Miklos Maroti
 */

interface BareSend
{
	/**
	 * Starts the transmission of the given message. This command must not
	 * be called while another send is in progress (so one must wait for the
	 * sendDone event). Returns EBUSY if a reception is in progress or for
	 * some other reason the request cannot be temporarily satisfied (e.g.
	 * the SPI bus access could not be acquired). In this case the send 
	 * command could be retried from a task. Returns SUCCESS if the 
	 * transmission could be started. In this case sendDone will be fired.
	 */
	command error_t send(message_t* msg);
	
	/**
	 * Signals the completion of the send command, exactly once for each 
	 * successfull send command. If the returned error code is SUCCESS, then 
	 * the message was sent (may not have been acknowledged), otherwise 
	 * the message was not transmitted over the air.
	 */
	event void sendDone(message_t* msg, error_t error);

	/**
	 * Cancel a requested transmission. Returns SUCCESS if the 
	 * transmission was cancelled properly (not sent in its
	 * entirety). Note that the component may not know
	 * if the send was successfully cancelled, if the radio is
	 * handling much of the logic; in this case, a component
	 * should be conservative and return an appropriate error code.
	 */
	command error_t cancel(message_t* msg);
}
