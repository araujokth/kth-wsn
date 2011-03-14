/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 * 	  of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *	  materials provided with the distribution.
 *
 * 	- Neither the name of the KTH Royal Institute of Technology nor the names of its
 *    contributors may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @author Joao Faria <jfff@kth.se>
 * 
 * @version  $Revision: 1.0 Date: 2010/11/03 $ 
 * @modified 2011/02/01 
 */

configuration ControllerAppC{
}
implementation {

	components new TimerMilliC() as TimerBeacon;
	components MainC, ControllerC, LedsC, LocalTime62500hzC, new TimerMilliC() as TimerTimeout;
	ControllerC.Boot -> MainC;
	ControllerC.Leds -> LedsC;
	
	ControllerC.TimerTimeout -> TimerTimeout;
	ControllerC.TimerBeacon -> TimerBeacon;
		
	/****************************************
	* 802.15.4
	*****************************************/
#ifdef TKN154_BEACON_DISABLED
	components Ieee802154NonBeaconEnabledC as MAC;
#else
	components Ieee802154BeaconEnabledC as MAC;
	ControllerC.MLME_SCAN -> MAC;
	ControllerC.MLME_SYNC -> MAC;
	ControllerC.MLME_BEACON_NOTIFY -> MAC;
	ControllerC.MLME_SYNC_LOSS -> MAC;
	ControllerC.BeaconFrame -> MAC;
	
	components RandomC;
	ControllerC.Random -> RandomC;


#endif
	
	ControllerC.MLME_RESET -> MAC;
	ControllerC.MLME_SET -> MAC;
	ControllerC.MLME_GET -> MAC;
	  
	ControllerC.MLME_START -> MAC;
	ControllerC.MCPS_DATA -> MAC;
	ControllerC.Frame -> MAC;
	ControllerC.Packet -> MAC;

	ControllerC.LocalTime -> LocalTime62500hzC;

}

