#include "printf.h"
module CC2420DriversP @ safe() {
provides interface RadioControl;
provides interface RadioSend;
uses interface Alarm<T32khz,uint32_t> as RxWaitAlarm;
provides interface CC2420sfd;
//   uses interface OpenSerial;
uses interface StdAsyncControl as TxControl;
uses interface StdAsyncControl as RxControl;
uses interface CC2420Transmit;
uses interface CC2420Config;
uses interface CC2420ReceivedNothing;
uses interface HplMsp430GeneralIO as Port34;

uses interface PinDebug;
}

implementation {

	/*-------------------------------- variables -----------------------------------------*/

	enum {
		S_STOPPED = 0,
		S_STARTING_VREG = 1,
		S_STARTING_OSCILLATOR = 2,
		S_STARTED = 3,
		S_LOADING_PACKET = 4,
		S_SETTING_CHANNEL = 5,
		S_READY_TX = 6,
		S_TRANSMITTING = 7,
		S_READY_RX = 8,
		S_RECEIVING = 9,
		S_STOPPING = 10,
		S_WRITING_ID = 11,
	};

	uint8_t state = S_STOPPED;
	bool timeLimitedRx = TRUE;//set to TRUE at initialization
	uint16_t rxWaitTime;
	error_t sendErr = SUCCESS;
	uint8_t frequencyChannel = 0;//set to a non-existing at initialization
	bool syncForSend; //has setChannel been requested for Send (TRUE) or Receive (FALSE)?

	/*-------------------------------- prototypes ----------------------------------------*/

	task void taskStartDone();
	task void taskStopDone();
	task void taskSendDone();
	task void taskStartOscillatorDone();
	void shutdown();

	/*-------------------------------- startup sequence ----------------------------------*/

	async command error_t RadioControl.start() {
		uint8_t temp_state;
		atomic {
			temp_state=state;
		}
		switch (temp_state) {
			case S_STARTED:
			return EALREADY;
			case S_STARTING_VREG:
			case S_STARTING_OSCILLATOR:
			case S_WRITING_ID:
			return SUCCESS;
			default:
			atomic state = S_STARTING_VREG;
			call CC2420Config.startVReg();
		}
		return SUCCESS;
	}
	async event void CC2420Config.startVRegDone() {
		atomic state = S_STARTING_OSCILLATOR;
		call CC2420Config.startOscillator();
	}
	async event void CC2420Config.startOscillatorDone() {
		post taskStartOscillatorDone();
	}
	task void taskStartOscillatorDone() {
		atomic state = S_WRITING_ID;
		call CC2420Config.writeId();
	}
	async event void CC2420Config.writeIdDone() {
		post taskStartDone();
	}

	task void taskStartDone() {
		call TxControl.start();
		call RxControl.start();
		atomic state = S_STARTED;
		signal RadioControl.startDone(SUCCESS);
	}

	/*-------------------------------- receive sequence ----------------------------------*/

	async command error_t RadioControl.prepareReceive(uint8_t frequencyChannel_param) {
		uint8_t temp_state;
		atomic {
			temp_state=state;
			frequencyChannel = frequencyChannel_param;
		}
		if (temp_state!=S_STARTED) {
			//         call OpenSerial.printError(COMPONENT_CC2420DRIVER,ERR_WRONG_STATE_IN_PREPARERECEIVE,
			//               (errorparameter_t)temp_state,
			//               (errorparameter_t)0);
//			printf("%u\n",temp_state);
//			printfflush();
			return FAIL;
		}
		atomic syncForSend = FALSE;
		atomic state = S_SETTING_CHANNEL;
		call CC2420Config.setChannel(frequencyChannel_param);
		return SUCCESS;
	}
	async command error_t RadioControl.receiveNow(bool timeLimitedRx_param, uint16_t rxWaitTime_param) {
		atomic {
			state = S_RECEIVING;
			timeLimitedRx = timeLimitedRx_param;
			rxWaitTime = rxWaitTime_param;
		}
		if (timeLimitedRx_param==TRUE) {
			call RxWaitAlarm.start(rxWaitTime_param);

		}
		return call CC2420Config.rxOn();
	}
	async command void CC2420sfd.sfd( uint32_t txTimestamp ) { //from CC2420TransmitP
		call RxWaitAlarm.stop();
		//      call PinDebug.ADC0toggle();
	}
	async command void CC2420sfd.sfd_dropped() { //from CC2420TransmitP
		call RxWaitAlarm.stop();
		call TxControl.start();
		call RxControl.start();
		signal RadioControl.receivedNothing();
	}
	async event void CC2420ReceivedNothing.receivedNothing() { //from CC2420ReceiveP
		call RxWaitAlarm.stop();
		call TxControl.start();
		call RxControl.start();
		signal RadioControl.receivedNothing();
	}
	async event void RxWaitAlarm.fired() {
		call RxWaitAlarm.stop();
		call TxControl.start();
		call RxControl.start();
		signal RadioControl.receivedNothing();
	}

	/*-------------------------------- send sequence -------------------------------------*/

	async command error_t RadioSend.prepareSend(OpenQueueEntry_t* msg) {
		uint8_t temp_state;
		bool temp_timeLimitedRx;
		uint16_t temp_rxWaitTime;
		atomic {
			temp_state = state;
			temp_timeLimitedRx = timeLimitedRx;
			temp_rxWaitTime = rxWaitTime;
		}
		//msg->owner = COMPONENT_CC2420DRIVER;//don't become owner so  that L2 retransmits are possible
		if (temp_state!=S_STARTED && temp_state!=S_RECEIVING) {
			//         call OpenSerial.printError(COMPONENT_CC2420DRIVER,ERR_WRONG_STATE_IN_PREPARESEND,
			//               (errorparameter_t)temp_state,
			//               (errorparameter_t)0);
			if (temp_timeLimitedRx==FALSE) { //go back to S_RECEIVING
				call RadioControl.receiveNow(temp_timeLimitedRx, temp_rxWaitTime);
			} else { //go back to S_STARTED
				call RadioControl.rfOff();
			}
			return FAIL;
		}
		atomic {
			state = S_LOADING_PACKET;
			frequencyChannel = msg->l1_channel;
		}
		call CC2420Transmit.loadPacket(msg);
		return SUCCESS;
	}
	async event void CC2420Transmit.loadPacketDone(error_t error) {
		uint8_t temp_frequencyChannel;
		atomic {
			temp_frequencyChannel = frequencyChannel;
		}
		if (error==SUCCESS) {
			atomic syncForSend=TRUE;
			atomic state = S_SETTING_CHANNEL;
			if (call CC2420Config.setChannel(temp_frequencyChannel)!=SUCCESS) {
				//            call OpenSerial.printError(COMPONENT_CC2420DRIVER,ERR_SETCHANNEL_FAILED,
				//                  (errorparameter_t)0,
				//                  (errorparameter_t)0);
			}
		} else {
			atomic state = S_STARTED;
			signal RadioSend.prepareSendDone(FAIL);
		}
	}
	async event void CC2420Config.setChannelDone(error_t error) {
		bool temp_syncForSend;
		atomic {
			temp_syncForSend = syncForSend;
		}
		if (temp_syncForSend) {
			if (error==SUCCESS) {
				atomic state = S_READY_TX;
				signal RadioSend.prepareSendDone(SUCCESS);
			} else {
				atomic state = S_STARTED;
				signal RadioSend.prepareSendDone(FAIL);
			}
		} else {
			if (error==SUCCESS) {
				atomic state = S_READY_RX;
				signal RadioControl.prepareReceiveDone(SUCCESS);
			} else {
				atomic state = S_STARTED;
				signal RadioControl.prepareReceiveDone(FAIL);
			}
		}
	}
	async command error_t RadioSend.sendNow() {
		bool useCca = FALSE;
		atomic state = S_TRANSMITTING;
		return call CC2420Transmit.sendNow(useCca);
	}
	async event void CC2420Transmit.sendNowDone(error_t err) {
		atomic sendErr = err;
		post taskSendDone();
	}
	task void taskSendDone() {
		uint8_t temp_state;
		error_t temp_sendErr;
		atomic {
			temp_state = state;
			temp_sendErr = sendErr;
		}
		if (temp_state == S_STOPPING) {
			shutdown();
		} else {
			atomic state = S_STARTED;
		}
		signal RadioSend.sendNowDone(temp_sendErr);
	}

	/*-------------------------------- shutdown sequence ---------------------------------*/

	async command error_t RadioControl.stop() {
		uint8_t temp_state;
		atomic temp_state=state;
		switch (temp_state) {
			case S_TRANSMITTING:
			atomic state=S_STOPPING; //the radio will shut down in sendDone()
			return SUCCESS;
			break;
			case S_STOPPING:
			return SUCCESS;
			break;
			case S_STOPPED:
			return EALREADY;
			break;
			default:
			atomic state = S_STOPPING;
			shutdown();
			return SUCCESS;
			break;
		}
		return EBUSY;
	}
	void shutdown() {
		call TxControl.stop();
		call RxControl.stop();
		call CC2420Config.stopVReg();
		post taskStopDone();
	}
	task void taskStopDone() {
		atomic state=S_STOPPED;
		signal RadioControl.stopDone( SUCCESS );
	}

	/*-------------------------------- switching off RF ----------------------------------*/

	async command error_t RadioControl.rfOff() {
		error_t temp_return;
		temp_return = call CC2420Config.rfOff();
		atomic state = S_STARTED;
		call TxControl.start();
		call RxControl.start();
		return temp_return;
	}
}
