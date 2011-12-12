#include "Timer.h"
#include "IDManager.h"

module CC2420ControlP @ safe() {

	//public
provides interface CC2420Config;
//   uses interface OpenSerial;

//public within the drivers
uses interface GeneralIO as CSN;
uses interface GeneralIO as RSTN;
uses interface GeneralIO as VREN;
uses interface GpioInterrupt as InterruptCCA;
uses interface Alarm<T32khz,uint32_t> as StartupAlarm;

//private
provides interface Init;

uses interface CC2420Ram as PANID;
uses interface CC2420Ram as SHORTADR;
uses interface CC2420Ram as IEEEADR;
uses interface CC2420Register as FSCTRL;
uses interface CC2420Register as IOCFG0;
uses interface CC2420Register as IOCFG1;
uses interface CC2420Register as MDMCTRL0;
uses interface CC2420Register as MDMCTRL1;
uses interface CC2420Register as RXCTRL1;
uses interface CC2420Register as RSSI;
uses interface CC2420Strobe as SRXON;
uses interface CC2420Strobe as SRFOFF;
uses interface CC2420Strobe as SXOSCOFF;
uses interface CC2420Strobe as SXOSCON;

uses interface Resource as OscResource;
uses interface Resource as SyncResource;
uses interface Resource as RxOnResource;
uses interface Resource as RfOffResource;
uses interface Resource as SpiResource;

uses interface Leds;
uses interface PinDebug;
}

implementation {

	/*-------------------------------- variables -----------------------------------------*/

	typedef enum {
		S_VREG_STOPPED,
		S_VREG_STARTING,
		S_VREG_STARTED,
		S_XOSC_STARTING,
		S_XOSC_STARTED,
	}cc2420_control_state_t;

	uint8_t m_channel;

	bool m_sync_busy = FALSE;
	bool m_rxOn_busy = FALSE;
	bool m_rfOff_busy = FALSE;

	norace cc2420_control_state_t m_state = S_VREG_STOPPED;

	/*-------------------------------- prototypes ----------------------------------------*/

	void startOscillator();
	void writeId();
	void syncConfig();
	void rxOn();
	void rfOff();

	void taskWriteId();

	/*-------------------------------- start voltage regulator sequence ------------------*/

	//no SPI access needed
	async command error_t CC2420Config.startVReg() {
		atomic {
			if ( m_state != S_VREG_STOPPED ) {
				return FAIL;
			}
			m_state = S_VREG_STARTING;
		}
		call VREN.set();
		call StartupAlarm.start( CC2420_TIME_VREN );
		return SUCCESS;
	}
	async event void StartupAlarm.fired() {
		if ( m_state == S_VREG_STARTING ) {
			m_state = S_VREG_STARTED;
			call RSTN.clr();
			call RSTN.set();
			signal CC2420Config.startVRegDone();
		}
	}

	/*-------------------------------- start oscillator sequence -------------------------*/

	async command error_t CC2420Config.startOscillator() {
		atomic {
			if ( m_state == S_VREG_STARTED ) {
				m_state = S_XOSC_STARTING;
				if ((call OscResource.immediateRequest()) != SUCCESS) {
					call OscResource.request();
				} else {
					startOscillator();
				}
			} else {
				return FAIL;
			}
		}
		return SUCCESS;
	}
	event void OscResource.granted() {
		startOscillator();
	}
	void startOscillator() {
		call CSN.clr();

		call IOCFG1.write( CC2420_SFDMUX_XOSC16M_STABLE << CC2420_IOCFG1_CCAMUX );
		call InterruptCCA.enableRisingEdge();
		call SXOSCON.strobe();

		call IOCFG0.write( ( 1 << CC2420_IOCFG0_FIFOP_POLARITY ) | ( 127 << CC2420_IOCFG0_FIFOP_THR ) );
		//RESERVED_FRAME_MODE=1  reserved frame types are accepted by address recognition
		//PAN_COORDINATOR=0      not a PAN coordinator
		//ADR_DECODE=0           radio does not do address recognition
		//CCA_HYST=2             CCA Hysteresis in dB, default
		//CCA_MOD=3              default
		//AUTOCRC=1              radio will append CRC on TX and check on RX
		//AUTOACK=0              radio does not send an ACK automatically
		//PREAMBLE_LENGTH=2      3 leading zero bytes (IEEE 802.15.4 compliant)

		call MDMCTRL0.write( ( 0 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE ) |
				( 0 << CC2420_MDMCTRL0_PAN_COORDINATOR ) |
				( 1 << CC2420_MDMCTRL0_ADR_DECODE ) |
				( 2 << CC2420_MDMCTRL0_CCA_HYST ) |
				( 3 << CC2420_MDMCTRL0_CCA_MOD ) |
				( 1 << CC2420_MDMCTRL0_AUTOCRC ) |
				( 1 << CC2420_MDMCTRL0_AUTOACK ) |
				( 2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH ) );

		call RXCTRL1.write( ( 1 << CC2420_RXCTRL1_RXBPF_LOCUR ) |
				( 1 << CC2420_RXCTRL1_LOW_LOWGAIN ) |
				( 1 << CC2420_RXCTRL1_HIGH_HGM ) |
				( 1 << CC2420_RXCTRL1_LNA_CAP_ARRAY ) |
				( 1 << CC2420_RXCTRL1_RXMIX_TAIL ) |
				( 1 << CC2420_RXCTRL1_RXMIX_VCM ) |
				( 2 << CC2420_RXCTRL1_RXMIX_CURRENT ) );

		call CSN.set();
	}
	async event void InterruptCCA.fired() {
		m_state = S_XOSC_STARTED;
		call InterruptCCA.disable();
		call CSN.clr();
		call IOCFG1.write( 0 );
		call CSN.set();
		call OscResource.release();
		signal CC2420Config.startOscillatorDone();
	}

	/*-------------------------------- write address recognition registers--------------*/

	async command error_t CC2420Config.writeId() {
		atomic {
			if ((call SpiResource.immediateRequest()) != SUCCESS) {
				call SpiResource.request();
			} else {
				writeId();
			}
		}
		return SUCCESS;
	}
	event void SpiResource.granted() {
		writeId();
	}
	void writeId() {
		nxle_uint16_t id[ 6 ];
		atomic {
			//        call FrameUtility.copyLocalExtendedAddressLE((uint8_t*) &id);
			id[ 4 ] = PAN_ID;
			id[ 5 ] = TOS_NODE_ID;
		}
		call CSN.clr();
		// ext.adr, PANID and short adr are located at consecutive addresses in RAM
		call IEEEADR.write(0, (uint8_t*)&id, sizeof(id) );
		call CSN.set();
		call SpiResource.release();
		signal CC2420Config.writeIdDone();
	}

	/*-------------------------------- set channel sequence ----------------------------*/

	async command error_t CC2420Config.setChannel( uint8_t channel ) {
		atomic {
			m_channel = channel;
			if ( m_sync_busy ) {
//				call PinDebug.ADC0toggle();
				return FAIL;
			}
			if ( m_state == S_XOSC_STARTED ) {
				m_sync_busy = TRUE;
				if ((call SyncResource.immediateRequest()) != SUCCESS) {
					call SyncResource.request();
				} else {
					syncConfig();
				}
			} else {
				call PinDebug.ADC2toggle();
				return FAIL;
			}
		}
		return SUCCESS;
	}
	event void SyncResource.granted() {
		syncConfig();
	}
	void syncConfig() {
		uint8_t temp_channel;
		atomic {
			temp_channel = m_channel;
		}
		call PinDebug.ADC1toggle();
		call CSN.clr();
		call SRFOFF.strobe();
		//LOCK_THR=1: lock iif 128 consecutive reference clock periods with successful synchronisation (default)
		call FSCTRL.write( ( 1 << CC2420_FSCTRL_LOCK_THR ) |
				( ( (temp_channel - 11)*5+357 ) << CC2420_FSCTRL_FREQ ) );
		call CSN.set();
		call SyncResource.release();
		atomic m_sync_busy = FALSE;
		signal CC2420Config.setChannelDone(SUCCESS);
	}

	/*-------------------------------- switching RF on/off -----------------------------*/

	async command error_t CC2420Config.rxOn() {
		atomic {
			if ( m_rxOn_busy ) {
				return FAIL;
			}
			m_rxOn_busy = TRUE;
			if ( m_state == S_XOSC_STARTED ) {
				if ((call RxOnResource.immediateRequest()) != SUCCESS) {
					call RxOnResource.request();
				} else {
					rxOn();
				}
			} else {
				return FAIL;
			}
		}
		return SUCCESS;
	}
	event void RxOnResource.granted() {
		rxOn();
	}
	void rxOn() {
		call CSN.clr();
		call SRXON.strobe();
		call CSN.set();
		call RxOnResource.release();
		atomic m_rxOn_busy = FALSE;
	}

	async command error_t CC2420Config.rfOff() {
		atomic {
			if ( m_rfOff_busy ) {
				return FAIL;
			}
			m_rfOff_busy = TRUE;
			if ( m_state == S_XOSC_STARTED ) {
				if ((call RfOffResource.immediateRequest()) != SUCCESS) {
					call RfOffResource.request();
				} else {
					rfOff();
				}
			} else {
				m_rfOff_busy = FALSE;
				return FAIL;
			}
		}
		return SUCCESS;
	}
	event void RfOffResource.granted() {
		rfOff();
	}
	void rfOff() {
		call CSN.clr();
		call SRFOFF.strobe();
		call CSN.set();
		call RfOffResource.release();
		atomic m_rfOff_busy = FALSE;
	}

	/*-------------------------------- stop oscillator sequence --------------------------*/

	async command error_t CC2420Config.stopOscillator() {
		atomic {
			if ( m_state != S_XOSC_STARTED ) {
				return FAIL;
			}
			m_state = S_VREG_STARTED;
			call SXOSCOFF.strobe();
		}
		return SUCCESS;
	}

	/*-------------------------------- stop voltage regulator sequence -------------------*/

	async command error_t CC2420Config.stopVReg() {
		m_state = S_VREG_STOPPED;
		call RSTN.clr();
		call VREN.clr();
		call RSTN.set();
		return SUCCESS;
	}

	/*-------------------------------- misc ----------------------------------------------*/

	//Init
	command error_t Init.init() {
		call CSN.makeOutput();
		call RSTN.makeOutput();
		call VREN.makeOutput();
		m_channel = CC2420_DEF_CHANNEL;
		return SUCCESS;
	}

	void taskWriteId()
	{
		nxle_uint16_t id[ 6 ];
		atomic {
			//        call FrameUtility.copyLocalExtendedAddressLE((uint8_t*) &id);
			id[ 4 ] = PAN_ID;
			id[ 5 ] = TOS_NODE_ID;
		}
		call CSN.clr();
		call IEEEADR.write(0, (uint8_t*)&id, sizeof(id) );
		//      call PANID.write(0, (uint8_t*)&pan_id, sizeof(nxle_uint16_t) );
		call CSN.set();
	}
}
