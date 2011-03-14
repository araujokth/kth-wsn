/*
 * SerialRS232P.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/06/10 
 * Last modification: 
 *            Author: aitorhh
 *     
 */

#include "AM.h"

#warning "Using SerialRS232"

module SerialRS232P {

  provides {
    interface Init;
    interface SplitControl;
    interface SendBytePacket;
    interface ReceiveBytePacket;
  }

  uses {
    interface SerialFrameComm;
    interface Leds;
    interface StdControl as SerialControl;
    interface SerialFlush;
  }
}
implementation {
  enum {
    RX_DATA_BUFFER_SIZE = 2,
    TX_DATA_BUFFER_SIZE = 4,
    SERIAL_MTU = 255,
    SERIAL_VERSION = 1,
  };

  enum {
    RXSTATE_NOSYNC,
    RXSTATE_TOKEN,
    RXSTATE_INFO,
    RXSTATE_INACTIVE
  };

  enum {
    TXSTATE_IDLE,
    TXSTATE_INFO,
    TXSTATE_FCS1,
    TXSTATE_FCS2,
    TXSTATE_ENDFLAG,
    TXSTATE_ENDWAIT,
    TXSTATE_FINISH,
    TXSTATE_ERROR,
    TXSTATE_INACTIVE
  };

  typedef enum {
    BUFFER_AVAILABLE,
    BUFFER_FILLING,
    BUFFER_COMPLETE,
  } tx_data_buffer_states_t;

  enum {
    TX_DATA_INDEX = 1,
    TX_BUFFER_COUNT = 2,
  };


  typedef struct {
    uint8_t writePtr;
    uint8_t readPtr;
    uint8_t buf[RX_DATA_BUFFER_SIZE+1]; // one wasted byte: writePtr == readPtr means empty
  } rx_buf_t;

  typedef struct {
    uint8_t state;
    uint8_t buf;    
  } tx_buf_t;
  
  /* Buffers */

  rx_buf_t rxBuf;
  tx_buf_t txBuf[TX_BUFFER_COUNT];

  /* Receive State */

  uint8_t  rxState;
  uint8_t  rxByteCnt;

  /* Transmit State */

  norace uint8_t  txState;
  norace uint8_t  txByteCnt;
  uint8_t  txPending;
  norace uint8_t txIndex;

  bool offPending = FALSE;

  // Prototypes

  inline void txInit();
  inline void rxInit();

  inline void rx_buffer_init();
  inline bool rx_buffer_is_full();
  inline bool rx_buffer_is_empty();
  inline void rx_buffer_push(uint8_t data);
  inline uint8_t rx_buffer_top();
  inline uint8_t rx_buffer_pop();

  void rx_state_machine(bool isDelimeter, uint8_t data);
  void MaybeScheduleTx();
  task void RunTx();
  


  inline void txInit(){
    uint8_t i;
    atomic for (i = 0; i < TX_BUFFER_COUNT; i++) txBuf[i].state = BUFFER_AVAILABLE;
    txState = TXSTATE_IDLE;
    txByteCnt = 0;
    txPending = FALSE;
    txIndex = 0;
  }

  inline void rxInit(){
    rxBuf.writePtr = rxBuf.readPtr = 0;
    rxState = RXSTATE_NOSYNC;
    rxByteCnt = 0;
  }

  command error_t Init.init() {

    txInit();
    rxInit();
    
    return SUCCESS;
  }


  /*
   *  buffer and queue manipulation
   */

  /* 
   * RX Buffer Manipulation
   */

  inline void rx_buffer_init(){
    rxBuf.writePtr = rxBuf.readPtr = 0;
  }
  inline bool rx_buffer_is_full() {
    uint8_t tmp = rxBuf.writePtr;
    if (++tmp > RX_DATA_BUFFER_SIZE) tmp = 0;
    return (tmp == rxBuf.readPtr);
  }
  inline bool rx_buffer_is_empty(){
    return (rxBuf.readPtr == rxBuf.writePtr);
  }
  inline void rx_buffer_push(uint8_t data){
    rxBuf.buf[rxBuf.writePtr] = data;
    if (++(rxBuf.writePtr) > RX_DATA_BUFFER_SIZE) rxBuf.writePtr = 0;
  }
  inline uint8_t rx_buffer_top(){
    uint8_t tmp = rxBuf.buf[rxBuf.readPtr];
    return tmp;
  }
  inline uint8_t rx_buffer_pop(){
    uint8_t tmp = rxBuf.buf[rxBuf.readPtr];
    if (++(rxBuf.readPtr) > RX_DATA_BUFFER_SIZE) rxBuf.readPtr = 0;
    return tmp;
  }
  
  task void startDoneTask() {
    call SerialControl.start();
    signal SplitControl.startDone(SUCCESS);
  }


  task void stopDoneTask() {
    call SerialFlush.flush();
  }

  event void SerialFlush.flushDone(){
    call SerialControl.stop();
    signal SplitControl.stopDone(SUCCESS);
  }

  task void defaultSerialFlushTask(){
    signal SerialFlush.flushDone();
  }
  default command void SerialFlush.flush(){
    post defaultSerialFlushTask();
  }

  command error_t SplitControl.start() {
    post startDoneTask();
    return SUCCESS;
  }

  void testOff() {
    bool turnOff = FALSE;
    atomic {
      if (txState == TXSTATE_INACTIVE &&
	  rxState == RXSTATE_INACTIVE) {
	turnOff = TRUE;
      }
    }
    if (turnOff) {
      post stopDoneTask();
      atomic offPending = FALSE;
    }
    else {
      atomic offPending = TRUE;
    }
  }
    
  command error_t SplitControl.stop() {
    atomic {
      if (rxState == RXSTATE_NOSYNC) {
	rxState = RXSTATE_INACTIVE;
      }
    }
    atomic {
      if (txState == TXSTATE_IDLE) {
	txState = TXSTATE_INACTIVE;
      }
    }
    testOff();
    return SUCCESS;
  }

  /*
   *  Receive Path
   */ 
  
  
  async event void SerialFrameComm.delimiterReceived(){
    rx_state_machine(TRUE,0);
  }
  async event void SerialFrameComm.dataReceived(uint8_t data){
    rx_state_machine(FALSE,data);
  }

  void rx_state_machine(bool isDelimeter, uint8_t data){

    switch (rxState) {
      
    case RXSTATE_NOSYNC: 
      if (isDelimeter) {
        rxInit();
        rxState = RXSTATE_TOKEN;
      }
      break;
      
    case RXSTATE_PROTO:
      if (!isDelimeter){
        rxState = RXSTATE_TOKEN;
        rxProto = data;
        if (!valid_rx_proto(rxProto))
          goto nosync;
        if (signal ReceiveBytePacket.startPacket() != SUCCESS){
          goto nosync;
        }
      }      
      break;
      
    case RXSTATE_TOKEN:
      if (isDelimeter) {
        goto nosync;
      }
      else {
        rxSeqno = data;
        rxState = RXSTATE_INFO;
      }
      break;
      
    case RXSTATE_INFO:
      if (rxByteCnt < SERIAL_MTU){ 
        if (isDelimeter) { /* handle end of frame */
          if (rxByteCnt >= 2) {
              signal ReceiveBytePacket.endPacket(SUCCESS);
              ack_queue_push(rxSeqno);
              goto nosync;
          }
          else {
            goto nosync;
          }
	}
        else { /* handle new bytes to save */
          if (rxByteCnt >= 2){ 
            signal ReceiveBytePacket.byteReceived(rx_buffer_top());
          }
	  rx_buffer_push(data);
          rxByteCnt++;
        }
      }
      
      /* no valid message.. */
      else {
        goto nosync;
       }
      break;
      
    default:      
      goto nosync;
    }
    goto done;

  nosync:
    /* reset all counters, etc */
    rxInit();
    call SerialFrameComm.resetReceive();
    signal ReceiveBytePacket.endPacket(FAIL);
    if (offPending) {
      rxState = RXSTATE_INACTIVE;
      testOff();
    }
    /* if this was a flag, start in proto state.. */
    else if (isDelimeter) {
      rxState = RXSTATE_PROTO;
    }
    
  done:
  }

  
  /*
   *  Send Path
   */ 


  void MaybeScheduleTx() {
    atomic {
      if (txPending == 0) {
        if (post RunTx() == SUCCESS) {
          txPending = 1;
        }
      }
    }
  }


  async command error_t SendBytePacket.completeSend(){
    bool ret = FAIL;
    atomic {
        txBuf[TX_DATA_INDEX].state = BUFFER_COMPLETE;
        ret = SUCCESS;
    }
    return ret;
  }

  async command error_t SendBytePacket.startSend(uint8_t b){
    bool not_busy = FALSE;
    atomic {
      if (txBuf[TX_DATA_INDEX].state == BUFFER_AVAILABLE){
        txBuf[TX_DATA_INDEX].state = BUFFER_FILLING;
        txBuf[TX_DATA_INDEX].buf = b;
        not_busy = TRUE;
      }
    }
    if (not_busy) {
      MaybeScheduleTx();
      return SUCCESS;
    }
    return EBUSY;

  }
  
  task void RunTx() {
    uint8_t idle;
    uint8_t done;
    uint8_t fail;
    
    /*
      the following trigger MaybeScheduleTx, which starts at most one RunTx:
      1) adding an ack to the ack queue (ack_queue_push())
      2) starting to send a packet (SendBytePacket.startSend())
      3) failure to send start delimiter in RunTx
      4) putDone: 
    */
    
    error_t result = SUCCESS;
    bool send_completed = FALSE;
    bool start_it = FALSE;
    
    atomic { 
      txPending = 0;
      idle = (txState == TXSTATE_IDLE);
      done = (txState == TXSTATE_FINISH);
      fail = (txState == TXSTATE_ERROR);
      if (done || fail){ 
        txState = TXSTATE_IDLE;
        txBuf[txIndex].state = BUFFER_AVAILABLE;
      }
    }
    
    /* if done, call the send done */
    if (done || fail) {
      txSeqno++;
      if (txProto == SERIAL_PROTO_ACK){
        ack_queue_pop();
      }
      else {
        result = done ? SUCCESS : FAIL;
        send_completed = TRUE;
      }
      idle = TRUE;
    }
    
    /* if idle, set up next packet to TX */ 
    if (idle) {
      bool goInactive;
      atomic goInactive = offPending;
      if (goInactive) {
        atomic txState = TXSTATE_INACTIVE;
      }
      else {
        /* acks are top priority */
        uint8_t myAckState;
        uint8_t myDataState;
        atomic {
          myAckState = txBuf[TX_ACK_INDEX].state;
          myDataState = txBuf[TX_DATA_INDEX].state;
        }
        if (!ack_queue_is_empty() && myAckState == BUFFER_AVAILABLE) {
          atomic {
            txBuf[TX_ACK_INDEX].state = BUFFER_COMPLETE;
            txBuf[TX_ACK_INDEX].buf = ack_queue_top();
          }
          txProto = SERIAL_PROTO_ACK;
          txIndex = TX_ACK_INDEX;
          start_it = TRUE;
        }
        else if (myDataState == BUFFER_FILLING || myDataState == BUFFER_COMPLETE){
          txProto = SERIAL_PROTO_PACKET_NOACK;
          txIndex = TX_DATA_INDEX;
          start_it = TRUE;
        }
        else {
          /* nothing to send now.. */
        }
      }
    }
    else {
      /* we're in the middle of transmitting */
    }
    
    if (send_completed){
      signal SendBytePacket.sendCompleted(result);
    }
    
    if (txState == TXSTATE_INACTIVE) {
      testOff();
      return;
    }
    
    if (start_it){
      /* OK, start transmitting ! */
      atomic { 
        txCRC = 0;
        txByteCnt = 0;
        txState = TXSTATE_PROTO; 
      }
      if (call SerialFrameComm.putDelimiter() != SUCCESS) {
        atomic txState = TXSTATE_ERROR; 
        MaybeScheduleTx();
      }
    }
    
  }
  
  async event void SerialFrameComm.putDone() {
    {
      error_t txResult = SUCCESS;
      
      switch (txState) {
        
      case TXSTATE_PROTO:

         txResult = call SerialFrameComm.putData(txProto);
#ifdef NO_TX_SEQNO
        txState = TXSTATE_INFO;
#else
        txState = TXSTATE_SEQNO;
#endif
        txCRC = crcByte(txCRC,txProto);
        break;
        
      case TXSTATE_SEQNO:
        txResult = call SerialFrameComm.putData(txSeqno);
        txState = TXSTATE_INFO;
        txCRC = crcByte(txCRC,txSeqno);
        break;
        
      case TXSTATE_INFO:
        atomic {
          txResult = call SerialFrameComm.putData(txBuf[txIndex].buf);
          txCRC = crcByte(txCRC,txBuf[txIndex].buf);
          ++txByteCnt;
          
          if (txIndex == TX_DATA_INDEX){
            uint8_t nextByte;
            nextByte = signal SendBytePacket.nextByte();
            if (txBuf[txIndex].state == BUFFER_COMPLETE || txByteCnt >= SERIAL_MTU){
              txState = TXSTATE_FCS1;
            }
            else { /* never called on ack b/c ack is BUFFER_COMPLETE initially */
              txBuf[txIndex].buf = nextByte;
            }
          }
          else { // TX_ACK_INDEX
            txState = TXSTATE_FCS1;
          }
        }
        break;
        
      case TXSTATE_FCS1:
        txResult = call SerialFrameComm.putData(txCRC & 0xff);
        txState = TXSTATE_FCS2;
        break;
        
      case TXSTATE_FCS2:
        txResult = call SerialFrameComm.putData((txCRC >> 8) & 0xff);
        txState = TXSTATE_ENDFLAG;
        break;
        
      case TXSTATE_ENDFLAG:
        txResult = call SerialFrameComm.putDelimiter();
        txState = TXSTATE_ENDWAIT;
        break;
        
      case TXSTATE_ENDWAIT:
        txState = TXSTATE_FINISH;
      case TXSTATE_FINISH:
        MaybeScheduleTx();
        break;
      case TXSTATE_ERROR:
      default:
        txResult = FAIL; 
        break;
      }
      
      if (txResult != SUCCESS) {
        txState = TXSTATE_ERROR;
        MaybeScheduleTx();
      }
    }
  }

  
 default event void SplitControl.startDone(error_t err) {}
 default event void SplitControl.stopDone(error_t err) {}
}
