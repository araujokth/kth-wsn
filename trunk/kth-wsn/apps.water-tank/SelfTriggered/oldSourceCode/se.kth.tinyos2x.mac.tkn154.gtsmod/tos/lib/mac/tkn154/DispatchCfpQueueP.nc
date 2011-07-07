/**
 * DispatchCfpQueueP.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.mac.tkn154
 *        Created on: 2010/05/05  
 * Last modification:   
 *            @author: aitorhh
 *     
 */

#include "TKN154_MAC.h"
generic module DispatchCfpQueueP() {
  provides
  {
    interface Init as Reset;
    interface FrameTx[uint8_t client];
    interface FrameRx as FrameRx[uint8_t frameType];
    interface Purge;
  } uses {
    interface Queue<ieee154_txframe_t*>;
    interface FrameTx as CfpTx;

  }
}
implementation
{
  task void txTask();
  bool m_state;
  uint8_t m_client;

  enum {
    TX_DONE_PENDING = 0x01,
    RESET_PENDING = 0x02,
  };

  bool isTxDonePending() { return (m_state & TX_DONE_PENDING) ? TRUE : FALSE; }
  void setTxDonePending() { m_state |= TX_DONE_PENDING; }
  void resetTxDonePending() { m_state &= ~TX_DONE_PENDING; }

  bool isResetPending() { return (m_state & RESET_PENDING) ? TRUE : FALSE; }
  void setResetPending() { m_state |= RESET_PENDING; }
  void resetResetPending() { m_state &= ~RESET_PENDING; }

  command error_t Reset.init()
  {
    setResetPending();
    while (call Queue.size()) {
      ieee154_txframe_t *txFrame = call Queue.dequeue();
      signal FrameTx.transmitDone[txFrame->client](txFrame, IEEE154_TRANSACTION_OVERFLOW);
    }
    resetResetPending();
    return SUCCESS;
  }

  command ieee154_status_t FrameTx.transmit[uint8_t client](ieee154_txframe_t *txFrame)
  {
    // request to send a frame in a GTS slot (triggered by MCPS_DATA.request())     
    txFrame->client = client;
    if (isResetPending() || call Queue.enqueue(txFrame) != SUCCESS)
      return IEEE154_TRANSACTION_OVERFLOW;
    else {
      post txTask();
      return IEEE154_SUCCESS;
    }
  }

  task void txTask()
  {
    if (!isTxDonePending() && call Queue.size()) {
      ieee154_txframe_t *txFrame = call Queue.head();
      if (txFrame->headerLen == 0) { 
        // was purged
        call Queue.dequeue();
        signal Purge.purgeDone(txFrame, IEEE154_SUCCESS);
        post txTask();
      }
      m_client = txFrame->client;
      setTxDonePending();
      if (call CfpTx.transmit(txFrame) != IEEE154_SUCCESS)
        resetTxDonePending();
    }
  }

  event void CfpTx.transmitDone(ieee154_txframe_t *txFrame, ieee154_status_t status)
  {
    resetTxDonePending();
    if (!call Queue.size())
      return; // all frames were spooled out (reset)
    call Queue.dequeue();
    signal FrameTx.transmitDone[txFrame->client](txFrame, status);
    if (IEEE154_BEACON_ENABLED_PAN && status == IEEE154_NO_BEACON) {
      // this means that we lost sync -> spool out all queued frames 
      while (call Queue.size()) {
        ieee154_txframe_t *frame = call Queue.dequeue();
        signal FrameTx.transmitDone[frame->client](frame, IEEE154_NO_BEACON);
      }
    }
    post txTask();
  }

  default event void FrameTx.transmitDone[uint8_t client](ieee154_txframe_t *txFrame, ieee154_status_t status) {}

  command ieee154_status_t Purge.purge(uint8_t msduHandle)
  {
    // request to purge a frame (triggered by MCPS_DATA.purge())

    uint8_t qSize = call Queue.size(), i;
    if (qSize > 1) {
      for (i=0; i<qSize-1; i++) {
        ieee154_txframe_t *txFrame = call Queue.element(i);
        if (((txFrame->header->mhr[MHR_INDEX_FC1] & FC1_FRAMETYPE_MASK) == FC1_FRAMETYPE_DATA) &&
          txFrame->handle == msduHandle) {
          txFrame->headerLen = 0; // mark as invalid
          return IEEE154_SUCCESS;
        }
      }
    }
    return IEEE154_INVALID_HANDLE;
  }
  
  default event void Purge.purgeDone(ieee154_txframe_t *txFrame, ieee154_status_t status) {}
}
