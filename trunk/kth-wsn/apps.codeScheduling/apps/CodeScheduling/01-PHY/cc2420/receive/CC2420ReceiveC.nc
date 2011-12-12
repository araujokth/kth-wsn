configuration CC2420ReceiveC {
   provides interface StdAsyncControl as RxControl;
   provides interface CC2420sfd;
   provides interface RadioReceive;
   provides interface CC2420ReceivedNothing;
   uses interface Malloc;
//   uses interface OpenSerial;
   uses interface GeneralIO as CSN;
   uses interface GeneralIO as FIFO;
   uses interface GeneralIO as FIFOP;
   uses interface GpioInterrupt as InterruptFIFOP;
}

implementation {
   components CC2420ReceiveP;
   RxControl              = CC2420ReceiveP;
   Malloc                 = CC2420ReceiveP;
//   OpenSerial             = CC2420ReceiveP;
   CC2420sfd              = CC2420ReceiveP;
   RadioReceive           = CC2420ReceiveP;
   CC2420ReceivedNothing  = CC2420ReceiveP;
   CSN                    = CC2420ReceiveP.CSN;
   FIFO                   = CC2420ReceiveP.FIFO;
   FIFOP                  = CC2420ReceiveP.FIFOP;
   InterruptFIFOP         = CC2420ReceiveP;

   components MainC;
   MainC.SoftwareInit -> CC2420ReceiveP;

   components new CC2420SpiC();
   CC2420ReceiveP.SpiResource      -> CC2420SpiC;
   CC2420ReceiveP.RXFIFO           -> CC2420SpiC.RXFIFO;
   CC2420ReceiveP.SFLUSHRX         -> CC2420SpiC.SFLUSHRX;

   components HplMsp430GeneralIOC;
   CC2420ReceiveP.Port34 -> HplMsp430GeneralIOC.Port34;
}
