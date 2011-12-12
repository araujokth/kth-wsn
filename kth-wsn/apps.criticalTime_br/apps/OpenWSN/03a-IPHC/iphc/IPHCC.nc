#include "OpenWSN.h"

configuration IPHCC {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   uses     interface OpenReceiveIp;
   //misc
   uses     interface OpenSerial;
   uses     interface IDManager;
   uses     interface NeighborGet;
}
implementation {
   components IPHCP;
   OpenSendFromUpper  = IPHCP.OpenSendFromUpper;
   OpenSendToLower    = IPHCP.OpenSendToLower;
   OpenReceiveFromLower        = IPHCP.OpenReceiveFromLower;
   OpenReceiveIp      = IPHCP.OpenReceiveIp;
   OpenSerial         = IPHCP.OpenSerial;
   IDManager          = IPHCP.IDManager;
   NeighborGet        = IPHCP.NeighborGet;

   components PacketFunctionsC;
   IPHCP.PacketFunctions->PacketFunctionsC;
}
