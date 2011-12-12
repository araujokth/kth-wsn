configuration OpenWSNWiring {
}
implementation {
	components MainC;
	//07-App
	//   components AppTcpOhloneC;
	//   components AppTcpPrintC;
	//   components AppTcpEchoC;
	//   components AppTcpInjectC;
	//   components AppUdpPrintC;
	//   components AppUdpEchoC;
	//   components AppUdpInjectC;
	//   components AppUdpSensorC;

	components appDATADevC;
	//   //04-TRAN
	//   components UDPC;
	//   components TCPC;
	//   //03b-RPL
	//   components ForwardingC;
	//   components ICMPv6C;
	//   components ICMPv6EchoC;
	//   components ICMPv6RouterC;
	//   components ICMPv6RPLC;
	//   //03a-IPHC
	//   components OpenBridgeC;
	//   components IPHCC;
	//02b-RES
	//components NoRESC            as RESC;
	components LatinResC as RESC;
	//   components URESC             as RESC;
//	components NeighborsC;
	//02a-MAC
	//components stupidMACC        as MACC;
	components LatinMacC as MACC;
	/* add 154E here */
	//01-PHY
	components CC2420DriversC as RadioDriversC;
	/* add RF231 drivers here */
	//cross-layers
	components IDManagerC;
	components OpenQueueC;
	//   components OpenSerialC;
	//misc
	components HplMsp430GeneralIOC;


	/*================================== 07-Application ==========================================*/

	//   AppTcpOhloneC.Malloc->OpenQueueC;
	//   AppTcpOhloneC.IDManager->IDManagerC;
	//   AppTcpOhloneC.OpenSerial->OpenSerialC;
	//   AppTcpOhloneC.TCPControl[WKP_TCP_HTTP]                     -> TCPC.TCPControl[WKP_TCP_HTTP];
	//   AppTcpOhloneC.OpenSendToLower[WKP_TCP_HTTP]                -> TCPC.OpenSendFromUpper[WKP_TCP_HTTP];
	//   TCPC.OpenReceiveToUpper[WKP_TCP_HTTP]                   -> AppTcpOhloneC.OpenReceiveFromLower[WKP_TCP_HTTP];
	//
	//   AppTcpPrintC.IDManager->IDManagerC;
	//   AppTcpPrintC.Malloc->OpenQueueC;
	//   AppTcpPrintC.OpenSerial->OpenSerialC;
	//   AppTcpPrintC.TCPControl[WKP_TCP_DISCARD]                -> TCPC.TCPControl[WKP_TCP_DISCARD];
	//   TCPC.OpenReceiveToUpper[WKP_TCP_DISCARD]                -> AppTcpPrintC.OpenReceiveFromLower[WKP_TCP_DISCARD];
	//
	//   AppTcpEchoC.Malloc->OpenQueueC;
	//   AppTcpEchoC.IDManager->IDManagerC;
	//   AppTcpEchoC.OpenSerial->OpenSerialC;
	//   AppTcpEchoC.TCPControl[WKP_TCP_ECHO]                    -> TCPC.TCPControl[WKP_TCP_ECHO];
	//   AppTcpEchoC.OpenSendToLower[WKP_TCP_ECHO]               -> TCPC.OpenSendFromUpper[WKP_TCP_ECHO];
	//   TCPC.OpenReceiveToUpper[WKP_TCP_ECHO]                   -> AppTcpEchoC.OpenReceiveFromLower[WKP_TCP_ECHO];
	//
	//   AppTcpInjectC.IDManager->IDManagerC;
	//   AppTcpInjectC.Malloc->OpenQueueC;
	//   AppTcpInjectC.OpenSerial->OpenSerialC;
	//   AppTcpInjectC.TCPControl[WKP_TCP_INJECT]                -> TCPC.TCPControl[WKP_TCP_INJECT];
	//   AppTcpInjectC.OpenSendToLower[WKP_TCP_INJECT]           -> TCPC.OpenSendFromUpper[WKP_TCP_INJECT];
	//
	//   AppUdpPrintC.IDManager->IDManagerC;
	//   AppUdpPrintC.Malloc->OpenQueueC;
	//   AppUdpPrintC.OpenSerial->OpenSerialC;
	//   UDPC.OpenReceiveToUpper[WKP_UDP_DISCARD]                -> AppUdpPrintC.OpenReceiveFromLower[WKP_UDP_DISCARD];
	//
	//   AppUdpEchoC.IDManager->IDManagerC;
	//   AppUdpEchoC.Malloc->OpenQueueC;
	//   AppUdpEchoC.OpenSerial->OpenSerialC;
	//   UDPC.OpenReceiveToUpper[WKP_UDP_ECHO]                   -> AppUdpEchoC.OpenReceiveFromLower[WKP_UDP_ECHO];
	//   AppUdpEchoC.OpenSendToLower[WKP_UDP_ECHO]               -> UDPC.OpenSendFromUpper[WKP_UDP_ECHO];
	//
	//   AppUdpInjectC.IDManager->IDManagerC;
	//   AppUdpInjectC.Malloc->OpenQueueC;
	//   AppUdpInjectC.OpenSerial->OpenSerialC;
	//   AppUdpInjectC.OpenSendToLower[WKP_UDP_INJECT]           -> UDPC.OpenSendFromUpper[WKP_UDP_INJECT];
	//
	//   AppUdpSensorC.IDManager->IDManagerC;
	//   AppUdpSensorC.Malloc->OpenQueueC;
	//   AppUdpSensorC.OpenSerial->OpenSerialC;
	//   AppUdpSensorC.OpenSendToLower[WKP_UDP_SENSOR]           -> UDPC.OpenSendFromUpper[WKP_UDP_SENSOR];

	//   BlinkHopCoordC.OpenSendToLower 						   -> MACC.OpenSendFromUpper;
	//   BlinkHopCoordC.IDManager		 						   -> IDManagerC;
	//   BlinkHopCoordC.Malloc		 						   -> OpenQueueC.Malloc;

	////   MACC.OpenReceiveToUpper	 					   			->BlinkHopDevC.OpenReceiveFromLower
	//   BlinkHopDevC.IDManager		 						   -> IDManagerC;
	//   BlinkHopDevC.Malloc		 						   		-> OpenQueueC.Malloc;

	appDATADevC.OpenSendToLower -> RESC.OpenSendFromUpper;

	appDATADevC.Malloc -> OpenQueueC;
//	appDATADevC.NeighborGet -> NeighborsC;
	appDATADevC.IDManager -> IDManagerC;


	/*================================== 04-TRAN==================================================*/
	//
	//   UDPC.OpenSendToLower[IANA_UDP]                          -> ForwardingC.OpenSendFromUpper[IANA_UDP];
	//   UDPC.IDManager->IDManagerC;
	//   UDPC.OpenSerial->OpenSerialC;
	//   UDPC.Malloc->OpenQueueC;
	//
	//   TCPC.OpenSendToLower[IANA_TCP]                          -> ForwardingC.OpenSendFromUpper[IANA_TCP];
	//   TCPC.IDManager->IDManagerC;
	//   TCPC.Malloc->OpenQueueC;
	//   TCPC.OpenSerial->OpenSerialC;

	/*================================== 03b-IPv6 ================================================*/

	//   ForwardingC.OpenReceiveToUpper[IANA_UDP]                -> UDPC.OpenReceiveFromLower[IANA_UDP];
	//   ForwardingC.OpenReceiveToUpper[IANA_TCP]                -> TCPC.OpenReceiveFromLower[IANA_TCP];
	//   ForwardingC.OpenReceiveToUpper[IANA_ICMPv6]             -> ICMPv6C.OpenReceiveFromLower[IANA_ICMPv6];
	//   ForwardingC.NeighborGet->NeighborsC;
	//   ForwardingC.Malloc->OpenQueueC;
	//   ForwardingC.IDManager->IDManagerC;
	//   ForwardingC.OpenSerial->OpenSerialC;
	//   ForwardingC.OpenSendToLower                             -> IPHCC.OpenSendFromUpper;
	//
	//   ICMPv6C.OpenSendToLower[IANA_ICMPv6]                    -> ForwardingC.OpenSendFromUpper[IANA_ICMPv6];
	//   ICMPv6C.OpenReceiveToUpper[IANA_ICMPv6_ECHO_REQUEST]    -> ICMPv6EchoC.OpenReceiveFromLower[IANA_ICMPv6_ECHO_REQUEST];
	//   ICMPv6C.OpenReceiveToUpper[IANA_ICMPv6_ECHO_REPLY]      -> ICMPv6EchoC.OpenReceiveFromLower[IANA_ICMPv6_ECHO_REPLY];
	//   ICMPv6C.OpenReceiveToUpper[IANA_ICMPv6_RS]              -> ICMPv6RouterC.OpenReceiveFromLower[IANA_ICMPv6_RS];
	//   ICMPv6C.OpenReceiveToUpper[IANA_ICMPv6_RA]              -> ICMPv6RouterC.OpenReceiveFromLower[IANA_ICMPv6_RA];
	//   ICMPv6C.OpenReceiveToUpper[IANA_ICMPv6_RPL]             -> ICMPv6RPLC.OpenReceiveFromLower[IANA_ICMPv6_RPL];
	//   ICMPv6C.Malloc->OpenQueueC;
	//   ICMPv6C.IDManager->IDManagerC;
	//   ICMPv6C.OpenSerial->OpenSerialC;
	//
	//   ICMPv6EchoC.OpenSendToLower[IANA_ICMPv6_ECHO_REQUEST]   -> ICMPv6C.OpenSendFromUpper[IANA_ICMPv6_ECHO_REQUEST];
	//   ICMPv6EchoC.OpenSendToLower[IANA_ICMPv6_ECHO_REPLY]     -> ICMPv6C.OpenSendFromUpper[IANA_ICMPv6_ECHO_REPLY];
	//   ICMPv6EchoC.Malloc->OpenQueueC;
	//   ICMPv6EchoC.IDManager->IDManagerC;
	//   ICMPv6EchoC.OpenSerial->OpenSerialC;
	//
	//   ICMPv6RouterC.OpenSendToLower[IANA_ICMPv6_RS]           -> ICMPv6C.OpenSendFromUpper[IANA_ICMPv6_RS];
	//   ICMPv6RouterC.OpenSendToLower[IANA_ICMPv6_RA]           -> ICMPv6C.OpenSendFromUpper[IANA_ICMPv6_RA];
	//   ICMPv6RouterC.Malloc->OpenQueueC;
	//   ICMPv6RouterC.IDManager->IDManagerC;
	//   ICMPv6RouterC.OpenSerial->OpenSerialC;
	//
	//   ICMPv6RPLC.OpenSendToLower[IANA_ICMPv6_RPL]             -> ICMPv6C.OpenSendFromUpper[IANA_ICMPv6_RPL];
	//   ICMPv6RPLC.Malloc->OpenQueueC;
	//   ICMPv6RPLC.IDManager->IDManagerC;
	//   ICMPv6RPLC.OpenSerial->OpenSerialC;

	/*================================== 03a-IPHC ================================================*/
	//
	//   IPHCC.OpenReceiveIp                                     -> ForwardingC.OpenReceiveIp;
	//   IPHCC.OpenSendToLower                                   -> RESC.OpenSendFromUpper;
	//   IPHCC.OpenSerial->OpenSerialC;
	//   IPHCC.IDManager->IDManagerC;
	//   IPHCC.NeighborGet->NeighborsC;

	//   OpenBridgeC.OpenSendToLower                             -> RESC.OpenSendFromBridge;
	//   OpenBridgeC.OpenSerial->OpenSerialC;
	//   OpenBridgeC.IDManager->IDManagerC;
	//   OpenBridgeC.Malloc->OpenQueueC;

	/*================================== 02b-RES =================================================*/

	RESC.OpenSendToLower -> MACC.OpenSendFromUpper;
	RESC.MAC -> MACC.MAC;
	//   RESC.OpenReceiveToUpper                                 -> IPHCC.OpenReceiveFromLower;
	RESC.OpenReceiveToUpper -> appDATADevC.OpenReceiveFromLower;

//	RESC.NeighborGet->NeighborsC;
	RESC.Malloc->OpenQueueC;
	//   RESC.OpenSerial->OpenSerialC;
//	RESC.OpenReceiveADVToNeighbor -> NeighborsC.OpenReceiveADVFromLower;
	//   RESC.OpenReceiveToBridge                                -> OpenBridgeC.OpenReceiveFromLower;
	RESC.IDManager->IDManagerC;

//	NeighborsC.GlobalTime->MACC;
//	NeighborsC.GlobalSync->MACC;
//	NeighborsC.OpenQueue->OpenQueueC;
//	NeighborsC.Malloc->OpenQueueC;
//	NeighborsC.IDManager->IDManagerC;
	//   NeighborsC.OpenSerial->OpenSerialC;
//	NeighborsC.OpenSendKAToLower -> RESC.OpenSendKAFromNeighbors;

	RESC.OpenQueue->OpenQueueC;

	/*================================== 02-MACC ===========================================*/


	//   CellUsageC.OpenSerial->OpenSerialC;

	MACC.Boot->MainC;
	MACC.OpenQueue->OpenQueueC;
//	MACC.NeighborGet->NeighborsC;


	MACC.IDManager->IDManagerC;
	MACC.Malloc->OpenQueueC;
	//   MACC.OpenSerial->OpenSerialC;
//	MACC.NeighborStats->NeighborsC;
	MACC.OpenReceiveToUpper -> RESC.OpenReceiveFromLower;
	MACC.LatinMatrix -> RESC.LatinMatrix;
	MACC.RadioControl -> RadioDriversC;
	MACC.RadioSend -> RadioDriversC;
	MACC.RadioReceive -> RadioDriversC;


	/*================================== 01-PHY ==================================================*/

	RadioDriversC.GlobalTime->MACC;
	RadioDriversC.Malloc->OpenQueueC;
	//   RadioDriversC.OpenSerial->OpenSerialC;

	/*================================== cross-layers ============================================*/

	//   IDManagerC.OpenSerial->OpenSerialC;

	OpenQueueC.GlobalTime->MACC;
	//   OpenQueueC.OpenSerial->OpenSerialC;

	//   OpenSerialC.IDManager->IDManagerC;
	//   OpenSerialC.TriggerIDManagerAboutBridge->IDManagerC.TriggerIDManagerAboutBridge;
	//   OpenSerialC.TriggerIDManagerAboutRoot->IDManagerC.TriggerIDManagerAboutRoot;
	//   OpenSerialC.TriggerTCPInject->AppTcpInjectC;
	//   OpenSerialC.TriggerUDPInject->AppUdpInjectC;
	//   OpenSerialC.TriggerUDPSensor->AppUdpSensorC;
	//   OpenSerialC.TriggerICMPv6Echo->ICMPv6EchoC;
	//   OpenSerialC.TriggerICMPv6Router->ICMPv6RouterC;
	//   OpenSerialC.TriggerICMPv6RPL->ICMPv6RPLC;
	//   OpenSerialC.TriggerOpenBridge->OpenBridgeC;
	//   OpenSerialC.PrintOpenSerial->OpenSerialC;
	//   OpenSerialC.PrintCellUsage->CellUsageC;
	//   OpenSerialC.PrintNeighbors->NeighborsC;
	//   OpenSerialC.PrintOpenQueue->OpenQueueC;
	//   OpenSerialC.PrintIEEE802154E->MACC;
	//   OpenSerialC.PrintMyDAGrank->RESC;
	////   OpenSerialC.PrintRPL->ForwardingC;
	//   OpenSerialC.PrintIDManager->IDManagerC;
}
