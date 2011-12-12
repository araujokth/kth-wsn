configuration PacketFunctionsC {
   provides interface PacketFunctions;
}
implementation {
   components PacketFunctionsP;
   PacketFunctions     = PacketFunctionsP.PacketFunctions;

//   components OpenSerialC;
//   PacketFunctionsP.OpenSerial->OpenSerialC;

   components IDManagerC;
   PacketFunctionsP.IDManager->IDManagerC;
}
