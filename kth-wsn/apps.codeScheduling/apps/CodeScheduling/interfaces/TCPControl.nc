#include "LATIN.h"

interface TCPControl {
   command error_t connect(open_addr_t* dest, uint16_t hisPort);
   event   void    connectDone(error_t error);
   command error_t close();
   event   bool    shouldIlisten();
}
