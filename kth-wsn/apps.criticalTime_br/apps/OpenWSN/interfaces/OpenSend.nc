#include "OpenWSN.h"

interface OpenSend {
   command error_t send    (OpenQueueEntry_t* msg);
   event   void    sendDone(OpenQueueEntry_t* msg, error_t error);
}
