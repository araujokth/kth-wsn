#include "OpenWSN.h"

interface OpenReceive {
  command void receive(OpenQueueEntry_t* msg);
}
