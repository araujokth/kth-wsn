#include "LATIN.h"
#include "IPHC.h" //needed for declaration of ipv6_header_iht

interface OpenReceiveIp {
  command void receive(OpenQueueEntry_t* msg, ipv6_header_iht ipv6_header);
}
