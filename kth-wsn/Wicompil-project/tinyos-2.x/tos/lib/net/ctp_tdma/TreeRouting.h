#ifndef _TREE_ROUTING_H
#define _TREE_ROUTING_H

enum {
    AM_TREE_ROUTING_CONTROL = 0xCE,
    BEACON_INTERVAL = 8192, 
    INVALID_ADDR  = TOS_BCAST_ADDR,
    ETX_THRESHOLD = 100, //50,      // link quality=20% -> ETX=5 -> Metric=50 
    PARENT_SWITCH_THRESHOLD = 15,
    MAX_METRIC = 0xFFFF,
    LOOP_DETECTION_THRESHOLD = 0, //20,
}; 
 

typedef struct {
  am_addr_t parent;
  uint16_t etx;
  //uint16_t lastHeardEtx;
  bool haveHeard;
  bool congested;
  bool healthy;  // A non healthy node is one, to which we cannot send data 
} route_info_t;

typedef struct {
  am_addr_t neighbor;
  route_info_t info;
} routing_table_entry;

inline void routeInfoInit(route_info_t *ri) {
    ri->parent = INVALID_ADDR;
    ri->etx = 0;
    //ri->lastHeardEtx = 0;
    ri->haveHeard = 0;
    ri->congested = FALSE;
    ri->healthy = TRUE;
}

typedef struct {
  am_addr_t child;   	 // The ID of that particular child
  bool hasTimeSlot;		 // If that child has a TimeSlot to transmit
  bool heardMB;			 // If modified beacon has been heard from the device
} child_info;

#endif
