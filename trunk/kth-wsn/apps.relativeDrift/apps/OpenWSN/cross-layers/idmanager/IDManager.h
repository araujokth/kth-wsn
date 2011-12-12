#ifndef IDMANAGER_H
#define IDMANAGER_H

typedef struct debugIDManagerEntry_t {
   bool          isDAGroot;
   bool          isBridge;
   open_addr_t   my16bID;
   open_addr_t   my64bID;
   open_addr_t   myPANID;
   open_addr_t   myPrefix;
} debugIDManagerEntry_t;
enum{
	PAN_ID		= 0xAAAA,
};
#endif
