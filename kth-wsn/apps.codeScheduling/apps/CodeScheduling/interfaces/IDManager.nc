#include "LATIN.h"

interface IDManager {
   async command bool          getIsDAGroot();
   async command error_t       setIsDAGroot(bool newRole);
   async command bool          getIsBridge();
   async command error_t       setIsBridge(bool newRole);
   async command open_addr_t*  getMyID(uint8_t type);
   async command error_t       setMyID(open_addr_t* newID);
   async command bool          isMyAddress(open_addr_t* addr);
   command void 			   setDAGRoot(bool setIsDAGRoot);
}
