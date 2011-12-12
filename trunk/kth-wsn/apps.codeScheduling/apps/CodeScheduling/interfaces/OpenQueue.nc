#include "LATIN.h"

interface OpenQueue {
	async command OpenQueueEntry_t* inQueueBySeed(uint8_t seedNumber);
	async command OpenQueueEntry_t* inQueueToChild(uint8_t seedNumber);
	command error_t removeAllPacketsToNeighbor(open_addr_t* neighbor);
}
