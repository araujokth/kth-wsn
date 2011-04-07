#include <stdio.h>
#include <stdlib.h>

#include "sfsource.h"

int main(int argc, char **argv) {
	int fd;

	if (argc != 3) {
		fprintf(
				stderr,
				"Usage: %s <host> <port> - dump packets from a serial forwarder\n",
				argv[0]);
		exit(2);
	}
	fd = open_sf_source(argv[1], atoi(argv[2]));
	if (fd < 0) {
		fprintf(stderr, "Couldn't open serial forwarder at %s:%s\n", argv[1],
				argv[2]);
		exit(1);
	}
	for (;;) {
		int len, i;
		const unsigned char *packet = read_sf_packet(fd, &len);

		if (!packet)
			exit(0);
//    	printf("0x%02x ", packet[4]); 	// PanID
//
//    	printf("0x%04x ", (packet[6] << 8) | packet[5]); 	// PanID
//    	printf("0x%04x ", (packet[8] << 8) | packet[7]); 	// DstID
//    	printf("0x%04x ", (packet[10] << 8) | packet[9]);	// SrcID
//    	printf("%u ", packet[11] ); //Channel
//
//    	// Sniffer metadata
//    	printf("%u ", packet[20] );	// LQI
//    	printf("%d ", packet[21] );	// RSSI
//    	//MHRLen
//    	printf("%d ", (packet[26] << 24) | (packet[25] << 16) | (packet[24] << 8) | packet[23]);


    	      for (i = 0; i < len; i++)
    		printf("%02x ", packet[i]);

		putchar('\n');

		fflush(stdout);
		free((void *) packet);
	}
}
