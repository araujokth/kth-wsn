#include <stdio.h>
#include <stdlib.h>

#include "serialsource.h"

static char *msgs[] = {
  "unknown_packet_type",
  "ack_timeout"	,
  "sync"	,
  "too_long"	,
  "too_short"	,
  "bad_sync"	,
  "bad_crc"	,
  "closed"	,
  "no_memory"	,
  "unix_error"
};

void stderr_msg(serial_source_msg problem)
{
  fprintf(stderr, "Note: %s\n", msgs[problem]);
}

int main(int argc, char **argv)
{
  serial_source src;

  if (argc != 3)
    {
      fprintf(stderr, "Usage: %s <device> <rate> - dump packets from a serial port\n", argv[0]);
      exit(2);
    }
  src = open_serial_source(argv[1], platform_baud_rate(argv[2]), 0, stderr_msg);
  if (!src)
    {
      fprintf(stderr, "Couldn't open serial port at %s:%s\n",
	      argv[1], argv[2]);
      exit(1);
    }
  for (;;)
    {
      int len, i;
      const unsigned char *packet = read_serial_packet(src, &len);

      if (!packet)
	exit(0);
        // MAC header + payload reader
    	printf("%04x ", (packet[5] << 8) | packet[6]); 	// PanID
    	printf("%04x ", (packet[7] << 8) | packet[8]); 	// DstID
    	printf("%04x ", (packet[9] << 8) | packet[10]);	// SrcID
    	printf("%02x ", packet[11] );

    	// Sniffer metadata
    	printf("%u ", packet[20] );	// LQI
    	printf("%d ", packet[21] );	// RSSI
    	//MHRLen
    	printf("%d ", (packet[23] << 24) | (packet[24] << 16) | (packet[25] << 8) | packet[26]);

//      for (i = 0; i < len; i++)
//	printf("%02x ", packet[i]);
      putchar('\n');
      free((void *)packet);
    }
}
