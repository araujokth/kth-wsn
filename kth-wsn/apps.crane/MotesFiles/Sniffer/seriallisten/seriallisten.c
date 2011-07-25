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
      int len, i, offset = 24+4*4;//24;
      const unsigned char *packet = read_serial_packet(src, &len);

      if (!packet)
	exit(0);

  	printf("%d ", (packet[offset] << 8) | packet[offset+1]);
  	printf("%d ", (packet[offset+2] << 8) | packet[offset+3]);
  	printf("%d ", (packet[offset+4] << 24) | (packet[offset+5] << 16) | (packet[offset+6] << 8) | packet[offset+7]);
  	printf("%d ", (packet[offset+8] << 24) | (packet[offset+9] << 16) | (packet[offset+10] << 8) | packet[offset+11]);

      putchar('\n');
      free((void *)packet);
    }
}
