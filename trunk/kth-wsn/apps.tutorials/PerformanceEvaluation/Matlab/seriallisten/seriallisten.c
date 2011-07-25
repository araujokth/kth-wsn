#include <stdio.h>
#include <stdlib.h>

#include "serialsource.h"

static char *msgs[] = { "unknown_packet_type", "ack_timeout", "sync",
		"too_long", "too_short", "bad_sync", "bad_crc", "closed", "no_memory",
		"unix_error" };

// packet structure
//			  | seq| pan  | dst  | src   | total | succe | delay |lqi|rssi| ch | timestamp
//02 17 61 88 | f4 | 34 12| 00 00| 01 00 | 00 70 | 00 70 | 84 a0 | 6c| e2 | 10 | 82 fb 17 00


static unsigned char headerLen = 11;
static unsigned char offsetChannel[] = { 0, 2, 4, 6, 7, 8, 9, 10 };
static char *fieldsChannel[] = { "pckTotal", "pckSuccess", "delay", "lqi",
		"rssi", "channel", "timestamp" };

static unsigned char offsetPck[] = { 4, 5, 7, 9 };
static char *fieldsPck[] = { "seqNum", "panId", "dstId", "srcId" };

void stderr_msg(serial_source_msg problem) {
	fprintf(stderr, "Note: %s\n", msgs[problem]);
}

int main(int argc, char **argv) {
	serial_source src;
	unsigned char * currentField;
	unsigned char i;

	if (argc != 3) {
		fprintf(
				stderr,
				"Usage: %s <device> <rate> - dump packets from a serial port\n",
				argv[0]);
		exit(2);
	}
	src = open_serial_source(argv[1], platform_baud_rate(argv[2]), 0,
			stderr_msg);
	if (!src) {
		fprintf(stderr, "Couldn't open serial port at %s:%s\n", argv[1],
				argv[2]);
		exit(1);
	}
	// print the name of the vars
	for (i = 0; i < 4; i++)
		printf("%s ", fieldsPck[i]);
	for (; i < 11; i++)
		printf("%s ", fieldsChannel[i - 4]);
	putchar('\n');

	for (;;) {
		int len, i;
		const unsigned char *packet = read_serial_packet(src, &len);
		const unsigned char *pPck;
		if (!packet)
			exit(0);

		currentField = offsetPck;

		printf("%u ", packet[*currentField]); //seq_num
		currentField++;

		printf("%u ", (packet[*currentField + 1] << 8) | packet[*currentField]); //pan
		currentField++;

		printf("%u ", (packet[*currentField + 1] << 8) | packet[*currentField]); //src
		currentField++;

		printf("%u ", (packet[*currentField + 1] << 8) | packet[*currentField]); //dst
		currentField++;

		currentField = offsetChannel;

		printf(
				"%u ",
				(packet[headerLen + *currentField] << 8) | packet[headerLen + *currentField + 1]); //success
		currentField++;

		printf(
				"%u ",
				(packet[headerLen + *currentField] << 8) | packet[headerLen + *currentField
						+ 1]); //total
		currentField++;

		printf(
				"%u ",
				(packet[headerLen + *currentField] << 8) | packet[headerLen + *currentField
						+ 1]); //delay
		currentField++;

		printf("%u ", packet[headerLen + *currentField]); //lqi
		currentField++;

		printf("%d ", (int8_t) packet[headerLen + *currentField]); //rssi
		currentField++;


		printf("%u ", packet[headerLen + *currentField]); //channel
		currentField++;

		printf(
				"%lu ",
				(packet[headerLen + *currentField + 3] << 24) | (packet[headerLen + *currentField + 2] << 16) |
				(packet[headerLen + *currentField + 1] << 8) | packet[headerLen + *currentField]);

		putchar('; \n');


		fflush(stdout);

		free((void *) packet);
	}
}
