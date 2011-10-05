#include <stdio.h>
#include <stdlib.h>

#include "serialsource.h"

static char *msgs[] = { "unknown_packet_type", "ack_timeout", "sync",
		"too_long", "too_short", "bad_sync", "bad_crc", "closed", "no_memory",
		"unix_error" };

// 02 11 61 88 88 34 12 00 00 02 00 00 00 00 60 00 60 00 fe 6b cb 00 76 32 9e 15


//typedef nx_struct PerformanceParams {
//
//	nx_uint16_t pckTotal;
//	nx_uint16_t pckSuccess;
//	nx_uint16_t delay;
//
//}PerformanceParams;

//02 12 61 88 ee 34 12 02 00 00 00 00 00 01 00 16 00 16 06 9e 6c ed 4f 84 | 28 12 b5 04
//typedef nx_struct ActuationValuesMsg {
//	nx_int16_t u; //actuation
//	nx_uint8_t wtId;
//
//	PerformanceParams performValues;
//
//
//}ActuationValuesMsg;

//				| pan |	 | src | dst |					   | total | succe | delay |lqi|rssi|mhr | ch | timestamp
//02 17 61 88 f4 34 12 00 00 01 00 04 0b 05 9f b5 38 91 46 | 00 70 | 00 70 | 84 a0 | 6c| e2 | 05 | 9f | 82 fb 17 00
//typedef struct WTSensorValuesMsg {
//	nx_uint16_t tankLevel[NUMBER_TANKS]; //value for the tk1 and tk2
//	nx_float integrator;
//
//	PerformanceParams performValues;
//} WTSensorValuesMsg;

static unsigned char offsetChannel[] = {14, 12, 10, 8, 7, 6, 5, 4 };
static char *fieldsChannel[] = { "pckTotal", "pckSuccess", "delay", "lqi", "rssi", "mhrLen", "channel", "timestamp"};

static unsigned char offsetPck[] = {4, 5, 7, 9};
static char *fieldsPck[] = { "seqNum", "panId", "dstId", "srcId"};


void stderr_msg(serial_source_msg problem) {
	fprintf(stderr, "Note: %s\n", msgs[problem]);
}

int main(int argc, char **argv) {
	serial_source src;
	unsigned char * currentField;
	unsigned char i;

	if (argc != 4) {
		fprintf(
				stderr,
				"Usage: %s <device> <rate> <file>- dump packets from a serial port\n",
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
	// open file to store
	fd = open(argv[3], O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
	dup2(fd, 1);

	// print the name of the vars
	for (i = 0 ; i < 4 ; i++) printf("%s ", fieldsPck[i]);
	for (; i < 12 ; i++) printf("%s ", fieldsChannel[i-4]);
	putchar('\n');

	for (;;) {
		int len, i;
		const unsigned char *packet = read_serial_packet(src, &len);

		if (!packet)
			exit(0);

		currentField = offsetPck;

		printf("%u ", packet[*currentField]); //seq_num
		currentField++;

		printf("%u ",
				(packet[*currentField + 1] << 8) | packet[*currentField]); //pan
		currentField++;

		printf("%u ",
				(packet[*currentField + 1] << 8) | packet[*currentField]); //src
		currentField++;

		printf("%u ",
				(packet[*currentField + 1] << 8) | packet[*currentField]); //dst
		currentField++;

		currentField = offsetChannel;

		printf("%u ",
				(packet[len - *currentField] << 8) | packet[len - *currentField  + 1]); //success
		currentField++;

		printf("%u ",
				(packet[len - *currentField] << 8) | packet[len - *currentField  + 1]); //total
		currentField++;

		printf("%u ",
				(packet[len - *currentField] << 8) | packet[len - *currentField + 1]); //delay
		currentField++;

		printf("%u ", packet[len - *currentField]); //lqi
		currentField++;

		printf("%d ",packet[len - *currentField]); //rssi
		currentField++;

		//mhrLen
		printf("%u ", packet[*currentField]); //mhrLen
		currentField++;

		printf("%u ", packet[len - *currentField]); //channel
		currentField++;

		printf("%lu ", (packet[len - *currentField +3] << 24) | (packet[len - *currentField + 2] << 16) | (packet[len - *currentField +1] << 8) | packet[len - *currentField]);

		putchar('; \n');


//		for (i = 0; i < len; i++)
//			printf("%02x ", packet[i]);
//		putchar('\n');
		free((void *) packet);
	}
}
