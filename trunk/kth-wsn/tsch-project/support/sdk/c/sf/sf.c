#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>

#include "sfsource.h"
#include "serialsource.h"

serial_source src;
int filefd;
int packets_read, packets_written, num_clients;
int srcId;

int unix_check(const char *msg, int result) {
	if (result < 0) {
		perror(msg);
		exit(2);
	}

	return result;
}

void *xmalloc(size_t s) {
	void *p = malloc(s);

	if (!p) {
		fprintf(stderr, "out of memory\n");
		exit(2);
	}
	return p;
}

void fd_wait(fd_set *fds, int *maxfd, int fd) {
	if (fd > *maxfd)
		*maxfd = fd;
	FD_SET(fd, fds);
}

void stderr_msg(serial_source_msg problem) {
	static char *msgs[] = { "unknown_packet_type", "ack_timeout", "sync",
			"too_long", "too_short", "bad_sync", "bad_crc", "closed",
			"no_memory", "unix_error" };

	fprintf(stderr, "Note: %s\n", msgs[problem]);
}

void open_serial(const char *dev, int baud) {
	char ldev[80];
	strcpy(ldev, dev);

	src = open_serial_source(ldev, baud, 1, stderr_msg);
	if (!src) {
		fprintf(stderr, "Couldn't open serial port at %s:%d\n", dev, baud);
		exit(1);
	}
}

void printf_packet(const unsigned char * packet, int len) {
	int i = 0;
	if (srcId != ((packet[10] << 8) | packet[9]) || srcId == -1) {
		srcId = (packet[10] << 8) | packet[9];

		printf("---------------------------------------\n");
	}

	printf("%lu ", (packet[26] << 24) | (packet[25] << 16) | (packet[24] << 8) | packet[23]); //Timestamp

	printf("%u ", packet[4]); // SeqNum
	printf("%u ", (packet[6] << 8) | packet[5]); // PanID
	printf("%u ", (packet[8] << 8) | packet[7]); // DstID
	printf("%u ", srcId); // SrctID

	printf("%u ", packet[11]); //Channel

	// Sniffer metadata
	printf("%u ", packet[20]); // LQI
	printf("%d", packet[21]); // RSSI
	//MHRLen

	putchar('\n');

	fflush(stdout);
}

void dispatch_packet(const void *packet, int len) {
	//if (write(filefd, packet, len) > 0)
	printf_packet((const unsigned char *) packet, len);
}

void check_serial(void) {
	int len;
	const unsigned char *packet = read_serial_packet(src, &len);

	if (packet) {
		packets_read++;
		dispatch_packet(packet, len);
		free((void *) packet);
	}
}

void open_file_fd(char *filename) {
	if ((filefd = open(filename, O_WRONLY | O_CREAT | O_TRUNC,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)) == -1) {
		perror("Cannot open output file\n");
		exit(1);
	}
}

// Attend the interruption
void sigproc(void) {
	close(filefd);
	exit(0);
}

int main(int argc, char **argv) {
	int serfd;

	if (argc != 4) {
		fprintf(stderr,
				"Usage: %s <device> <rate> <file> - act as a serial forwarder on <port>\n"
					"(listens to serial port <device> at baud rate <rate>)\n",
				argv[0]);
		exit(2);
	}

	if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
		fprintf(stderr, "Warning: failed to ignore SIGPIPE.\n");
	if (signal(SIGINT, sigproc) == SIG_ERR)
		fprintf(stderr, "Warning: failed to set the SIGINT interruption.\n");
	srcId = -1;

	open_serial(argv[1], platform_baud_rate(argv[2]));

	serfd = serial_source_fd(src);

	open_file_fd(argv[3]);

	dup2(filefd, 1);

	for (;;) {
		fd_set rfds;
		int maxfd = -1;
		struct timeval zero;
		int serial_empty;
		int ret;

		zero.tv_sec = zero.tv_usec = 0;

		FD_ZERO(&rfds);
		fd_wait(&rfds, &maxfd, serfd);

		serial_empty = serial_source_empty(src);
		check_serial();

		if (ret >= 0) {
			if (FD_ISSET(serfd, &rfds))
				check_serial();
		}
	}
}
