/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#include <getopt.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pthread.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BUF_SIZE 1024

volatile sig_atomic_t eflag = 0;
void handler(int signo){
  eflag = 1;
}

#define XBEE_TX64 0x00
#define XBEE_RCMD 0x07
#define XBEE_AT 0x08
#define XBEE_ATQ 0x09
#define XBEE_ZB_TRN 0x10
#define XBEE_ZB_TRN_EX 0x11
#define XBEE_ZB_ATR  0x17
#define XBEE_TX_IP4 0x20
#define XBEE_PUT_RQ 0x28
#define XBEE_DEV_RS 0x2A
#define XBEE_RX64 0x80
#define XBEE_RCMD_RS 0x87
#define XBEE_AT_RS 0x88
#define XBEE_TX_ST 0x89
#define XBEE_MDM_ST 0x8A
#define XBEE_ZB_TX_ST 0x8B
#define XBEE_IOD_RX 0x8F
#define XBEE_ZB_RCV 0x90
#define XBEE_ZB_RCV_EX 0x91
#define XBEE_ZB_ATR_RS 0x97
#define XBEE_RX_IP4 0xB0
#define XBEE_PUT_RS 0xB8
#define XBEE_DEV_RQ 0xB9
#define XBEE_DEV_RS_ST 0xBA
#define XBEE_FRM_ERR 0xFE

int wrTX64(uint8_t * buf, size_t len_buf,uint8_t fid, 
	   unsigned int addr, bool back, char * data, size_t len_data)
{
  unsigned short len = 11 + len_data;
  uint8_t cs = 0;

  if(len_buf - 5 < len_data)
    return -1;

  buf[0] = 0x7E;
  buf[1] = ((uint8_t*) &len)[1];
  buf[2] = ((uint8_t*) &len)[2];
  buf[3] = XBEE_TX64;
  buf[4] = fid;
  buf[5] = buf[6] = buf[7] = buf[8] = 0;
  buf[9] = ((uint8_t*) &addr)[3];
  buf[10] = ((uint8_t*) &addr)[2];  
  buf[11] = ((uint8_t*) &addr)[1];
  buf[12] = ((uint8_t*) &addr)[0];
  
  memcpy(&buf[13], data, sizeof(uint8_t) * len_data);
  len += 3;

  for(int i = 3; i < len; i++){
    cs += buf[i];
  }
  buf[len] = 0xFF - cs;

  return len;
}

int wrAT(uint8_t * buf, size_t len_buf, uint8_t fid, 
	 unsigned int addr, bool back, char * data, size_t len_data)
{
  unsigned short len;
  uint8_t cs = 0;

  buf[0] = 0x7E;

  buf[3] = XBEE_AT;
  buf[4] = fid;
  
}

static const char *device = "/dev/spidev32766.0";
static uint8_t mode;
struct spi_ioc_transfer tr = {
  .tx_buf = (unsigned long) NULL,
  .rx_buf = (unsigned long) NULL,
  .len = BUF_SIZE,
  .delay_usecs = 0,
  .speed_hz = 3500000,
  .bits_per_word = 8
};

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static int btr = 0;
pthread_mutex_t mtx_tr;
pthread_t th_write;
pthread_t th_read;
pthread_cond_t cond;

static struct timespec ts = {
  .tv_sec = 0,
  .tv_nsec = 10000
};

void * func_write(void * fd){
  int count;
  unsigned char c;
  unsigned char buf[BUF_SIZE];

  count = 0;
  while(!eflag){
    c = getc(stdin);
    
    if(c == 0x0D || c == 0x0A){
      pthread_mutex_lock(&mtx_tr);
      memcpy((void*)tr.tx_buf, (void*) buf, sizeof(unsigned char) * count);
      pthread_cond_wait(&cond, &mtx_tr);
      pthread_mutex_unlock(&mtx_tr);
      count = 0;
    }else{
      count++;
    }
  }
  return NULL;
}

void * func_read(void * fd){
  unsigned char * buf;

  while(!eflag){
    pthread_mutex_lock(&mtx_tr);
    pthread_cond_wait(&cond, &mtx_tr);
    buf = (unsigned char *) tr.rx_buf;
    for(; *buf != 0x0D || *buf !=0x00 || *buf!= 0x0A; buf++);
    if(*buf == 0x0D || *buf == 0x0A)
      *buf = '\0';
    printf("%s", (unsigned char*) tr.rx_buf);
    pthread_mutex_unlock(&mtx_tr);
  }

  return NULL;
}

/*
static void transfer(int fd)
{
	int ret;
	uint8_t tx[] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
		0xF0, 0x0D,
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
}
*/

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			tr.speed_hz = atoi(optarg);
			break;
		case 'd':
			tr.delay_usecs = atoi(optarg);
			break;
		case 'b':
			tr.bits_per_word = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}


int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	signal(SIGINT, handler);

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &tr.bits_per_word);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &tr.bits_per_word);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &tr.speed_hz);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &tr.speed_hz);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", tr.bits_per_word);
	printf("max speed: %d Hz (%d KHz)\n", tr.speed_hz, tr.speed_hz/1000);

	tr.tx_buf = (unsigned long) malloc(sizeof(uint8_t) * tr.len);
	tr.rx_buf = (unsigned long) malloc(sizeof(uint8_t) * tr.len);
	pthread_mutex_init(&mtx_tr, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_create(&th_write, NULL, func_write, NULL);
	pthread_create(&th_read, NULL, func_write, NULL);
	while(!eflag){
	  nanosleep(&ts, NULL);
	  pthread_mutex_lock(&mtx_tr);
	  if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
		pabort("can't send spi message");
	  btr = 1;

	  pthread_mutex_unlock(&mtx_tr);
	  pthread_cond_broadcast(&cond);
	}
	pthread_join(th_write, NULL);
	pthread_join(th_read, NULL);
	pthread_mutex_destroy(&mtx_tr);
	free((void*) tr.tx_buf);
	free((void*) tr.rx_buf);

	close(fd);

	return ret;
}
