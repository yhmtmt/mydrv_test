#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include "zdmai.h"

#define DATA_ALIGN 3
#define DATA_SIZE (1 << DATA_ALIGN)
static unsigned int size_transfer = 0;
struct thdata{
  int * buf;
  int fd;
  int res;
};

void * tx_thread(void * ptr){
  struct thdata * d = (struct thdata*) ptr;
  //  printf("tx start.fd=%d size=%d\n", d->fd, sizeof(int) * size_transfer);
  d->res = write(d->fd, d->buf, sizeof(int) * size_transfer);
  //  printf("tx end.\n");
  return NULL;
}

void * rx_thread(void * ptr){
  struct thdata * d = (struct thdata*) ptr;
  // printf("rx start.fd=%d size=%d\n", d->fd, sizeof(int) * size_transfer);
  d->res = read(d->fd, d->buf, sizeof(int) * size_transfer);
  // printf("rx end.\n");
  return NULL;
}

void proc_dma(int argc, char ** argv, int fd)
{
    pthread_t tx, rx;
    struct thdata txd;
    struct thdata rxd;
    int i;
    double t;
    struct timeval s, f;
    if(argc != 3){
      printf("dma <bytes to transfer>\n");
      return;
    }
    size_transfer = (unsigned int) atoi(argv[2]);

    srand(size_transfer);

    unsigned char * tx_buf_ = (unsigned char*) 
      malloc(sizeof(int) * size_transfer + DATA_SIZE);
    txd.buf = (int*)(((unsigned long) (tx_buf_ + DATA_SIZE)  >> DATA_ALIGN) << DATA_ALIGN);
    unsigned char * rx_buf_ = (unsigned char*) 
      malloc(sizeof(int) * size_transfer + DATA_SIZE);
    rxd.buf = (int*)(((unsigned long) (rx_buf_ + DATA_SIZE) >> DATA_ALIGN) << DATA_ALIGN);
    txd.fd = rxd.fd = fd;

    for(i = 0; i < size_transfer; i++){
      txd.buf[i] = rand();
      rxd.buf[i] = 0;
    }

    printf("fd=%d size_transfer=%d\n", fd, sizeof(int) * size_transfer);
    gettimeofday(&s, NULL);
    for(i = 0; i < 100 ;i++){
      pthread_create(&tx, NULL, tx_thread, (void*) &txd);
      pthread_create(&rx, NULL, rx_thread, (void*) &rxd);
      
      pthread_join(tx, NULL);
      pthread_join(rx, NULL);
    }

    gettimeofday(&f, NULL);
    t = (f.tv_sec - s.tv_sec) * 1000 + (double) (f.tv_usec - s.tv_usec) / 1000.0;
    t /= 100;
    printf("transfer took %f ms (%f B/sec)\n", t, 
	   (double) (sizeof(int) * size_transfer) * 1000.0 / t);

    /*
    printf("Write start.\n");
    txd.res =  write(txd.fd, txd.buf, sizeof(int) * size_transfer);
    printf("Read start.\n");
    rxd.res =  read(rxd.fd, rxd.buf, sizeof(int) * size_transfer);
    */
    for(i = 0; i < size_transfer; i++){
      if(txd.buf[i] != rxd.buf[i]){
	printf("Wrong pattern. txd.buf[%d] = %d, rxd.buf[%d] = %d\n", i, 
	       txd.buf[i], i, rxd.buf[i]);
	break;
      }
    }

    if(i != size_transfer){
      printf("transfer failed.\n");
    }else{
      printf("trasfer successfully completed.\n");
    }
    free(tx_buf_);
    free(rx_buf_);
}

void proc_dmaw(int argc, char ** argv, int fd)
{
   struct thdata txd;
    int i;
    if(argc != 3){
      printf("dma <bytes to transfer>\n");
      return;
    }
    size_transfer = (unsigned int) atoi(argv[2]);
    
    srand(size_transfer);
    size_t sz = sizeof(int) * size_transfer + DATA_SIZE;    
    unsigned char * tx_buf_ = (unsigned char*) 
      malloc(sizeof(int) * size_transfer + DATA_SIZE);
    txd.buf = (int*)(((unsigned long) (tx_buf_ + DATA_SIZE)  >> DATA_ALIGN) << DATA_ALIGN);
    txd.fd = fd;
    
    printf("pbuf_=%08x pbuf_off=%08x\n",
	   (unsigned long) tx_buf_, (unsigned long) txd.buf);

    for(i = 0; i < size_transfer; i++){
      txd.buf[i] = i;
    }
    
    printf("fd=%d size_transfer=%d\n", fd, sizeof(int) * size_transfer);
    printf("Write start.\n");
    txd.res =  write(txd.fd, txd.buf, sizeof(int) * size_transfer);
    printf("Write done. %d bytes\n", txd.res);

    /*    
    for(i = 0; i < size_transfer; i++){
      printf("%08x ", txd.buf[i]);
    }
    */
    /*
    for(i = 0; i < txd.res; i++){
      printf("[%d]%02x ", i, tx_buf_[i]);
    }
    */
    if(txd.res !=  sizeof(int) * size_transfer){
      printf("transfer failed.\n");
    }else{
      printf("trasfer successfully completed.\n");
    }
    free(tx_buf_);
}

void proc_dmar(int argc, char ** argv, int fd)
{
  struct thdata rxd;
  int i;
  if(argc != 3){
    printf("dma <bytes to transfer>\n");
    return;
  }
  size_transfer = (unsigned int) atoi(argv[2]);

  srand(size_transfer);
  size_t sz = sizeof(int) * size_transfer + DATA_SIZE;
  unsigned char * rx_buf_ = (unsigned char*) 
    malloc(sizeof(int) * size_transfer + DATA_SIZE);
  rxd.buf = (int*)(((unsigned long) (rx_buf_ + DATA_SIZE) >> DATA_ALIGN) << DATA_ALIGN);
  printf("pbuf_=%08x size=%d pbuf_off=%08x\n",
	 (unsigned long) rx_buf_, sz, (unsigned long) rxd.buf);
  
  rxd.fd = fd;
  
  for(i = 0; i < size_transfer; i++){
    rxd.buf[i] = 0;
  }
  
  for(i = 0; i < sz; i++){
    printf("[%d]%02x ", i, rx_buf_[i]);
  }
  printf("\n");
  
  printf("fd=%d size_transfer=%d\n", fd, sizeof(int) * size_transfer);
  printf("Read start.\n");
  rxd.res =  read(rxd.fd, rxd.buf, sizeof(int) * size_transfer);
  printf("Read done. %d bytes\n", rxd.res);
  
  for(i = 0; i < sz; i++){
    printf("[%d]%02x ", i, rx_buf_[i]);
  }
  printf("\n");
  
  free(rx_buf_);
}

void imdmaw(int argc, char ** argv, int fd)
{
  int hsize, vsize, psize, dscsize;
  int num_trans;
  int last;
  int tot_size;
  int res;
  int i;
  unsigned char * buf;

  if(argc != 6){
    printf("imdmaw <hsize> <vsize> <psize> <dscsize>\n");
    return;
  }

  hsize = atoi(argv[2]);
  vsize = atoi(argv[3]);
  psize = atoi(argv[4]);
  dscsize = atoi(argv[5]);

  tot_size = hsize * vsize * psize;
  num_trans = tot_size / dscsize;
  last = tot_size % dscsize;
  printf("%dx%d pixel %dbyte/pixel %d total bytes divided into %d write operation\n", hsize, vsize, psize, tot_size, num_trans + (last ? 1 : 0));

  buf = NULL;
  res = posix_memalign((void**)&buf, DATA_SIZE, dscsize);

  if(res != 0){
    printf("Failed to allocate buffer.\n");
    return;
  }

  // preparing data
  for(i = 0; i < dscsize; i++){
    buf[i] = (unsigned char)(i % 0xFF);
  }

  for(i = 0; i < num_trans; i++){
    printf("%d th write operation ... ", i);
    res = write(fd, buf, dscsize);
    printf(" done. %d bytes transfered.\n", res);
    if(res <= 0){
      printf("Failed to transfer data. ret = %d\n", res);
      return;
    }
  }

  if(last){
    printf("last write operation ... ");
    res = write(fd, buf, last);
    printf(" done. %d bytes transfered.\n", res);
    if(res <= 0){
      printf("Failed to transfer data. ret = %d\n", res);
      return;
    }
  }
}

void imdmar(int argc, char ** argv, int fd)
{
  int hsize, vsize, psize, dscsize;
  int num_trans;
  int last;
  int tot_size;
  int res;
  int i, j;
  unsigned char * buf;

  if(argc != 6){
    printf("imdmar <hsize> <vsize> <psize> <dscsize>\n");
    return;
  }
  hsize = atoi(argv[2]);
  vsize = atoi(argv[3]);
  psize = atoi(argv[4]);
  dscsize = atoi(argv[5]);

  tot_size = hsize * vsize * psize;
  num_trans = tot_size / dscsize;
  last = tot_size % dscsize;
  printf("%d x %d pixel %d byte/pixel %d total bytes divided into %d read operation\n", hsize, vsize, psize, tot_size, num_trans + (last ? 1 : 0));
  buf = NULL;
  res = posix_memalign((void**)&buf, DATA_SIZE, dscsize);

  if(res != 0){
    printf("Failed to allocate buffer.\n");
    return;
  }

  for(i = 0; i < num_trans; i++){
    printf("%d th read operation ... ", i);
    res = read(fd, buf, dscsize);
    printf(" done. %d bytes transfered.\n", res);
    if(res <= 0){
      printf("Failed to receive data. ret=%d \n", res);
      return;
    }
    for(j = 0; j < dscsize; j++){
      if(buf[j] != (unsigned char)(j % 0xFF)){
	printf("In transfer %d, %dth byte, error happened. %02x != %02x\n",
	       i, j, buf[j], (unsigned char)(j % 0xFF));
      }
    }
  }

  if(last){
    printf("last th read operation ... ");
    res = read(fd, buf, last);
    printf(" done. %d bytes transfered.\n", res);
    if(res <= 0){
      printf("Failed to receive data. ret=%d \n", res);
      return;
    }
    for(j = 0; j < last; j++){
      if(buf[j] != (unsigned char)(j % 0xFF)){
	printf("In last transfer, %dth byte, error happened. %02x != %02x\n",
	       j, buf[j], (unsigned char)(j % 0xFF));
      }
    }
  }
}


int main(int argc, char ** argv){
  int fd;
 
  fd = open("/dev/zdmai0", O_RDWR);
  if(argc < 2){
    printf("zdmai_test <operation> [<hex value>]\n");
    return 0;
  }
  if(strcmp(argv[1], "dma") == 0){
    proc_dma(argc, argv, fd);
  }else if(strcmp(argv[1], "dmaw") == 0){
    proc_dmaw(argc, argv, fd);
  }else if(strcmp(argv[1], "dmar") == 0){
    proc_dmar(argc, argv, fd);
  }else if(strcmp(argv[1], "imdmar") == 0){
    imdmar(argc,argv, fd);
  }else if(strcmp(argv[1], "imdmaw") == 0){
    imdmaw(argc, argv, fd);
  }else{
    printf("Error: unknown operation %s.", argv[2]);
    return 1;
  }
  
  return 0;
}
