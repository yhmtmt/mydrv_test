
// ioctl command definition
#define MYDRV_IOC_MAGIC 'm'
#define MYDRV_IOCRESET _IO(MYDRV_IOC_MAGIC, 0)
#define MYDRV_IOCSET _IOW(MYDRV_IOC_MAGIC, 1, int)
#define MYDRV_IOCGET _IOR(MYDRV_IOC_MAGIC, 2, int)
#define MYDRV_IOGET2 _IOR(MYDRV_IOC_MAGIC, 3, int)
#define MYDRV_IOC_MAXNR 3
