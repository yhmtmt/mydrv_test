/*  mydrv.c  */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <asm/uaccess.h>

#include "mydrv.h"

/* Standard module information*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR
("Yohei Matsumoto - Tokyo Univ. Marine Science and Technology <yhmtmt@kaiyodai.ac.jp>");
MODULE_DESCRIPTION
("mydrv - test driver module for led and switch of zedboard connected by AXI_GPIO ");

#define DRIVER_NAME "mydrv"

// GPIO's TBUF state mask. initialized at probe.
unsigned int tbuf_mask = 0xffffff00;
int tbuf_addr = 0x04;
unsigned int irq_mask = 0x80000000;
int irq_addr = 0x11c;
unsigned int irq_en_mask = 0x00000001;
int irq_en_addr = 0x128;
unsigned int irq_st_mask = 0x00000001;
int irq_st_addr = 0x120;

int mydrv_major = 0; // major number (dynamically allocated in probe)
int mydrv_minor = 0; // minor number (zero fixed)
int mydrv_nr_devs = 1; // only one device node is supported.

////////////////////////////////////////////////////////// file operation override
ssize_t mydrv_read(struct file * filp, char __user * buf, size_t count,
		   loff_t * f_pos);
ssize_t mydrv_write(struct file * filp, const char __user * buf, size_t count,
		    loff_t * f_pos);

long mydrv_ioctl(struct file * filp, unsigned int cmd, unsigned long arg);

loff_t mydrv_llseek(struct file * filp, loff_t off, int whence);
int mydrv_open(struct inode * inode, struct file * filp);
int mydrv_release(struct inode * inode , struct file * filp);

// file operation object
struct file_operations mydrv_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = mydrv_ioctl,
  .read = mydrv_read,
  .write = mydrv_write,
  .llseek = mydrv_llseek,
  .open = mydrv_open,
  .release = mydrv_release,
};

module_param( tbuf_mask , int , S_IRUGO );
module_param( tbuf_addr , int , S_IRUGO );
module_param( irq_mask, int , S_IRUGO);
module_param( irq_addr, int , S_IRUGO);
module_param( irq_en_mask, int , S_IRUGO);
module_param( irq_en_addr, int , S_IRUGO);
module_param( irq_st_mask, int , S_IRUGO);
module_param( irq_st_addr, int , S_IRUGO);

struct mydrv_local {
  int irq;
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
  struct cdev cdev;
};

////////////////////////////////////////////////// fop implementation
ssize_t mydrv_read(struct file * filp, char __user * buf, size_t count,
		   loff_t * f_pos)
{
  ssize_t retval = 0;
  return retval;
}

ssize_t mydrv_write(struct file * filp, const char __user * buf, size_t count,
		    loff_t * f_pos)
{
  ssize_t retval = -ENOMEM;
  return retval;
}

long mydrv_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  int err = 0;
  int retval = 0;
  unsigned int val;
  struct mydrv_local * lp = filp->private_data;

  if (_IOC_TYPE(cmd) != MYDRV_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > MYDRV_IOC_MAXNR) return -ENOTTY;
  
  if(_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user*) arg, _IOC_SIZE(cmd));
  else if(_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
  if(err) return -EFAULT;
  
  switch(cmd){
  case MYDRV_IOCRESET:
    iowrite32(tbuf_mask, lp->base_addr);
    break;
  case MYDRV_IOCSET:
    retval = __get_user(val, (unsigned int __user *) arg);
    iowrite32(val, lp->base_addr);
    break;
  case MYDRV_IOCGET:
    val = ioread32(lp->base_addr);
    retval = __put_user(val, (unsigned int __user*) arg);
    break;
  }
  return retval;
}

loff_t mydrv_llseek(struct file * filp, loff_t off, int whence)
{
  loff_t newpos = off;
  return newpos;
}

int mydrv_open(struct inode * inode, struct file * filp)
{
  struct mydrv_local * lp;
 
  lp = container_of(inode->i_cdev, struct mydrv_local, cdev);
  filp->private_data = lp;
  return 0;
}


int mydrv_release(struct inode * inode , struct file * filp)
{
  return 0;
}

////////////////////////////////////////////////// fop end.

////////////////////////////////////////////////// platform driver functions
static irqreturn_t mydrv_irq(int irq, void *lp)
{
  printk(KERN_INFO "mydrv interrupt\n");
  iowrite32(irq_st_mask, ((struct mydrv_local *)lp)->base_addr + irq_st_addr);

  return IRQ_HANDLED;
}

static int __devinit mydrv_probe(struct platform_device *pdev)
{
  struct resource *r_irq; /* Interrupt resources */
  struct resource *r_mem; /* IO mem resources */
  struct device *dev = &pdev->dev;
  struct mydrv_local *lp = NULL;
  dev_t devno = 0;
  int rc = 0;
  
  dev_info(dev, "Device Tree Probing\n");
  
  /* Get iospace for the device */
  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) {
    dev_err(dev, "invalid address\n");
    return -ENODEV;
  }
  
  lp = (struct mydrv_local *) kmalloc(sizeof(struct mydrv_local), GFP_KERNEL);
  if (!lp) {
    dev_err(dev, "Cound not allocate mydrv device\n");
    return -ENOMEM;
  }
	
  dev_set_drvdata(dev, lp);
  
  lp->mem_start = r_mem->start;
  lp->mem_end = r_mem->end;
  
  if (!request_mem_region(lp->mem_start,
			  lp->mem_end - lp->mem_start + 1,
			  DRIVER_NAME)) {
    dev_err(dev, "Couldn't lock memory region at %p\n",
	    (void *)lp->mem_start);
    rc = -EBUSY;
    goto error1;
  }
  
  lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);
  if (!lp->base_addr) {
    dev_err(dev, "mydrv: Could not allocate iomem\n");
    rc = -EIO;
    goto error2;
  }
  
  /* Get IRQ for the device */
  r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
  if (!r_irq) {
    dev_info(dev, "no IRQ found\n");
    dev_info(dev, "mydrv at 0x%08x mapped to 0x%08x\n",
	     (unsigned int __force)lp->mem_start,
	     (unsigned int __force)lp->base_addr);
    return 0;
  } 
  lp->irq = r_irq->start;
  
  rc = request_irq(lp->irq, &mydrv_irq, 0, DRIVER_NAME, lp);
  if (rc) {
    dev_err(dev, "testmodule: Could not allocate interrupt %d.\n",
	    lp->irq);
    goto error3;
  }
  dev_info(dev,"mydrv at 0x%08x mapped to 0x%08x, irq=%d\n",
	   (unsigned int __force)lp->mem_start,
	   (unsigned int __force)lp->base_addr,
	   lp->irq);
  
  // initialization for char device
  if(mydrv_major){
    devno = MKDEV(mydrv_major, mydrv_minor);
    rc = register_chrdev_region(devno, mydrv_nr_devs, DRIVER_NAME);
  }else{
    rc = alloc_chrdev_region(&devno, mydrv_minor, mydrv_nr_devs, DRIVER_NAME);
    mydrv_major = MAJOR(devno);
  }
  
  dev_info(dev, "mydrv allocate cdev %d %d", mydrv_major, mydrv_minor);
  
  if(rc < 0){
    printk(KERN_WARNING "%s: can't get major %d\n", DRIVER_NAME, mydrv_major);
    return rc;
  }
  
  // determining io direction by applying tbuf_mask. 
  iowrite32(tbuf_mask, lp->base_addr + tbuf_addr);
  //global irq enabling
  iowrite32(irq_mask, lp->base_addr + irq_addr);
  iowrite32(irq_en_mask, lp->base_addr + irq_en_addr);
  
  cdev_init(&lp->cdev, &mydrv_fops);
  lp->cdev.owner = THIS_MODULE;
  rc = cdev_add(&lp->cdev, devno, 1);
  if(rc)
    printk(KERN_NOTICE "Error %d adding %s%d", rc, DRIVER_NAME, 0);
  
  return 0;
 error3:
  free_irq(lp->irq, lp);
 error2:
  release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
 error1:
    kfree(lp);
    dev_set_drvdata(dev, NULL);
    return rc;
}

static int __devexit mydrv_remove(struct platform_device *pdev)
{
  struct device *dev = &pdev->dev;
  struct mydrv_local *lp = dev_get_drvdata(dev);

  dev_t devno = MKDEV(mydrv_major, mydrv_minor);
  cdev_del(&lp->cdev);
  unregister_chrdev_region(devno, mydrv_nr_devs);

  free_irq(lp->irq, lp);

  release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
  kfree(lp);
  dev_set_drvdata(dev, NULL);
  return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mydrv_of_match[] __devinitdata = {
  { .compatible = "xlnx,axi-gpio-1.01.b", },
  { .compatible = "xlnx,xps-gpio-1.00.a", },
  { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, mydrv_of_match);
#else
# define mydrv_of_match
#endif


static struct platform_driver mydrv_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table	= mydrv_of_match,
  },
  .probe		= mydrv_probe,
  .remove		= __devexit_p(mydrv_remove),
};

static int __init mydrv_init(void)
{
  printk(KERN_INFO "start mydrv.");
 
  return platform_driver_register(&mydrv_driver);
}


static void __exit mydrv_exit(void)
{
  platform_driver_unregister(&mydrv_driver);
  printk(KERN_INFO "end mydrv.\n");
}

module_init(mydrv_init);
module_exit(mydrv_exit);

