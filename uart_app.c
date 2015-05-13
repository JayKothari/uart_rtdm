/****************************************
 * Quick test of I2C routines
 * Temp/pressure measuring using BMP085
 ****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>	/* for MCL_CURRENT and MCL_FUTURE */
#include <native/task.h>
#include <rtdm/rtdm.h>
//#include "i2cfunc.h"

// we will use I2C2 which is enumerated as 1 on the BBB
#define I2CBUS 1

// set to 1 to print out the intermediate calculations
#define DBG_PRINT 1
// set to 1 to use the example values in the datasheet
// rather than the values retrieved via i2c
#define ALG_TEST 1

#define DEVICE_NAME		"omap-i2c"
#define I2C_SLAVE		1 
RT_TASK 		rt_task_desc;

int uart_open(unsigned int addr)                                 //
{	
	int device,ret;
        device=rt_dev_open(DEVICE_NAME,0);
        if (device < 0) 
	{
                printf("ERROR : can't open device %s (%s)\n",
                       DEVICE_NAME, strerror(-device));
                exit(1);
        }
        ret=rt_dev_ioctl(device,I2C_SLAVE,(void *)addr);
        if(ret)
        {
                printf("Error in IOCTL\n");
        }
        return device;
}

int uart_write(int handle,unsigned char *buf,size_t size)
{
     if(rt_dev_write(handle,(void *)buf, size) != size)
                printf("write Error\n");
        return size;
}

int uart_write_byte(int handle,unsigned char val)
{
     if(rt_dev_write(handle,&val, 1) != 1)
                printf("write Error\n");
        return 1;
}

int uart_read(int handle,unsigned char *buf,size_t length)
{
  if(rt_dev_read(handle,(unsigned char *)buf,length) != length)
          printf("read Error\n");
  return length;
}

int uart_read_byte(int handle,unsigned char *val)
{
  if(rt_dev_read(handle,val,1) != 1)
          printf("read Error\n");
  return 1;
}

int uart_close(int device)
{
        int ret;
        ret = rt_dev_close(device);
        if (ret < 0)
        {
                printf("ERROR : can't close device %s (%s)\n",
                       DEVICE_NAME, strerror(-ret));
                exit(1);
        }
        return ret;
}

int main(void)
{ 
	  int ret;
	  int handle;
	  unsigned char buf[10];
	
	  ret = mlockall(MCL_CURRENT | MCL_FUTURE);
	  if (ret) 
	  {
		perror("ERROR : mlockall has failled");
		exit(1);
	  }

	/*
	 * Turn the current task into a RT-task.
	 * The task has no name to allow multiple program instances to be run
	 * at the same time.
	 */
	 ret = rt_task_shadow(&rt_task_desc, NULL, 1, 0);
	 if (ret)
  	{
		fprintf(stderr, "ERROR : rt_task_shadow: %s\n",strerror(-ret));
		exit(1);
  	}
    
  	handle=uart_open(0x47);
  	uart_write_byte(handle, 0xf6);
  	uart_read(handle, buf, 2);
	uart_close(handle);

  return(0);
}


