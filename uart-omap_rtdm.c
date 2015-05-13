#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
//#include <linux/of_i2c.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/i2c-omap.h>
#include<linux/irq.h>
#include <linux/pinctrl/consumer.h>

#include <rtdm/rtdm_driver.h>
#include<rtdm/rtdm.h>

#define DEVICE_NAME_1	"omap_uart"
#define DRIVER_NAME_1	"omap_uart"

#include <linux/init.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/platform_data/serial-omap.h>
#include<linux/kernel.h>

#define OMAP_MAX_HSUART_PORTS	6

#define UART_BUILD_REVISION(x, y)	(((x) << 8) | (y))

#define OMAP_UART_REV_42 0x0402
#define OMAP_UART_REV_46 0x0406
#define OMAP_UART_REV_52 0x0502
#define OMAP_UART_REV_63 0x0603

#define UART_ERRATA_i202_MDR1_ACCESS	BIT(0)
#define UART_ERRATA_i291_DMA_FORCEIDLE	BIT(1)

#define DEFAULT_CLK_SPEED 48000000 /* 48Mhz*/

/* SCR register bitmasks */
#define OMAP_UART_SCR_RX_TRIG_GRANU1_MASK		(1 << 7)
#define OMAP_UART_SCR_TX_EMPTY			(1 << 3)

/* FCR register bitmasks */
#define OMAP_UART_FCR_RX_FIFO_TRIG_MASK			(0x3 << 6)
#define OMAP_UART_FCR_TX_FIFO_TRIG_MASK			(0x3 << 4)

/* MVR register bitmasks */
#define OMAP_UART_MVR_SCHEME_SHIFT	30

#define OMAP_UART_LEGACY_MVR_MAJ_MASK	0xf0
#define OMAP_UART_LEGACY_MVR_MAJ_SHIFT	4
#define OMAP_UART_LEGACY_MVR_MIN_MASK	0x0f

#define OMAP_UART_MVR_MAJ_MASK		0x700
#define OMAP_UART_MVR_MAJ_SHIFT		8
#define OMAP_UART_MVR_MIN_MASK		0x3f

#define OMAP_UART_DMA_CH_FREE	-1

#define MSR_SAVE_FLAGS		UART_MSR_ANY_DELTA
#define OMAP_MODE13X_SPEED	230400

/* WER = 0x7F
 * Enable module level wakeup in WER reg
 */
#define OMAP_UART_WER_MOD_WKUP	0X7F

/* Enable XON/XOFF flow control on output */
#define OMAP_UART_SW_TX		0x08

/* Enable XON/XOFF flow control on input */
#define OMAP_UART_SW_RX		0x02

#define OMAP_UART_SW_CLR	0xF0

#define OMAP_UART_TCR_TRIG	0x0F
 
#define SOC_PRCM_REGS                        (0x44E00000)
#define SOC_PRCM_SIZE                           (0x400 )
#define CM_PER_UART4_CLKCTRL 			(1 << 1)
 
 
#define BUFFER_SIZE		64

struct circ_buf_1 {
          char buf[BUFFER_SIZE];
          int head;
          int tail;
};

//	void  ringBufS_init  (ringBufS *_this);
//      int   ringBufS_empty (ringBufS *_this);
//      int   ringBufS_full  (ringBufS *_this);
//      int   ringBufS_get   (ringBufS *_this);
//      void  ringBufS_put   (ringBufS *_this, const unsigned char c);
//      void  ringBufS_flush (ringBufS *_this, const int clearBuffer);

typedef struct uart_omap_port {
	struct device		*dev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		fcr;
	unsigned char		efr;
	unsigned char		dll;
	unsigned char		dlh;
	unsigned char		mdr1;
	unsigned char		scr;

	int			use_dma;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
	unsigned char		msr_saved_flags;
	char			name[20];
	unsigned long		port_activity;
	int			context_loss_cnt;
	u32			errata;
	u8			wakeups_enabled;

	int			DTR_gpio;
	int			DTR_inverted;
	int			DTR_active;

	struct pm_qos_request	pm_qos_request;
	u32			latency;
	u32			calc_latency;
	struct work_struct	qos_work;
	struct pinctrl		*pins;


	
	unsigned int            irq;                    /* irq number */
	unsigned long           irqflags;               /* irq flags  */
	unsigned int            fifosize;               /* tx fifo size */
	unsigned char           regshift;               /* reg offset shift */
	unsigned int            line;                   /* port index */	
	resource_size_t         mapbase;                /* for ioremap */	
	unsigned char __iomem   *membase;               /* read/write[bwl] */
	upf_t                   flags;
	unsigned int            uartclk;                /* base uart clock */
	unsigned int            mctrl;
	unsigned int            read_status_mask;       /* driver specific */
	unsigned int            ignore_status_mask;     /* driver specific */

	u8                      *buf_tx;
	size_t			buf_len_tx;

	u8			*buf_rx;
	size_t			buf_len_rx;

	rtdm_irq_t              irq_handle;
        rtdm_event_t            w_event_tx;
	rtdm_event_t		w_event_rx;
        rtdm_lock_t             lock;
	struct rtdm_device      rtdm_dev;
	unsigned long 		timeout,systime,systime1;
	struct circ_buf_1		rbuf;
}MY_DEV;


void f_cir_buf(MY_DEV *up)
{
//up->rbuf=rtdm_malloc(sizeof(BUFFER_SIZE));
memset(up->rbuf.buf,0,BUFFER_SIZE);
up->rbuf.head = 0;
up->rbuf.tail= 0;
}

void write_buffer(MY_DEV *up, char data)
{
  unsigned int next = (unsigned int)(up->rbuf.head + 1) % BUFFER_SIZE;

  if (next != up->rbuf.tail)
  {
        up->rbuf.buf[up->rbuf.head] = data;
        up->rbuf.head = next;
  }
}

char read_buffer(MY_DEV *up)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (up->rbuf.head == up->rbuf.tail)
  {
  printk("buffer is empty\n");
  return -1;        // quit with an error
  }
  else
  {
    char data = up->rbuf.buf[up->rbuf.tail];
    up->rbuf.tail = (unsigned int)(up->rbuf.tail + 1) % BUFFER_SIZE;
    return data;
  }
}


static inline unsigned int serial_in(struct uart_omap_port *up, int offset)//read
{
	offset <<= up->regshift;
	return readw(up->membase + offset);
}

static inline void serial_out(struct uart_omap_port *up, int offset, int value)//write
{
	offset <<= up->regshift;
	writew(value, up->membase + offset);
}


static void serial_omap_mdr1_errataset(struct uart_omap_port *up, u8 mdr1)
{
         u8 timeout = 255;
 
	printk("serial_omap_mdr1_errataser start\n");
         serial_out(up, UART_OMAP_MDR1, mdr1);

	 rtdm_task_sleep(2000);
         serial_out(up, UART_FCR, up->fcr | UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR);
         /*
          * Wait for FIFO to empty: when empty, RX_FIFO_E bit is 0 and
          * TX_FIFO_E bit is 1.
          */
         while (UART_LSR_THRE != (serial_in(up, UART_LSR) & (UART_LSR_THRE | UART_LSR_DR))) 
	{
                 timeout--;
                 if (!timeout) 
		 {
                         /* Should *never* happen. we warn and carry on */
                         dev_crit(up->dev, "Errata i202: timedout %x\n",serial_in(up, UART_LSR));
                         break;
                 }
		rtdm_task_sleep(1000);
         }
	 printk("serial_omap_mdr1_errataser end\n");
}

static inline void serial_omap_clear_fifos(struct uart_omap_port *up)//clearing FIFO before trasmiting
{
	printk("..................................serial_omap_clear_fifos");
	//UART_FCR is selested if LCR[7]=0 FCR[5:4] can only be writtern if EFR[4]=1
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);//enable FIFO(can change only when baud rate is not running DLL and DLH cleared to 0)
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);//Clear RCVR FIFO and XMIT FIFO
	serial_out(up, UART_FCR, 0);//disable tx and rx fifo:nochange in other value
	
	 printk(".............................serial_omap_clear_fifos end");
}


static void serial_omap_rdi(struct uart_omap_port *up, unsigned int lsr)
{	
	printk("..................serial_omap_rdi\n");
	//dump_stack();
	u16	w;
	if (!(lsr & UART_LSR_DR))
	{	printk("Receive buffer is full\n");
		return;
	}
//	while(up->buf_len_rx--)
        {
                w = serial_in(up, UART_RX);
		printk("Receive buffer=%x\n",w);
//                *up->buf_rx++ = w;
		write_buffer(up,w);
//                up->buf_len_rx--;
        }
	printk("...............serial_omap_rdi\n");
}

static unsigned int serial_omap_get_divisor(struct uart_omap_port *port, unsigned int baud)
{
	unsigned int divisor;
	
	printk(".................serial_omap_get_divisor\n");

	if (baud > OMAP_MODE13X_SPEED && baud != 3000000)
	{	divisor = 13;
		printk("x13 baudrate\n");
	}
	else
	{
		divisor = 16;
		printk("x16 baudrate\n");
	}
	 printk("............serial_omap_get_divisor end\n");

	return port->uartclk/(baud * divisor);
}

static ssize_t uart_rd_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info, void *buf,size_t nbyte)
{
	int err;
	int ret=0;
	int count;
	 MY_DEV *up=(MY_DEV *)context->device->device_data;
	u8 *tmp;

	printk("..............uart_rd_rt start\n");
	tmp=(u8 *)rtdm_malloc(nbyte);
	up->buf_rx=(u8 *)tmp;
	up->buf_len_rx = nbyte;
	count =nbyte;

	if (!(up->ier & UART_IER_RDI))
        {
                up->ier |= UART_IER_RDI;
                serial_out(up, UART_IER, up->ier);
        }

	 err=rtdm_event_wait(&up->w_event_rx);
         if(err<0)
         {
                dev_err(up->dev,"controller timed out\n");
                rtdm_printk("rtdm_event_timedwait: timeout\n");
                return -ETIMEDOUT;
         }
         if(err==0)
         {
         ret=nbyte;
         }
	
	while(count--)
	{
	*tmp=read_buffer(up);
	printk("Receive rd=%x\n",*tmp);
	tmp = tmp +1;
	}
	tmp=tmp-nbyte;
	if(rtdm_safe_copy_to_user(user_info,buf,(void *)tmp, nbyte))
                rtdm_printk("ERROR : can't copy data from driver\n");

	printk("............uart_rd_rt end\n");

        rtdm_free(tmp);
	
	return ret;
}

static void serial_omap_stop_tx(struct uart_omap_port *up)
{
	printk(".......................serial_omap_stop_tx\n");
	
	//disable transmit holding register interrupt
	if (up->ier & UART_IER_THRI) 
	{    
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
	
	printk("..........................serial_omap_stop_tx end\n");
}	
		
static void transmit_chars(struct uart_omap_port *up, unsigned int lsr)
{	
	u16             w;
	
//trasmit holding register empty
	printk("trasmit_char.....start\n");

	if(!(lsr & UART_LSR_THRE))
	{	printk("Holding register is empty\n");
		return;
	}
	
//(void) serial_in(up, UART_LSR);
	
        while (up->buf_len_tx)
        {
//                w = *up->buf_tx++;
		w = read_buffer(up);
		printk("up->buf_len_tx--=%d\n",up->buf_len_tx);
                up->buf_len_tx--;
		
       //         rtdm_printk("BUFFER ADDRESS IN TRASMIT MODE=%x\n",up->buf);
                rtdm_printk("buffer value in trasmit_char=%x\n",w);
		serial_out(up, UART_TX, w);
        }
	

	if(up->buf_len_tx == 0)
	{
	serial_omap_stop_tx(up);
	}

	printk("trasmit_char......end\n");
}

static ssize_t uart_wr_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info,const void *buf, size_t nbyte)
{
	int ret=0;
	int err;	
	int count;
	char c;

	MY_DEV *up=(MY_DEV *)context->device->device_data;
	
	char *tmp;
	up->buf_len_tx = nbyte;
	
	printk("uart_wr_rt  start\n");
	tmp=rtdm_malloc(nbyte);
	
	if ((rtdm_safe_copy_from_user(user_info,tmp, buf, up->buf_len_tx)))
                rtdm_printk("ERROR : can't copy data to driver\n");
	

	 count=nbyte;
	while(count--)
	{
	write_buffer(up,*tmp);
	tmp=tmp+1;
	
//	up->buf_tx=(char *)tmp;
//	printk("up->buf_tx=%x\n",*up->buf_tx);
			
	//enable Trasmitter holding Register
	if (!(up->ier & UART_IER_THRI)) 
	{
		up->ier |= UART_IER_THRI;
		 up->systime = rtdm_clock_read();
		serial_out(up, UART_IER, up->ier);
	}

	}

	printk("Tx interrupt enable\n");
	printk("rtdm_event_wait before\n");

	err=rtdm_event_wait(&up->w_event_tx);
        if(err<0)
        {
                dev_err(up->dev,"controller timed out\n");
                rtdm_printk("rtdm_event_timedwait: timeout\n");
                return -ETIMEDOUT;
        }

	up->systime1 = rtdm_clock_read();

	up->timeout=(up->systime1)-(up->systime);

	printk("scheduling latency=%ld\n",up->timeout);

	if(err==0)
	{
	 	ret=nbyte;
	}
	printk("rtdm_event_wait after\n");
	printk("uart_wr_rt end\n");
	rtdm_free(tmp);
	return ret;
}

#define CS5_1		 0
#define CS6_1		 1
#define CS7_1		 2		
#define CS8_1		 3

#define BAUD_4800	 0
#define BAUD_9600	 1
#define BAUD_115200	 2

static void serial_omap_set_termios(MY_DEV *up, unsigned int request)
{
	int val;
	unsigned char cval = 0;
	unsigned int baud, quot;
	rtdm_lockctx_t context1;
	int err;

	printk("serial_omap_set_termios\n");
	printk("Local struct up=%x\n",up);

	printk("request=%x\n",request);
	val=request & 0x03;
	printk("val=%x",val);
	switch(val)
	{
		case CS5_1:
			printk("CS5\n");
			cval = UART_LCR_WLEN5;
			break;
		
		case CS6_1:
			printk("CS6\n");
			cval = UART_LCR_WLEN6;
			break;
		
		case CS7_1:
			printk("CS7\n");
			cval = UART_LCR_WLEN7;
			break;
		default:
		case CS8_1:
			printk("CS8\n");
			cval = UART_LCR_WLEN8;
			break;
	}
	
	if(request & 0x04)
	{	printk("set two stop bits\n");
		cval |= UART_LCR_STOP;
	}

	if(request & 0x08)
	{	printk("set even patity\n");
		cval |= UART_LCR_PARITY;
	}

	if(request & 0x10)
	{	printk("set odd parity\n");
		cval |=  UART_LCR_EPAR;
	}

	val=request & 0x60;
	val = val >> 5;

	switch(val)
	{
		case BAUD_4800:
			printk("BAUD_4800\n");
			baud = 4800;
			break;
		case BAUD_9600:
			printk("BAUD_9600\n");
			baud = 9600;
			break;
		case BAUD_115200:
			printk("BAUD_115200\n");
			baud = 115200;
		default:
			printk("default\n");
			baud = 9600;
	}
//	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/13);
	quot = serial_omap_get_divisor(up, baud);//for getting dll and dlh register value
	printk("serial_omap_get_divisor=%d\n",quot);

	up->calc_latency = (USEC_PER_SEC * up->fifosize) / (baud / 8);
	up->latency = up->calc_latency;

	up->dll = quot & 0xff;
	up->dlh = quot >> 8;
	up->mdr1 = UART_OMAP_MDR1_DISABLE;

	up->fcr = UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_01 | UART_FCR_ENABLE_FIFO;

       err = rtdm_irq_disable(&up->irq_handle);
       if(err<0)
               rtdm_printk("error in rtdm_irq_enable\n");
       rtdm_lock_get_irqsave(&up->lock,context1);

		up->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;    

//	if (termios->c_iflag & INPCK)
//		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;	      //Frame error indicator, Parity error indicator
//
//	if (termios->c_iflag & (BRKINT | PARMRK))
//		up->port.read_status_mask |= UART_LSR_BI;		      //Break interrupt indicator


		up->ignore_status_mask = 0;
	//this should be passed from user space
//	if (termios->c_iflag & IGNBRK) 					// IGNBRK Ignore BREAK condition on input.						
//	{
		printk("Ignore Break condition on input\n");
		up->ignore_status_mask |= UART_LSR_BI;
//	}
		
		up->ier &= ~UART_IER_MSI;
		
		serial_out(up, UART_IER, up->ier);	
		printk("Enable interrupt\n");
		serial_out(up, UART_LCR, cval);	//writing the setting to Line control register 	/* reset DLAB */
		
		up->lcr = cval;			//saving the setting of line control register
		up->scr = OMAP_UART_SCR_TX_EMPTY;	
		
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);		
		serial_out(up, UART_DLL, 0);
		serial_out(up, UART_DLM, 0);
		serial_out(up, UART_LCR, 0);
//***********************************************************************
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);    //config to mode B

		up->efr = serial_in(up, UART_EFR) & ~UART_EFR_ECB;//value of efr register without enhance function write enable bit
		up->efr &= ~UART_EFR_SCD;			  //remove special character detect enable
		serial_out(up, UART_EFR, up->efr | UART_EFR_ECB); //writing to EFR register with enhance function write enable bit

//************************************************************************************

		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);   	//config to mode A

		up->mcr = serial_in(up, UART_MCR) & ~UART_MCR_TCRTLR;	//value to TCRTLR=0(No action) if 1 then we can enable TCR and TLR
		serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);    //writing value to the MCR with TCRTLR enable
	/* FIFO ENABLE, DMA MODE */

		up->scr |= OMAP_UART_SCR_RX_TRIG_GRANU1_MASK;		//enable the granularity of 1 for trigger RX level
		
//*******************************************************************
	/* Set receive FIFO threshold to 16 characters and
	 * transmit FIFO threshold to 16 spaces
	 */
		up->fcr &= ~OMAP_UART_FCR_RX_FIFO_TRIG_MASK;						// dont set RX_FIFO_TRIG to 60 character
		up->fcr &= ~OMAP_UART_FCR_TX_FIFO_TRIG_MASK;						//dont set TX_FIFO_TRIG to 56 character	
		up->fcr |= UART_FCR6_R_TRIGGER_16 | UART_FCR6_T_TRIGGER_24 | UART_FCR_ENABLE_FIFO;	//Rx fifo trigger at 16 character | Tx fifo trigger at 32 char | FIFO_EN		
		
		serial_out(up, UART_FCR, up->fcr);							//write to FCR
//********************************************************************
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);			//config to mode B
							
		serial_out(up, UART_OMAP_SCR, up->scr);				//writing to SCR(supplementary control register)
//*******************************************************************							
	/* Reset UART_MCR_TCRTLR: this must be done with the EFR_ECB bit set */
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);			//config mode A
		serial_out(up, UART_MCR, up->mcr);				//writing to MCR without TCRTLR
//*******************************************************************
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);			//config mode B
		serial_out(up, UART_EFR, up->efr);				//writing to EFR register without special character detect enable
//*******************************************************************
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);			//config mode A
							
	/* Protocol, Baud Rate, and Interrupt Settings */
							
		if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)			//
			serial_omap_mdr1_errataset(up, up->mdr1);
		else
			serial_out(up, UART_OMAP_MDR1, up->mdr1);
//********************************************************************
		
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);			//config mode B
		serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);		//writing to EFR register with special character
		
		serial_out(up, UART_LCR, 0);					//writing line control register
		serial_out(up, UART_IER, 0);					//writing to IER
//********************************************************************
		
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);			
		
		serial_out(up, UART_DLL, up->dll);	/* LS of divisor */
		serial_out(up, UART_DLM, up->dlh);	/* MS of divisor */
		
		serial_out(up, UART_LCR, 0);
		serial_out(up, UART_IER, up->ier);
//********************************************************************
		
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
		
		serial_out(up, UART_EFR, up->efr);
		serial_out(up, UART_LCR, cval);
		
		if (baud > 230400 && baud != 3000000)
		{	printk("baud > 230400\n");
			up->mdr1 = UART_OMAP_MDR1_13X_MODE;
		}
		else
		{	printk("baud < 230400\n");
			up->mdr1 = UART_OMAP_MDR1_16X_MODE;
		}
		if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)	
		{	printk("up->errata condition true\n");
			serial_omap_mdr1_errataset(up, up->mdr1);
		}
		else
		{	printk("up->errata condition false\n");
			serial_out(up, UART_OMAP_MDR1, up->mdr1);
		}
//***********************************************************************

	/* Configure flow control */
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	/* XON1/XOFF1 accessible mode B, TCRTLR=0, ECB=0 */

//	serial_out(up, UART_XON1, termios->c_cc[VSTART]);
//	serial_out(up, UART_XOFF1, termios->c_cc[VSTOP]);

	/* Enable access to TCR/TLR */
		serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
		serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);

		serial_out(up, UART_TI752_TCR, OMAP_UART_TCR_TRIG);//TCR trasmission control register value 0xFF 

	//no hardware control 
		up->efr &= ~(UART_EFR_CTS | UART_EFR_RTS);

	
		serial_out(up, UART_MCR, up->mcr);			//write to MCR 
		printk("write to UART_MCR\n");
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);		//write to LCR for switching to config mode B
		printk("Switch to config mode B\n");
		serial_out(up, UART_EFR, up->efr);			//write to EFR
		printk("write to EFR register\n");
		serial_out(up, UART_LCR, up->lcr);			//write to LCR
		printk("write to LCR\n");
	
      	rtdm_lock_put_irqrestore(&up->lock,context1);
        	err = rtdm_irq_enable(&up->irq_handle);
         if(err<0)
              	rtdm_printk("error in rtdm_irq_enable\n");

	printk("serial_omap_set_termios end\n");	 
}	
	
static int uart_ioctl_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info,unsigned int req, void *arg)
{	
	MY_DEV *up=(MY_DEV *)context->device->device_data;
	
	printk("Local struct up=%x\n",up);
	
	printk("uart_ioctl_rt start\n");
	printk("req from userspace=%x\n",req);
	if(req != 0)
	{
		serial_omap_set_termios(up, req);
		printk("uart_ioctl_rt end\n");
	}
	return 0;
}	
	
static irqreturn_t serial_omap_irq(int irq,void *dev_id)
{	
//	 irqreturn_t ret;
//         MY_DEV *dev = dev_id;
	
	rtdm_printk("..............my_isr_2..............\n");
	
	return IRQ_HANDLED;
}	


static int rtdm_my_isr(rtdm_irq_t *irq_context)
{

	MY_DEV *up=rtdm_irq_get_arg(irq_context,MY_DEV);

	up->systime1 = rtdm_clock_read();

	up->timeout = up->systime1 - up->systime;

	printk("Interrupt Latency=%dl\n",up->timeout);

	up->systime1=0;
	up->systime=0;

	unsigned int iir,lsr;
	unsigned int type;
	irqreturn_t ret=IRQ_NONE;
	int err;
	int max_count = 256;
	rtdm_lockctx_t context1;

	printk("I am in rtdm_my_isr......!!!\n");

	printk("Local struct up=%x\n",up);

        err = rtdm_irq_disable(&up->irq_handle);
        if(err<0)
             rtdm_printk("error in rtdm_irq_enable\n");
        rtdm_lock_get_irqsave(&up->lock,context1);

	do{
	iir = serial_in(up,UART_IIR);
	if(iir & UART_IIR_NO_INT)
		break;

	ret=IRQ_HANDLED;
	lsr = serial_in(up,UART_LSR);
	type = iir & 0x3e;
	
		switch(type)
		{
			case UART_IIR_THRI:
			printk("type of int:UART_IIR_THRI\n");
			transmit_chars(up,lsr);
			rtdm_event_signal(&up->w_event_tx);
			break;

			case UART_IIR_RX_TIMEOUT:
			/*FALLTHROUGH*/

			case UART_IIR_RDI:
				printk("type of int:UART_IIR_RDI\n");
				serial_omap_rdi(up,lsr);	
				 rtdm_event_signal(&up->w_event_rx);
				break;
			
			case UART_IIR_RLSI:
				printk("type of int:UART_IIR_RLSI\n");
//				serial_omap_rlsi(up,lsr);
				break;
			
			case UART_IIR_CTS_RTS_DSR:
				break;
			
			case UART_IIR_XOFF:
			/*simpleThrough*/
			default:
				break;
		}
	}while(!(iir & UART_IIR_NO_INT) && max_count--);

      	rtdm_lock_put_irqrestore(&up->lock,context1);
        err = rtdm_irq_enable(&up->irq_handle);
        if(err<0)
              rtdm_printk("error in rtdm_irq_enable\n");

	printk("rtdm_irq ended\n");
	
	 up->systime = rtdm_clock_read();
	
	return RTDM_IRQ_HANDLED;
}
 	               

static int uart_open_nrt(struct rtdm_dev_context *context,rtdm_user_info_t *user_info_t,int oflags_t)
{
	MY_DEV *up=(MY_DEV *)context->device->device_data;
//	rtdm_lockctx_t context1;	
	int retval;
	
	printk("Local struct up=%x\n",up);

	f_cir_buf(up);

	rtdm_lock_init(&up->lock);
	rtdm_event_init(&up->w_event_tx,0);
	rtdm_event_init(&up->w_event_rx,0);
	
	printk("name of irq=%s\n",up->name);

	retval = request_irq(up->irq, serial_omap_irq,0, up->name, up);
//	if (retval)
//		return retval;
	
	retval=rtdm_irq_request(&up->irq_handle,up->irq,rtdm_my_isr,0,up->name,up);
	if(retval<0) 
	{
	 rtdm_printk("error in requesting irq\n");
         dev_err(up->dev, "failure requesting irq %i\n", up->irq);
	return retval;
	}

	dev_dbg(up->dev, "serial_omap_startup+%d\n", up->line);
	
	serial_omap_clear_fifos(up);
	serial_out(up,UART_MCR,UART_MCR_RTS);
	
	(void)serial_in(up,UART_LSR);
	
	if(serial_in(up,UART_LSR) & UART_LSR_DR)
		(void)serial_in(up,UART_RX);
	
	(void)serial_in(up,UART_IIR);
	(void)serial_in(up,UART_MSR);
	
	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	printk("UART has word length of 8 bit\n");
	
	up->msr_saved_flags=0;
	
	//enabling interrupts
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up,UART_IER,up->ier);
	
	printk("enabling RLSI and RDI interrupt\n");
	
	//enable module level wake up
	serial_out(up,UART_OMAP_WER,OMAP_UART_WER_MOD_WKUP);
	
	printk("OMAP_UART_WER_MOD_WKUP\n");
//	up->port_activity=jiffies;

        return 0;
}	

static int uart_close_nrt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info)
{
	int err;
        MY_DEV *up=(MY_DEV *)context->device->device_data;
	
	dev_dbg(up->dev, "serial_omap_shutdown+%d\n", up->line);	
	
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	//disable break condition and FIFOs
	serial_out(up,UART_LCR,serial_in(up,UART_LCR) & ~UART_LCR_SBC);
	serial_omap_clear_fifos(up);

	//read data port to reset things and then free irq
	if(serial_in(up,UART_LSR) & UART_LSR_DR)
		(void)serial_in(up,UART_RX);

	err=rtdm_irq_disable(&up->irq_handle);//enable irq
        if(err<0)
        {
                rtdm_printk("error in rtdm_irq_disable\n");
                return err;
        }
        rtdm_printk("rtdm_irq_disable\n");
	rtdm_irq_free(&up->irq_handle);	
	 free_irq(up->irq, up);		
        return 0;
}


#if defined(CONFIG_OF)
static const struct of_device_id omap_i2c_of_match[] = {
        { .compatible = "jay,serial" },
        { .compatible = "jay,serial" },
        { .compatible = "jay,serial" },
        {},
};
MODULE_DEVICE_TABLE(of, omap_i2c_of_match);
#endif

static struct rtdm_device uart_device = {

        .struct_version         = RTDM_DEVICE_STRUCT_VER,
        .device_flags           = RTDM_NAMED_DEVICE,
        .context_size           = sizeof(MY_DEV),
        .device_name            = DEVICE_NAME_1,
        .proc_name              = DEVICE_NAME_1,
        .open_nrt               = uart_open_nrt,

        .ops={
                .close_nrt      =	uart_close_nrt,
                .read_rt        =	uart_rd_rt,
                .write_rt       =	uart_wr_rt,
		.ioctl_rt  	=	uart_ioctl_rt,
        },

        .device_class           =RTDM_CLASS_SERIAL,
        .device_sub_class       =2015,
        .profile_version        =1,
        .driver_name            =DRIVER_NAME,
        .driver_version         =RTDM_DRIVER_VER(1,1,0),
        .peripheral_name        ="RTDM UART MASTER",
        .provider_name          ="JAY KOTHARI",
};

static void omap_serial_fill_features_erratas(struct uart_omap_port *up)
{
	u32 mvr, scheme;
	u16 revision, major, minor;
	printk(".................omap_serial_fill_feature_erratas\n");
	mvr = serial_in(up, UART_OMAP_MVER);//* Module version register */
	/* Check revision register scheme */
	scheme = mvr >> OMAP_UART_MVR_SCHEME_SHIFT;

	switch (scheme) 
	{
	case 0: /* Legacy Scheme: OMAP2/3 */
		/* MINOR_REV[0:4], MAJOR_REV[4:7] */
		major = (mvr & OMAP_UART_LEGACY_MVR_MAJ_MASK) >> OMAP_UART_LEGACY_MVR_MAJ_SHIFT;
		minor = (mvr & OMAP_UART_LEGACY_MVR_MIN_MASK);
		printk("case_0\n");
		break;
	case 1:
		/* New Scheme: OMAP4+ */
		/* MINOR_REV[0:5], MAJOR_REV[8:10] */
		major = (mvr & OMAP_UART_MVR_MAJ_MASK) >>
					OMAP_UART_MVR_MAJ_SHIFT;
		minor = (mvr & OMAP_UART_MVR_MIN_MASK);
		printk("case_1\n");
		break;
	default:
		dev_warn(up->dev,"Unknown %s revision, defaulting to highest\n",up->name);
		/* highest possible revision */
		major = 0xff;
		minor = 0xff;
		printk("default\n");
	}

	/* normalize revision for the driver */
	revision = UART_BUILD_REVISION(major, minor);

	switch (revision) 
	{
	case OMAP_UART_REV_46:
		printk("revision number: OMAP_UART_REV_46\n");
		up->errata |= (UART_ERRATA_i202_MDR1_ACCESS | UART_ERRATA_i291_DMA_FORCEIDLE);
		break;
	case OMAP_UART_REV_52:
		printk("revision number: OMAP_UART_REV_52\n");
		up->errata |= (UART_ERRATA_i202_MDR1_ACCESS | UART_ERRATA_i291_DMA_FORCEIDLE);
		break;
	case OMAP_UART_REV_63:
		printk("revision number: OMAP_UART_REV_63\n");
		up->errata |= UART_ERRATA_i202_MDR1_ACCESS;
		break;
	default:
		break;
	}
	printk("................omap_serial_fill_feature_erratas end\n");
}


static struct omap_uart_port_info *of_get_uart_port_info(struct device *dev)
{
	struct omap_uart_port_info *omap_up_info;

	omap_up_info = devm_kzalloc(dev, sizeof(*omap_up_info), GFP_KERNEL);
	if (!omap_up_info)
		return NULL; /* out of memory */

	of_property_read_u32(dev->of_node, "clock-frequency",&omap_up_info->uartclk);

	return omap_up_info;
}

static int uart_omap_probe(struct platform_device *pdev)
{
	printk("omap_i2c_probe started\n");

	struct uart_omap_port *up;
	struct resource *mem,*irq;
	struct omap_uart_port_info *omap_up_info=pdev->dev.platform_data;
	int ret;
	struct rtdm_device *rdev;	
	void __iomem            *mem_1;
	int retval;

	if(pdev->dev.of_node)
		omap_up_info = of_get_uart_port_info(&pdev->dev);

	mem = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if(!mem)
	{
		dev_err(&pdev->dev,"no mem resource\n");
		return -ENODEV;
	}

	printk("platform_get_resource for mem\n");

	irq = platform_get_resource(pdev,IORESOURCE_IRQ,0);
	if(!irq)
	{
		dev_err(&pdev->dev,"no irq resource\n");
		return -ENODEV;
	}
	
	printk("platform_get_resource for irq\n");
	
	if(!devm_request_mem_region(&pdev->dev,mem->start,resource_size(mem),pdev->dev.driver->name))
	{
		dev_err(&pdev->dev,"memory region already claimed\n");
		return -EBUSY;
	}

	printk("mem->start=%x\n",mem->start);
	printk("irq->start=%x\n",irq->start);

	if ( gpio_is_valid(omap_up_info->DTR_gpio) && omap_up_info->DTR_present) 
	{
		ret = gpio_request(omap_up_info->DTR_gpio, "omap-serial");
			printk("gpio_request\n");
		if (ret < 0)
			return ret;

		ret = gpio_direction_output(omap_up_info->DTR_gpio,omap_up_info->DTR_inverted);
			printk("gpio_direction_output\n");
		if (ret < 0)
			return ret;
	}

	printk("gpio_is_valid\n");

	up = devm_kzalloc(&pdev->dev, sizeof(*up), GFP_KERNEL);
	if (!up)
		return -ENOMEM;

	printk("Local struct up=%x\n",up);


	rtdm_printk("clock enabling.......UART4\n");
        mem_1 = ioremap(SOC_PRCM_REGS, SOC_PRCM_SIZE);
        if(!mem)
         {
            printk (KERN_ERR "HI: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
            return 0;
         }

//        retval=ioread32(mem_1 + 0x78);
//        rtdm_printk("value of clock i2c enable retval=%d\n",retval);
        iowrite32(CM_PER_UART4_CLKCTRL , mem_1 + 0x78);
//        retval = ioread32(CM_PER_UART4_CLKCTRL + 0x78);
//        rtdm_printk("value of clock i2c enable retval=%d\n",retval);
        rtdm_printk("clock enable for UART4\n");
	
	
	rdev=kzalloc(sizeof(struct rtdm_device),GFP_KERNEL);
        rdev = &up->rtdm_dev;
        memcpy(rdev, &uart_device, sizeof(struct rtdm_device));
	
        ret=rtdm_dev_register(rdev);
        if(ret<0)
        {
                    printk("RTDM device not registered\n");
        }
	
        rdev->device_data =  devm_kzalloc(&pdev->dev, sizeof(MY_DEV), GFP_KERNEL);
        rdev->device_data = up;
	
	printk("RTDM driver register\n");
	
	
	if (gpio_is_valid(omap_up_info->DTR_gpio) && omap_up_info->DTR_present) 
	{
		up->DTR_gpio = omap_up_info->DTR_gpio;
		up->DTR_inverted = omap_up_info->DTR_inverted;
		printk("DTR gpio valid\n");
	} 
	else
		up->DTR_gpio = -EINVAL;
	
	up->DTR_active = 0;
	up->dev = &pdev->dev;
	up->irq = irq->start;
	
	up->regshift = 2;
	up->fifosize = 64;

	if (pdev->dev.of_node)
	{
		up->line = of_alias_get_id(pdev->dev.of_node, "serial");
		printk("pdev->dev.of_node\n");
	}
	else
	{
		up->line = pdev->id;
		printk("pdev->id\n");
	}
	if (up->line < 0) 
	{
		dev_err(&pdev->dev, "failed to get alias/pdev id, errno %d\n",up->line);
		ret = -ENODEV;
//		goto err_port_line;
	}	
	
	
	up->pins = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(up->pins)) 
	{
		dev_warn(&pdev->dev, "did not get pins for uart%i error: %li\n",up->line, PTR_ERR(up->pins));
		up->pins = NULL;
	}
	
	printk("devm_pinctrl_get_select_default\n");
	
	sprintf(up->name, "OMAP UART%d", up->line);
	
	
	up->mapbase = mem->start;
	up->membase = devm_ioremap(&pdev->dev,mem->start,resource_size(mem));//what does ioremap do???? read about it
	
	if (!up->membase) 
	{
		dev_err(&pdev->dev, "can't ioremap UART\n");
		ret = -ENOMEM;
//		goto err_ioremap;
	}
	
	up->flags = omap_up_info->flags;
	up->uartclk = omap_up_info->uartclk;
	
	printk("clock freq=%d\n",up->uartclk);
	
	if(!up->uartclk) 
	{	
		up->uartclk = DEFAULT_CLK_SPEED;
		dev_warn(&pdev->dev, "No clock speed specified: using default:""%d\n", DEFAULT_CLK_SPEED);
	}	
		
	up->latency = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE;
	up->calc_latency = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE;

	platform_set_drvdata(pdev, up);

	omap_serial_fill_features_erratas(up);		

        return 0;

//err_unuse_clocks:
//        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
//err_free_mem:
//        platform_set_drvdata(pdev, NULL);
//        return r;
}

static int uart_omap_remove(struct platform_device *pdev)
{
	rtdm_printk("omap_i2c_remove\n");
	
	
	rtdm_printk("omap_i2c_remove");
	
	return 0;
}


struct platform_driver omap_uart_driver = {
	.probe		= uart_omap_probe,
	.remove		= uart_omap_remove,
	.driver		= {
		.name	= "feserial",
//		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(omap_i2c_of_match),
	},
};

/* I2C may be needed to bring up other drivers */
static int __init omap_uart_init_driver(void)
{
	rtdm_printk("omap_uart_init_driver function\n");
	return platform_driver_register(&omap_uart_driver);
}

module_init(omap_uart_init_driver);

static void __exit omap_uart_exit_driver(void)
{
	rtdm_printk("omap_uart_exit_driver exit\n");

	platform_driver_unregister(&omap_uart_driver);

	rtdm_dev_unregister(&uart_device, 1000); 

}
module_exit(omap_uart_exit_driver);

MODULE_AUTHOR("JAY KOTHARI <jaikothari10@gmail.com>");
MODULE_DESCRIPTION("TI OMAP UART ");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:omap_uart");


