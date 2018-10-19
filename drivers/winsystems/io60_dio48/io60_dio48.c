///****************************************************************************
//	
//	Copyright 2014 by WinSystems Inc.
//
//	Permission is hereby granted to the purchaser of WinSystems IO60 cards 
//	to distribute any binary file or files compiled using this source code
//	directly or in any work derived by the user from this file. In no case 
//	may the source code, original or derived from this file, be distributed 
//	to any third party except by explicit permission of WinSystems. This file
//	is distributed on an "as-is" basis and no warranty as to performance or 
//	fitness of purposes is expressed or implied. In no case shall WinSystems 
//	be liable for any direct or indirect loss or damage, real or consequential 
//	resulting from the usage of this source code. It is the user's sole 
//	responsibility to determine fitness for any considered purpose.
//
///****************************************************************************
//
//	Name	 : io60_dio48.c
//
//	Project	 : IO60-DIO48 Linux Device Driver
//
//	Author	 : Brett DeMetrotion
//
///****************************************************************************
//
//	  Date		Revision	                Description
//	--------	--------	---------------------------------------------
//	11/3/14	      1.0		  Original Release	
//
///****************************************************************************

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/cdev.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>         // cli(), *_flags
#include <asm/uaccess.h>        // copy_from/to_user
#include <asm/irq.h>
#include <asm/hw_irq.h>
#include <asm/mach/irq.h>

#include "io60_dio48.h"

#define DEBUG				1
#define SUCCESS				0
#define	DISABLE				0
#define	ENABLE				1
#define ERROR				100
#define SPI_WRITE			0x80
#define IO60_DIO48_IRQ		gpio_to_irq(IMX_GPIO_NR(7, 12))
#define MAX_INTS			1024
#define IO60_DIO48_ID		0x10
#define IO60_DIO48_MAJ		211
#define IO60_DIO48_MIN		1
#define DRVR_NAME			"io60_dio48"

// Function prototypes for local functions 
static u8 io60_dio48_read_port(u8 port);
static u8 io60_dio48_rev_id(void);
static int io60_dio48_enbl_dsbl(u8 enbl_dsbl);
static int io60_dio48_write_port(u8 port, u8 value);
static int get_int(void);
static int get_buffered_int(void);
static void clr_int(int);

// Interrupt handlers 
irqreturn_t irq_handler(int, void *);
void common_handler(void);

// Wait Queue Declared
DECLARE_WAIT_QUEUE_HEAD(wq);

// buffer up the transition interrupts & pass them on to waiting applications
unsigned char int_buffer[MAX_INTS];
int inptr = 0;
int outptr = 0;

// local data structures
struct io60_dio48_data {
	struct mutex	mutex;
} ;

static struct spi_device *spi;
static struct cdev io60_dio48_cdev;

// spi info
//static struct io60_dio48_pdata plat_data = {
//};

static struct spi_board_info io60_dio48_info[] __initdata = {
	{
		.modalias		= IO60_DIO48_NAME,
		.irq			= IO60_DIO48_IRQ,
		.max_speed_hz	= 20000000,
		.bus_num		= 2,
		.chip_select	= 0,
		//.platform_data  = &plat_data,
	}
};

// Driver Declarations
static struct spi_driver io60_dio48_driver = {
	.driver = {
		.name   = IO60_DIO48_NAME,
		.owner  = THIS_MODULE,
	},
	
};

// This is the common interrupt handler
void common_handler()
{
	int c;
		
	while(1)
	{
		c = get_int();

		if (c != 0)
		{
			// Clears the interrupt then updates count
			clr_int(c);
			
			#ifdef DEBUG
				printk("<1>io60_dio48: Interrupt on bit %d\n", c);
			#endif

			// Adds the interrupt to the buffer
			int_buffer[inptr++] = c;
	  			
			if(inptr == MAX_INTS)
				inptr = 0;
	
			wake_up_interruptible(&wq);
		}
		else 
			break;
	}
}

// Handler
irqreturn_t irq_handler(int irq, void *dev_id)
{
	#ifdef DEBUG
	printk("<1>io60_dio48: Interrupt received\n");
	#endif

	common_handler();

	return IRQ_HANDLED;
}

// Generic Read Port Function
static u8 io60_dio48_read_port(u8 port)
{	
	// Read Buffer
	u8 rxbuf[1];
	// Write Buffer
	u8 txbuf[3];
	int ret;
	
	txbuf[0] = 0x05;	
	txbuf[1] = port;	// Address
	txbuf[2] = 0xff;	// Dummy (Any value can be placed here)

    ret = spi_write_then_read(spi, txbuf, 3, rxbuf, 1);
	 
#ifdef DEBUG
    printk("<1>io60_dio48_read: port 0x%02x -> 0x%02x\n", port, rxbuf[0]);
#endif

    return rxbuf[0];
}

// Revision ID retrieve
static u8 io60_dio48_rev_id(void)
{	
	// Read Buffer
	u8 rxbuf[1];
	// Write Buffer
	u8 txbuf[2];
	int ret;
	
	txbuf[0] = 0x9f;	
	txbuf[1] = 0xff;	// Dummy (Any value can be placed here)

    ret = spi_write_then_read(spi, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf));
	 
#ifdef DEBUG
    printk("<1>io60_dio48_rev_id: 0x%02x %d\n", rxbuf[0], ret);
#endif

    return rxbuf[0];
}

// Generic Enable/Disable Function
static int io60_dio48_enbl_dsbl(u8 enbl_dsbl)
{
	u8 tmp[1];
    
	if(enbl_dsbl)
		tmp[0] = 0x06;
	else
		tmp[0] = 0x04;

#ifdef DEBUG
	printk("<1>io60_dio48_enbl_dsbl\n");
#endif

	return spi_write(spi, tmp, sizeof(tmp));
}

// Generic Port Writing Function
static int io60_dio48_write_port(u8 port, u8 value){
	u8 tmp[3];
            
	tmp[0] = 0x01; 
	tmp[1] = port;		// Port number
	tmp[2] = value;		// Write data

#ifdef DEBUG
	printk("<1>io60_dio48_write_port: 0x%02x -> port 0x%02x \n", value, port);
#endif

	return spi_write(spi, tmp, sizeof(tmp));
}

// return first bit with an active interrupt
static int get_int()
{
	int i, j, val;

	// See if any bit set, if so return the bit number 
	for (i = 0; i < 6; i++)
	{
		val = io60_dio48_read_port(IO60_DIO48_PORT0_INT_STAT + i);
					
		// If val = 0 - next increment in loop
		if (!val)
			continue;
					
		for (j = 0; j < 8; j++)
		{
			if (val & (1 << j))
			{
				printk("<1>get_int: port %d, bit %d interrupt occured\n", i, j);
			
				// Return bit number of interrupt
				return ((i * 8) + j + 1);
			}
		}
	}

	return 0;
}

// return first buffered bit with an active interrupt
static int get_buffered_int(void)
{
	int temp;

	if(outptr != inptr)
	{
	    temp = int_buffer[outptr++];

		if(outptr == MAX_INTS)
			outptr = 0;
		
		#ifdef DEBUG
			printk("<1>get_buffered_interrupt: irq %d = %d\n", outptr - 1, temp);
		#endif

		return temp;
	}

	return 0;
}

// Clears the interrupt that was passed in by get_int
static void clr_int(int bit_number)
{
	struct io60_dio48_data *s = dev_get_drvdata(&spi->dev);
	unsigned int port, bit;
	u8 val;

	// obtain lock
	mutex_lock(&s->mutex);

	// Calculate the I/O address based upon bit number
	bit_number--;
	port = (bit_number / 8);
	bit = (bit_number % 8);

	// to clear irq write 0 to bit and then reenable by writing 1 to same bit
	val = io60_dio48_read_port(port + IO60_DIO48_PORT0_IRQ_CLR);
	io60_dio48_write_port(port + IO60_DIO48_PORT0_IRQ_CLR, val & ~(1 << bit));
	io60_dio48_write_port(port + IO60_DIO48_PORT0_IRQ_CLR, val);

	printk("<1>clr_int:  port %d, bit %d interrupt cleared\n", port, bit);

	//release lock
	mutex_unlock(&s->mutex);
}

// Ioctl Case Function
static long io60_dio48_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct io60_dio48_data *s = dev_get_drvdata(&spi->dev);
	u8 port, data, return_data, bit, val;

	// Switch according to the ioctl called 
	switch (ioctl_num)
	{
		case IOCTL_READ_PORT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl read port\n");
			#endif
			port = ioctl_param & 0xFF;
				
			return_data = io60_dio48_read_port(port);

			return return_data;
			
		case IOCTL_WRITE_PORT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl write port\n");
			#endif

			port = ioctl_param & 0xFF;
			
			// Checks if the port is in the valid parameters
			data = (ioctl_param >> 8) & 0xFF;
			
			io60_dio48_write_port(port, data);

			return SUCCESS;

		case IOCTL_READ_BIT: 
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl read bit\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			port = ioctl_param / 8;
			bit = ioctl_param % 8;
			val = io60_dio48_read_port(port);
			return_data = (val >> bit) & 1;

			if(return_data)
				return 1;
			else
				return 0;
			
		// Sets a specific bit in the port
		case IOCTL_SET_BIT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl set bit\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			port = ioctl_param / 8;
			bit = ioctl_param % 8;

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(port);
			val |= (1 << bit);
			io60_dio48_write_port(port, val);

			//release lock
			mutex_unlock(&s->mutex);
			
			return SUCCESS;

		// Clears a specific bit in the port
		case IOCTL_CLR_BIT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl clear bit\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			port = ioctl_param / 8;
			bit = ioctl_param % 8;

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(port);
			val &= ~(1 << bit);
			io60_dio48_write_port(port, val);

			//release lock
			mutex_unlock(&s->mutex);
			
			return SUCCESS;
			
		// Enables rising edge interrupt
		case IOCTL_ENBL_RE_INT: 
			#ifdef DEBUG 
				printk("<1>io60_dio48: ioctl enable rising edge interrupt\n"); 
			#endif 
 
            ioctl_param &= 0xFF; 
            ioctl_param--; 
                 
            port = ioctl_param / 8;
            bit = ioctl_param % 8; 

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(IO60_DIO48_PORT0_RE_IRQ_EN + port);
            val |= (1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_RE_IRQ_EN + port, val); 
            val = io60_dio48_read_port(IO60_DIO48_PORT0_IRQ_CLR + port);
            val |= (1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_IRQ_CLR + port, val); 

			//release lock
			mutex_unlock(&s->mutex);
			
			return SUCCESS;
			
		// Enables falling edge interrupt
		case IOCTL_ENBL_FE_INT:
            #ifdef DEBUG 
	            printk("<1>io60_dio48: ioctl enable falling edge interrupt\n"); 
            #endif 
 
			ioctl_param &= 0xFF; 
            ioctl_param--;  
                 
            port = ioctl_param / 8;
            bit = ioctl_param % 8; 

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(IO60_DIO48_PORT0_FE_IRQ_EN + port);
            val |= (1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_FE_IRQ_EN + port, val); 
            val = io60_dio48_read_port(IO60_DIO48_PORT0_IRQ_CLR + port);
            val |= (1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_IRQ_CLR + port, val); 
 
			//release lock
			mutex_unlock(&s->mutex);
			
            return SUCCESS;
			
		// Disables rising edge interrupt
		case IOCTL_DSBL_RE_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl disable rising edge interrupt\n");
			#endif

            ioctl_param &= 0xFF; 
            ioctl_param--;
                 
            port = ioctl_param / 8;
            bit = ioctl_param % 8; 

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(IO60_DIO48_PORT0_RE_IRQ_EN + port);
            val &= ~(1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_RE_IRQ_EN + port, val); 
            val = io60_dio48_read_port(IO60_DIO48_PORT0_IRQ_CLR + port);
            val &= ~(1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_IRQ_CLR + port, val); 

			//release lock
			mutex_unlock(&s->mutex);

			return SUCCESS;

		// Disables falling edge interrupt
		case IOCTL_DSBL_FE_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl disable falling edge interrupt\n");
			#endif

			ioctl_param &= 0xFF; 
            ioctl_param--;
                 
            port = ioctl_param / 8;
            bit = ioctl_param % 8; 

			// obtain lock
			mutex_lock(&s->mutex);

			val = io60_dio48_read_port(IO60_DIO48_PORT0_FE_IRQ_EN + port);
            val &= ~(1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_FE_IRQ_EN + port, val); 
            val = io60_dio48_read_port(IO60_DIO48_PORT0_IRQ_CLR + port);
            val &= ~(1 << bit); 
            io60_dio48_write_port(IO60_DIO48_PORT0_IRQ_CLR + port, val); 

			//release lock
			mutex_unlock(&s->mutex);
			
			return SUCCESS;

		// Reads a specific Rising Edge Interrupt Bit
		case IOCTL_READ_RE_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl read rising edge interrupt\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			// obtain lock
			mutex_lock(&s->mutex);

			port = ioctl_param / 8;
			bit = ioctl_param % 8;
			val = io60_dio48_read_port(IO60_DIO48_PORT0_RE_IRQ_EN + port);
			return_data = (val >> bit) & 1;

			//release lock
			mutex_unlock(&s->mutex);
			
			if(return_data)
				return 1;
			else
				return 0;

		// Reads a specific Rising Edge Interrupt Bit
		case IOCTL_READ_FE_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl read falling edge interrupt\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			port = ioctl_param / 8;
			bit = ioctl_param % 8;
			val = io60_dio48_read_port(IO60_DIO48_PORT0_FE_IRQ_EN + port);
			return_data = (val >> bit) & 1;

			if(return_data)
				return 1;
			else
				return 0;

		// This will search through the interrupt registers and return the bit of the first one that is set.
		case IOCTL_GET_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl get interrupt\n");
			#endif

			return get_buffered_int();

		//Clears specific interrupt bit
		case IOCTL_CLR_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl clear interrupt\n");
			#endif

			clr_int(ioctl_param & 0xFF);

			return SUCCESS;

		// This will go into a sleep that can be interrupted. Once that is broken, it will access the interrupt handler
		case IOCTL_WAIT_INT:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl wait interrupt\n");
			#endif

			// if interrupt already exists, service it
			if ((data = get_buffered_int()))
				return data;

			interruptible_sleep_on(&wq);

			#ifdef DEBUG
				printk("<1>io60_dio48: awoken\n");
			#endif

			// Getting here does not guarantee that there's an in interrupt available
			// we may have been awakened by some other signal. In any case We'll 
			// return whatever's available in the interrupt queue even if it's empty
			return get_buffered_int();

		// Checks all the interrupt registers and returns the value. If it is a 1, an interrupt occured.
		case IOCTL_INT_STATUS:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl interrupt status\n");
			#endif

			ioctl_param &= 0xFF;
			ioctl_param--;

			port = ioctl_param / 8;
			bit = ioctl_param % 8;
			val = io60_dio48_read_port(IO60_DIO48_PORT0_INT_STAT + port);
			return_data = (val >> bit) & 1;

			if(return_data)
				return 1;
			else
				return 0;
				
			return SUCCESS;

		// This returns the rev ID of the board/software
		case READ_REV:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl read revision\n");
			#endif

			return io60_dio48_rev_id();
			
		// This controls the onboard LED to show that the device, driver, and processor are communicating correctly.
		case LED_CONTROL:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl LED control\n");
			#endif

			ioctl_param &= 0xFF;

			if(ioctl_param == 0x06)
				io60_dio48_enbl_dsbl(DISABLE);
			else if(ioctl_param == 0x04) 
				io60_dio48_enbl_dsbl(ENABLE);
				
			return SUCCESS;

		// Catch all return
		default:
			#ifdef DEBUG
				printk("<1>io60_dio48: ioctl call undefined\n");
			#endif

			return(-EINVAL);
	}
}

// io60_dio48 Open Function
int io60_dio48_open(struct inode *inode, struct file *file)
{
	#ifdef DEBUG
	printk("<1>io60_dio48: device_open(%p, %p)\n", inode, file);
	#endif

	return SUCCESS;
}

// io60_dio48 Release Function
int io60_dio48_release(struct inode *inode, struct file *file)
{
	#ifdef DEBUG
	printk("<1>io60_dio48: device_release(%p, %p)\n", inode, file);
	#endif

	return SUCCESS;
}

// FOPS Structure
struct file_operations io60_dio48_fops = {
	owner:					THIS_MODULE,
	unlocked_ioctl:         io60_dio48_ioctl,
	open:                   io60_dio48_open,
	release:                io60_dio48_release,
};

// Probe Module
static int io60_dio48_probe(struct device *dev){
	struct io60_dio48_data *data;
	int ret_val, devno;

	// register the character device
	devno = MKDEV(IO60_DIO48_MAJ, 0);
	ret_val = register_chrdev_region(devno, 1, DRVR_NAME);

	if(ret_val < 0)
	{
		printk("<1>io60_dio48: cannot register chrdev: %d\n", IO60_DIO48_MAJ);
		return -ENODEV;
	}

	// add character device
	cdev_init(&io60_dio48_cdev, &io60_dio48_fops);
	io60_dio48_cdev.owner = THIS_MODULE;
	io60_dio48_cdev.ops = &io60_dio48_fops;
	ret_val = cdev_add(&io60_dio48_cdev, MKDEV(IO60_DIO48_MAJ, 0), 1);
	
	if(ret_val < 0)
	{
		printk("<1>io60_dio48: cannot add char dev: %d\n", ret_val);
		ret_val = -ENODEV;
		goto nocdevadd;
	}

	// alloc io60_dio48_data space
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
	{
		printk("<1>io60_dio48: no memory for data structure\n");
		ret_val = -ENOMEM;
		goto nomem;
	}
	
	// point io60_dio48_data to device structure
	dev_set_drvdata(dev, data);
	
	// initailize 
	mutex_init(&data->mutex);

	// request irq
	ret_val = devm_request_threaded_irq(dev, IO60_DIO48_IRQ, NULL, irq_handler,
									IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
									dev_name(dev), data);

	return SUCCESS;

nomem:
	cdev_del(&io60_dio48_cdev);
nocdevadd:
	unregister_chrdev_region(MKDEV(IO60_DIO48_MAJ, 0), 1);
	
	return ret_val;
}

// Init Module
int __init io60_dio48_init(void)
{
	struct spi_master *bus;
	int status;
	u8 val;

	// register spi driver
	status = spi_register_driver(&io60_dio48_driver);
	if (status) {
		// cannot register driver
		printk("<1>io60_dio48: cannot register spi driver\n");
		return status;
	}

	// populate bus
	bus = spi_busnum_to_master(2);
	if (!bus) {
		// no such bus
		printk("<1>io60_dio48: invalid spi bus\n");
		status = -ENODEV;
		goto nobus;
	}

	// add devices
	spi = spi_new_device(bus, io60_dio48_info);
	if (!spi) {
		// device registration failed
		printk("<1>io60_dio48: unable to add spi device\n");
		status = -ENODEV;
		goto nodev;
	}

	// verify card is installed
	val = io60_dio48_rev_id();	
	
	if (val != 1){
		printk("<1>io60_dio48: unable to detect spi device: val -> %0x\n", val);
		status = -ENODEV;
		goto nobrd;
	}
	
	// card is present so probe
	status = io60_dio48_probe(&spi->dev);
	if (status)
		goto noprobe;

	printk("<1>io60_dio48_init completed!\n");

	return SUCCESS;

noprobe:
nobrd:
	spi_unregister_device(spi);
nodev:
nobus:
	spi_unregister_driver(&io60_dio48_driver); 

	return status;
}

// Cleanup Module
void __exit io60_dio48_exit(void)
{
	printk("<1>io60_dio48_exit executed\n");

	cdev_del(&io60_dio48_cdev);
	unregister_chrdev_region(MKDEV(IO60_DIO48_MAJ, 0), 1);
	spi_unregister_device(spi);
	spi_unregister_driver(&io60_dio48_driver); 
}  

module_init(io60_dio48_init);
module_exit(io60_dio48_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("WinSystems,Inc. IO60-DIO48 Driver");
MODULE_AUTHOR("Brett DeMetrotion");
