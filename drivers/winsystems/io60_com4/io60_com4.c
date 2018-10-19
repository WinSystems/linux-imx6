///****************************************************************************
//	
//	Copyright 2014 by WinSystems Inc.
//
//  Based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
//  Based on max3100.c, by Christian Pellegrin <chripell@evolware.org>
//  Based on max3110.c, by Feng Tang <feng.tang@intel.com>
//  Based on max3107.c, by Aavamobile
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
//	Name	 : io60_com4.c
//
//	Project	 : IO60-COM4 Linux Device Driver
//
//	Author	 : Paul DeMetrotion
//
///****************************************************************************
//
//	  Date		Revision	                Description
//	--------	--------	---------------------------------------------
//	05/30/14	  1.0		Original Release	
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
#include <linux/ratelimit.h>

#include <linux/delay.h>
#include <linux/cdev.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>         // cli(), *_flags
#include <asm/uaccess.h>        // copy_from/to_user
#include <asm/irq.h>
#include <asm/hw_irq.h>
#include <asm/mach/irq.h>

#include "io60_com4.h"


//#define DEBUG				1
#define SUCCESS				0
#define SPI_WRITE			0x80
#define IO60_COM4_IRQ		gpio_to_irq(IMX_GPIO_NR(7, 12))
#define IO60_COM4_FIFO_SIZE 128
#define IO60_COM4_UART_CLK	1843200
#define	MAX14830_REVID		0xb3
#define IO60_COM4_REV_MASK  0xfc

// modprobe command line argument
static char *protocol[UART_NR] = {"rs232",
			 					  "rs232",
								  "rs232",
								  "rs232"};
module_param_array(protocol, charp, NULL, S_IRUGO);

// local data structures
struct io60_com4_one {
	struct uart_port        port;
	struct work_struct      tx_work;
} ;

struct io60_com4_data {
	struct uart_driver		uart;
	void					(*power)(struct uart_port *, int);
	struct mutex            mutex;
	struct io60_com4_pdata	*pdata;
	struct io60_com4_one	p[4];
} ;

struct spi_device *spi;

// generic read + write functions
static int io60_com4_read(struct uart_port *port, int offset)
{
	unsigned int reg, val;

	reg = (port->line << 5) | offset;
	val = spi_w8r8(spi, reg);

	#ifdef DEBUG
	printk("<1>io60_com4_read : port %d, reg 0x%0x -> 0x%02x\n", port->line, offset, val);
	#endif

	return val;
}

static int io60_com4_write(struct uart_port *port, int offset, u8 value)
{
	u8 tmp[2];
		
	tmp[0] = SPI_WRITE | (port->line << 5) | offset;
	tmp[1] = value;

	#ifdef DEBUG
	printk("<1>io60_com4_write : 0x%02x -> port %d, reg 0x%0x\n", value, port->line, offset);
	#endif

	return spi_write(spi, tmp, sizeof(tmp));
}

static int glbl_cmd_write(u8 command)
{
	u8 tmp[2];
		
	tmp[0] = SPI_WRITE | MAX14830_GLOBALCMD_REG;
	tmp[1] = command;

	return spi_write(spi, tmp, sizeof(tmp));
}

static void io60_com4_handle_rx(struct uart_port *port, unsigned int rxlen)
{
	unsigned int sts, ch, flag;

	#ifdef DEBUG
	printk("<1>io60_com4_handle_rx called (rxlen = %d) for port %d\n", rxlen, port->line);
	#endif

	if (unlikely(rxlen >= port->fifosize)) {
		// code for dev_warn_ratelimited - not supported by our kernel version
		do {
			static DEFINE_RATELIMIT_STATE(_rs, DEFAULT_RATELIMIT_INTERVAL, DEFAULT_RATELIMIT_BURST);

			if (__ratelimit(&_rs))
				dev_warn(port->dev, "Port %i: Possible RX FIFO overrun\n", port->line);
		} while (0);

		port->icount.buf_overrun++;
		// ensure sanity of RX level
		rxlen = port->fifosize;
		#ifdef DEBUG
		printk("<1>io60_com4_handle_rx overrun\n");
		#endif
	}

	while (rxlen--) {
		ch = io60_com4_read(port, MAX14830_RHR_REG);
		sts = io60_com4_read(port, MAX14830_LSR_IRQSTS_REG);

		sts &= MAX14830_LSR_RXPAR_BIT | MAX14830_LSR_FRERR_BIT |
			MAX14830_LSR_RXOVR_BIT | MAX14830_LSR_RXBRK_BIT;

		port->icount.rx++;
		flag = TTY_NORMAL;

		if (unlikely(sts)) {
			if (sts & MAX14830_LSR_RXBRK_BIT) {
				port->icount.brk++;
                
				if (uart_handle_break(port))
					continue;
			} 
			else if (sts & MAX14830_LSR_RXPAR_BIT)
				port->icount.parity++;
			else if (sts & MAX14830_LSR_FRERR_BIT)
				port->icount.frame++;
			else if (sts & MAX14830_LSR_RXOVR_BIT)
				port->icount.overrun++;

			sts &= port->read_status_mask;
			if (sts & MAX14830_LSR_RXBRK_BIT)
				flag = TTY_BREAK;
			else if (sts & MAX14830_LSR_RXPAR_BIT)
				flag = TTY_PARITY;
			else if (sts & MAX14830_LSR_FRERR_BIT)
				flag = TTY_FRAME;
			else if (sts & MAX14830_LSR_RXOVR_BIT)
				flag = TTY_OVERRUN;
		}
 
		if (uart_handle_sysrq_char(port, ch))
			continue;

		if (sts & port->ignore_status_mask)
			continue;

		uart_insert_char(port, sts, MAX14830_LSR_RXOVR_BIT, ch, flag);
	}

	tty_flip_buffer_push(port->state->port.tty);
}

static void io60_com4_handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send;

	#ifdef DEBUG
	printk("<1>io60_com4_handle_tx called for port %d\n", port->line);
	#endif

	if (unlikely(port->x_char)) {
		io60_com4_write(port, MAX14830_THR_REG, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	// get length of data pending in circular buffer
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		// limit to size of TX FIFO
		txlen = io60_com4_read(port, MAX14830_TXFIFOLVL_REG);
		txlen = port->fifosize - txlen;
		to_send = (to_send > txlen) ? txlen : to_send;

		// add data to send
		port->icount.tx += to_send;
		while (to_send--) {
			io60_com4_write(port, MAX14830_THR_REG, xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		};
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void io60_com4_wq_proc(struct work_struct *ws)
{
	struct io60_com4_one *one = container_of(ws, struct io60_com4_one, tx_work);
	struct io60_com4_data *s = dev_get_drvdata(one->port.dev);

	#ifdef DEBUG
	printk("<1>io60_com4_wq_proc called\n");
	#endif

	mutex_lock(&s->mutex);
	io60_com4_handle_tx(&one->port);
	mutex_unlock(&s->mutex);
}

// interrupt handler
static void io60_com4_port_irq(struct io60_com4_data *s, int portno)
{
	struct uart_port *port = &s->p[portno].port;
	unsigned int ists, lsr, rxlen;
//	u8 val;

	#ifdef DEBUG
	printk("<1>io60_com4_port_irq called for port %d\n", portno);
	#endif

	do {
		// read IRQ status & RX FIFO level
		ists = io60_com4_read(port, MAX14830_IRQSTS_REG);
		rxlen = io60_com4_read(port, MAX14830_RXFIFOLVL_REG);
//		val = io60_com4_read(port, MAX14830_LCR_REG);
//		printk("<1>ists = %0x rxlen = %d\n", ists, rxlen);

		if (!ists && !rxlen) {
//			printk("<1>   No interrupt\n");
//			io60_com4_write(port, MAX14830_LCR_REG, val);
			break;
		}

//		if (ists & MAX14830_IRQ_RXFIFO_BIT) {
//			printk("<1>   RxFIFOTrigInt\n");
//			io60_com4_write(port, MAX14830_LCR_REG, val | MAX14830_LSR_CTS_BIT);
//		}

		if (ists & MAX14830_IRQ_CTS_BIT) {
			lsr = io60_com4_read(port, MAX14830_LSR_IRQSTS_REG);
//			printk("<1>   CTSInt lsr = %0x\n", lsr);
			uart_handle_cts_change(port, !!(lsr & MAX14830_LSR_CTS_BIT));
		}

		if (rxlen)
//			printk("<1>   RxFIFOEmptyInt\n");
			io60_com4_handle_rx(port, rxlen);

		if (ists & MAX14830_IRQ_TXEMPTY_BIT) {
//			printk("<1>   TxFIFOEmptyInt\n");
			mutex_lock(&s->mutex);
			io60_com4_handle_tx(port);
			mutex_unlock(&s->mutex);
		}
	} while (1);
}

static irqreturn_t io60_com4_ist(int irq, void *dev_id)
{
	struct io60_com4_one *one = container_of(dev_id, struct io60_com4_one, port);
	struct io60_com4_data *s = (struct io60_com4_data *)dev_id;
	u8 val = ~0;

	#ifdef DEBUG
	printk("<1>io60_com4_ist called\n");
	#endif

	do {
		val = io60_com4_read(&one->port, MAX14830_GLOBALIRQ_REG);

		val = 0xf & ~val;

		if (!val)
			break;

		io60_com4_port_irq(s, fls(val) - 1);
	} while (1);

	return IRQ_HANDLED;
}

// uart functions
static void io60_com4_set_baud(struct uart_port *port, int baud)
{
	unsigned int mode = 0, div = port->uartclk / baud;

	#ifdef DEBUG
	printk("<1>io60_com4_set_baud called (baud = %d)\n", baud);
	printk("<1>io60_com4_set_baud: div = %d\n", div);
	#endif

	if (!(div / 16)) {
		// Mode x2
		mode = MAX14830_BRGCFG_2XMODE_BIT;
		div = (port->uartclk * 2) / baud;
	}

	if (!(div / 16)) {
		// Mode x4
		mode = MAX14830_BRGCFG_4XMODE_BIT;
		div = (port->uartclk * 4) / baud;
	}

	io60_com4_write(port, MAX14830_BRGDIVMSB_REG, (div / 16) >> 8);
	io60_com4_write(port, MAX14830_BRGDIVLSB_REG, div / 16);
	io60_com4_write(port, MAX14830_BRGCFG_REG, (div % 16) | mode);
}

static unsigned int io60_com4_tx_empty(struct uart_port *port)
{
	unsigned int lvl, sts;

	#ifdef DEBUG
	printk("<1>io60_com4_tx_empty called\n");
	#endif

	lvl = io60_com4_read(port, MAX14830_TXFIFOLVL_REG);
	sts = io60_com4_read(port, MAX14830_IRQSTS_REG);

	return ((sts & MAX14830_IRQ_TXEMPTY_BIT) && !lvl) ? TIOCSER_TEMT : 0;
}

static void io60_com4_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	#ifdef DEBUG
	printk("<1>io60_com4_set_mctrl called\n");
	#endif
}

static unsigned int io60_com4_get_mctrl(struct uart_port *port)
{
	u8 val, rtn = 0;

	#ifdef DEBUG
	printk("<1>io60_com4_get_mctrl called\n");
	#endif

//	val = io60_com4_read(port, MAX14830_FLOWCTRL_REG);

//	if (val & MAX14830_FLOWCTRL_AUTORTS_BIT)
//		rtn |= TIOCM_RTS;
//	if (val & MAX14830_FLOWCTRL_AUTOCTS_BIT)
//		rtn |= TIOCM_CTS;

//	return rtn | TIOCM_DSR | TIOCM_CAR;
	return rtn;
}

static void io60_com4_stop_tx(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_stop_tx called\n");
	#endif
}

static void io60_com4_start_tx(struct uart_port *port)
{
	struct io60_com4_one *one = container_of(port, struct io60_com4_one, port);

	#ifdef DEBUG
	printk("<1>io60_com4_start_tx called\n");
	#endif

	if (!work_pending(&one->tx_work))
		schedule_work(&one->tx_work);
}

static void io60_com4_stop_rx(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_com4_stop_rx called\n");
	#endif
}

static void io60_com4_enable_ms(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_com4_enable_ms called\n");
	#endif
}

static void io60_com4_break_ctl(struct uart_port *port, int break_state)
{
	u8 val;

	#ifdef DEBUG
	printk("<1>io60_com4_break_ctl called\n");
	#endif
	
	val = io60_com4_read(port, MAX14830_LCR_REG);
	if (break_state)
		val |= MAX14830_LCR_TXBREAK_BIT;
	else
		val &= ~MAX14830_LCR_TXBREAK_BIT;

	io60_com4_write(port, MAX14830_LCR_REG, val);
}

static int io60_com4_startup(struct uart_port *port)
{
	struct io60_com4_data *s = dev_get_drvdata(&spi->dev);
	u8 val;
//	int i;

	#ifdef DEBUG
	printk("<1>io60_com4_startup called\n");
	#endif

	// power up port
	s->power(port, 1);

	// configure baud rate, 9600 as default
	io60_com4_set_baud(port, 9600);

	// Configure LCR register, 8N1 mode by default
	io60_com4_write(port, MAX14830_LCR_REG, MAX14830_LCR_WORD_LEN_8);

	// Configure MODE1 register
	if (!(strcmp(protocol[port->line], "rs422")) || !(strcmp(protocol[port->line], "rs485")))
	{
		val = io60_com4_read(port, MAX14830_MODE1_REG);
		io60_com4_write(port, MAX14830_MODE1_REG, val | MAX14830_MODE1_TRNSCVCTRL_BIT);
		#ifdef DEBUG
		printk("<1>io60_com4_startup: protocol[%d] = %s\n", port->line, protocol[port->line]);
		#endif
	}
	else
	{
		#ifdef DEBUG
		printk("<1>io60_com4_startup: protocol[%d] = %s\n", port->line, protocol[port->line]);
		#endif
	}

	// Configure MODE2 register
	val = MAX14830_MODE2_RXEMPTINV_BIT;// | MAX14830_MODE2_LOOPBACK_BIT;

	// reset FIFOs
	val |= MAX14830_MODE2_FIFORST_BIT;
	io60_com4_write(port, MAX14830_MODE2_REG, val);
	val &= ~MAX14830_MODE2_FIFORST_BIT;
	io60_com4_write(port, MAX14830_MODE2_REG, val);

	// configure flow control levels
	// flow control halt level 96, resume level 48
	val = MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96);
	io60_com4_write(port, MAX14830_FLOWLVL_REG, val);

	// temp irq on rxbuffer
//	io60_com4_write(port, MAX14830_FIFOTRIGLVL_REG, 0x40);

	// Clear IRQ status register
	io60_com4_read(port, MAX14830_IRQSTS_REG);

	// Enable RX, TX, CTS change interrupts
	val = MAX14830_IRQ_RXEMPTY_BIT | MAX14830_IRQ_TXEMPTY_BIT | MAX14830_IRQ_CTS_BIT;// | MAX14830_IRQ_RXFIFO_BIT;
	io60_com4_write(port, MAX14830_IRQEN_REG, val);

	// check all registers at end of startup
//	for (i=0; i<0x20; i++) {
//		printk("<1>reg %0x = %0x\n", i, io60_com4_read(port, i));
//	}

	return SUCCESS;
}

static void io60_com4_shutdown(struct uart_port *port)
{
	struct io60_com4_data *s = dev_get_drvdata(port->dev);

	#ifdef DEBUG
	printk("<1>io60_com4_shutdown called\n");
	#endif

	// disable all interrupts
	io60_com4_write(port, MAX14830_IRQEN_REG, 0);

	s->power(port, 0);
}

static void io60_com4_set_termios(struct uart_port *port, 
								  struct ktermios *termios, 
								  struct ktermios *old)
{
	unsigned int lcr, flow = 0;
	int baud;
	u8 val;

	#ifdef DEBUG
	printk("<1>io60_com4_set_termios called\n");
	#endif

	// mask termios capabilities we don't support
	termios->c_cflag &= ~CMSPAR;

	// word size
	switch (termios->c_cflag & CSIZE) {
		case CS5:
			lcr = MAX14830_LCR_WORD_LEN_5;
			break;
		case CS6:
			lcr = MAX14830_LCR_WORD_LEN_6;
			break;
		case CS7:
			lcr = MAX14830_LCR_WORD_LEN_7;
			break;
		case CS8:
		default:
			lcr = MAX14830_LCR_WORD_LEN_8;
			break;
	}

	// parity
	if (termios->c_cflag & PARENB) {
		lcr |= MAX14830_LCR_PARITY_BIT;
		if (!(termios->c_cflag & PARODD))
			lcr |= MAX14830_LCR_EVENPARITY_BIT;
	}

	// stop bits
	if (termios->c_cflag & CSTOPB)
		lcr |= MAX14830_LCR_STOPLEN_BIT; // 2 stops

	// update LCR register
	io60_com4_write(port, MAX14830_LCR_REG, lcr);

	// set read status mask
	port->read_status_mask = MAX14830_LSR_RXOVR_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= MAX14830_LSR_RXPAR_BIT | MAX14830_LSR_FRERR_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= MAX14830_LSR_RXBRK_BIT;
 
	// set status ignore mask
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= MAX14830_LSR_RXBRK_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= MAX14830_LSR_RXPAR_BIT |
									MAX14830_LSR_RXOVR_BIT |
									MAX14830_LSR_FRERR_BIT |
									MAX14830_LSR_RXBRK_BIT;
	
	// configure flow control
	io60_com4_write(port, MAX14830_XON1_REG, termios->c_cc[VSTART]);
	io60_com4_write(port, MAX14830_XOFF1_REG, termios->c_cc[VSTOP]);
	if (termios->c_cflag & CRTSCTS)
		flow |= MAX14830_FLOWCTRL_AUTOCTS_BIT | MAX14830_FLOWCTRL_AUTORTS_BIT;
	if (termios->c_iflag & IXON)
		flow |= MAX14830_FLOWCTRL_SWFLOW3_BIT | MAX14830_FLOWCTRL_SWFLOWEN_BIT;
	if (termios->c_iflag & IXOFF)
		flow |= MAX14830_FLOWCTRL_SWFLOW1_BIT | MAX14830_FLOWCTRL_SWFLOWEN_BIT;

	// must disable transmitter before changing autocts bit
	if (flow & MAX14830_FLOWCTRL_AUTOCTS_BIT) {
		val = io60_com4_read(port, MAX14830_MODE1_REG);
		val |= MAX14830_MODE1_TXDIS_BIT;
		io60_com4_write(port, MAX14830_MODE1_REG, val);
	}

	// change flow settings
	io60_com4_write(port, MAX14830_FLOWCTRL_REG, flow);

	// reenable transmitter
	if (flow & MAX14830_FLOWCTRL_AUTOCTS_BIT) {
		val &= ~MAX14830_MODE1_TXDIS_BIT;
		io60_com4_write(port, MAX14830_MODE1_REG, val);
	}

	// get baud rate generator configuration
	baud = uart_get_baud_rate(port, termios, old,
							  port->uartclk / 16 / 0xffff,
							  port->uartclk / 16);

	// setup baudrate generator
	io60_com4_set_baud(port, baud);

	// update timeout according to new baud rate
	uart_update_timeout(port, termios->c_cflag, baud);
}

static const char *io60_com4_type(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_com4_type called\n");
	#endif

	return ( ( port->type == PORT_MAX14830 ) ? IO60_COM4_NAME : NULL );
}

static int io60_com4_request_port(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_com4_request_port called\n");
	#endif
	
	// do nothing
	return SUCCESS;
}

static void io60_com4_release_port(struct uart_port *port)
{
	#ifdef DEBUG
	printk("<1>io60_com4_release_port called\n");
	#endif

	// do nothing
}

static void io60_com4_config_port(struct uart_port *port, int flags)
{
	#ifdef DEBUG
	printk("<1>io60_com4_config_port called\n");
	#endif
	
	if (flags & UART_CONFIG_TYPE)
	{
		port->type = PORT_MAX14830;
		printk("<1>io60_com4_config_port: type set\n");
	}
}

static int io60_com4_verify_port(struct uart_port *port, struct serial_struct *s)
{
	#ifdef DEBUG
	printk("<1>io60_com4_verify_port called\n");
	#endif
	
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_MAX14830))
	{
		printk("<1>io60_com4_verify_port: type wrong\n");
		return -EINVAL;
	}
	if (s->irq != port->irq)
	{
		printk("<1>io60_com4_verify_port: irq wrong\n");
		return -EINVAL;
	}

	return SUCCESS;
}

// uart info
static struct uart_ops io60_com4_ops = {
	.tx_empty		= io60_com4_tx_empty,
	.set_mctrl		= io60_com4_set_mctrl,
	.get_mctrl		= io60_com4_get_mctrl,
	.stop_tx			= io60_com4_stop_tx,
	.start_tx		= io60_com4_start_tx,
	.stop_rx			= io60_com4_stop_rx,
	.enable_ms		= io60_com4_enable_ms,
	.break_ctl		= io60_com4_break_ctl,
	.startup			= io60_com4_startup,
	.shutdown		= io60_com4_shutdown,
	.set_termios	= io60_com4_set_termios,
	.type				= io60_com4_type,
	.request_port  = io60_com4_request_port,
	.release_port  = io60_com4_release_port,
	.config_port   = io60_com4_config_port,
	.verify_port   = io60_com4_verify_port,
};

// spi info
static struct io60_com4_pdata plat_data = {
	.uart_flags[0]  = IO60_COM4_ECHO_SUPRESS | IO60_COM4_AUTO_DIR_CTRL,
};

static struct spi_board_info io60_com4_info[] __initdata = {
	{
		.modalias		= IO60_COM4_NAME,
		.irq			= IO60_COM4_IRQ,
		.max_speed_hz	= 20000000,
		.bus_num		= 2,
		.chip_select	= 0,
		.platform_data  = &plat_data,
	}
};

// Driver Declarations
static struct spi_driver io60_com4_driver = {
	.driver = {
		.name   = IO60_COM4_NAME,
		.owner  = THIS_MODULE,
	},
};

static void max14830_power(struct uart_port *port, int on)
{
	u8 val;

	#ifdef DEBUG
	printk("<1>max14830_power called\n");
	#endif
	
	val = io60_com4_read(port, MAX14830_BRGCFG_REG);

	if (on)
		val &= ~MAX14830_BRGCFG_CLKDIS_BIT;
	else
		val |=  MAX14830_BRGCFG_CLKDIS_BIT;

	io60_com4_write(port, MAX14830_BRGCFG_REG, val);

	if (on)
		msleep(50);
}

// Probe Module
static int io60_com4_probe(struct device *dev)
{
	struct io60_com4_data *data;
	int i, ret;
	u8 val, tmp[2];

	// alloc io60_com4_data space
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
	{
		printk("<1>io60_com4 : no memory for data structure\n");
		return -ENOMEM;
	}

	// point io60_com4_data to device structure
	dev_set_drvdata(dev, data);

	// set power function
	data->power = &max14830_power;

	// initailize 
	mutex_init(&data->mutex);

	// reset each uart
	for (i = 0; i < UART_NR; i++) {
		// reset port
		tmp[0] = SPI_WRITE | (i << 5) | MAX14830_MODE2_REG;
		tmp[1] = MAX14830_MODE2_RST_BIT;
		#ifdef DEBUG
		printk("<1>port %d reset\n", i);
		printk("<1>spi write addr = 0x%0x data = 0x%0x\n", tmp[0], tmp[1]);
		#endif
		spi_write(spi, tmp, sizeof(tmp));

		// clear port reset
		tmp[1] = 0;
		spi_write(spi, tmp, sizeof(tmp));
		#ifdef DEBUG
		printk("<1>spi write addr = 0x%0x data = 0x%0x\n", tmp[0], tmp[1]);
		#endif

		// wait for port startup
		do {
			val = spi_w8r8(spi, (i << 5) | MAX14830_BRGDIVLSB_REG);
			#ifdef DEBUG
			printk("<1>wait for port %d startup\n", i);
			printk("<1>spi read addr = 0x%0x data = 0x%0x\n", (i << 5) | MAX14830_BRGDIVLSB_REG, val);
			#endif
		} while (val != 0x01);
	}

	// register uart driver
	data->uart.owner			= THIS_MODULE;
	data->uart.driver_name		= IO60_COM4_NAME;
	data->uart.dev_name			= IO60_COM4_DEV;
	data->uart.major			= IO60_COM4_MAJ;
 	data->uart.minor			= IO60_COM4_MIN;
	data->uart.nr				= UART_NR;

	ret = uart_register_driver(&data->uart);
	if (ret) {
		printk("<1>io60_com4 : cannot register uart driver\n");
		kfree(data);
		return ret;
	}

	// add uart ports
	for (i = 0; i < UART_NR; i++) {
		// initialize port data
		data->p[i].port.line		= i;
		data->p[i].port.dev			= dev;
		data->p[i].port.irq			= IO60_COM4_IRQ;
		data->p[i].port.type		= PORT_MAX14830;
		data->p[i].port.fifosize	= IO60_COM4_FIFO_SIZE;
		data->p[i].port.flags		= UPF_SKIP_TEST | UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		data->p[i].port.iotype		= UPIO_PORT;
		data->p[i].port.iobase		= i * 0x20;
		data->p[i].port.membase		= (void __iomem *)~0;
		data->p[i].port.uartclk		= IO60_COM4_UART_CLK;
		data->p[i].port.ops			= &io60_com4_ops;

		// disable all interrupts
		io60_com4_write(&data->p[i].port, MAX14830_IRQEN_REG, 0);

		// clear IRQ status register
		io60_com4_read(&data->p[i].port, MAX14830_IRQSTS_REG);

		// enable IRQ pin
		val = io60_com4_read(&data->p[i].port, MAX14830_MODE1_REG);
		val |= MAX14830_MODE1_IRQSEL_BIT;
		io60_com4_write(&data->p[i].port, MAX14830_MODE1_REG, val);

		// initialize queue for start TX
		INIT_WORK(&data->p[i].tx_work, io60_com4_wq_proc);

		// register port
		uart_add_one_port(&data->uart, &data->p[i].port);

		//Go to suspend mode
		data->power(&data->p[i].port, 0);

		printk("<1>io60_com4 : added uart port ttyio60com%d\n", i);
	}

	// request irq
	ret = devm_request_threaded_irq(dev, IO60_COM4_IRQ, NULL, io60_com4_ist,
									IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
									dev_name(dev), data);

	if (ret) {
		printk("<1>io60_com4 : interrupt %d denied\n", IO60_COM4_IRQ);
		uart_unregister_driver(&data->uart);
        return ret;
	}

	return SUCCESS;
}

// Init Module
int __init io60_com4_init(void)
{
	struct spi_master *bus;
	int status;
	u8	revid;

	// register spi driver
	status = spi_register_driver(&io60_com4_driver);
	if (status) {
		// cannot register driver
		printk("<1>io60_com4: cannot register spi driver\n");
		return status;
	}

	// populate bus
	bus = spi_busnum_to_master(2);
	if (!bus) {
		// no such bus
		printk("<1>io60_com4: invalid spi bus\n");
		status = -ENODEV;
		goto nobus;
	}

	// add devices
	spi = spi_new_device(bus, io60_com4_info);

	if (!spi) {
		// device registration failed
		printk("<1>io60_com4: unable to add spi device\n");
		status = -ENODEV;
		goto nodev;
	}

	// verify card is installed
	glbl_cmd_write(MAX14830_EXTREG_ENBL);

	revid = spi_w8r8(spi, MAX14830_REVID_EXTREG);
	if (revid != MAX14830_REVID)
	{
		printk("<1>io60_com4 : board not installed, revid = 0x%02x\n", revid);
		status = -ENODEV;
		goto nobrd;
	}

	glbl_cmd_write(MAX14830_EXTREG_DSBL);

	#ifdef DEBUG
	printk("<1>io60_com4 : revid = 0x%02x\n", revid);
	#endif

	// card is present so probe
	status = io60_com4_probe(&spi->dev);
	if (status)
		goto noprobe;

	printk("<1>io60_com4_init completed!\n");

	return SUCCESS;

noprobe:
nobrd:
	spi_unregister_device(spi);
nodev:
nobus:
	spi_unregister_driver(&io60_com4_driver); 

	return status;
}

// Cleanup Module
void __exit io60_com4_exit(void)
{
	struct io60_com4_data *s = dev_get_drvdata(&spi->dev);
	int i;

	printk("<1>io60_com4_exit executed\n");

	for (i=0; i < UART_NR; i++)
		uart_remove_one_port(&s->uart, &s->p[i].port);
	uart_unregister_driver(&s->uart);
	kfree(s);
	spi_unregister_device(spi);
	spi_unregister_driver(&io60_com4_driver); 
}  

module_init(io60_com4_init);
module_exit(io60_com4_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WinSystems,Inc. IO60-COM4 Driver");
MODULE_AUTHOR("Paul DeMetrotion");
