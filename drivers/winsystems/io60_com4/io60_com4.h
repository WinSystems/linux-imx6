////***************************************************************************
//	
//	Copyright 2014 by WinSystems Inc.
//
//	Permission is hereby granted to the purchaser of WinSystems GPIO cards 
//	and CPU products incorporating a GPIO device, to distribute any binary 
//	file or files compiled using this source code directly or in any work 
//	derived by the user from this file. In no case may the source code, 
//	original or derived from this file, be distributed to any third party 
//	except by explicit permission of WinSystems. This file is distributed 
//	on an "As-is" basis and no warranty as to performance or fitness of pur-
//	poses is expressed or implied. In no case shall WinSystems be liable for 
//	any direct or indirect loss or damage, real or consequential resulting 
//	from the usage of this source code. It is the user's sole responsibility 
//	to determine fitness for any considered purpose.
//
////***************************************************************************
//
//	Name	 : io60-com4.h
//
//	Project	 : IO60-COM4 Linux Device Driver
//
//	Author	 : Paul DeMetrotion
//
////***************************************************************************
//
//	  Date		Revision	                Description
//	--------	--------	---------------------------------------------
//	05/30/14	  1.0		Original Release	
//
////***************************************************************************

#define	IO60_COM4_NAME		"io60_com4"
#define IO60_COM4_DEV		"ttyio60com"
#define	IO60_COM4_MAJ		210
#define IO60_COM4_MIN		1
#define	UART_NR				4

// MAX14830 register definitions
#define MAX14830_RHR_REG                 (0x00) // RX FIFO
#define MAX14830_THR_REG                 (0x00) // TX FIFO
#define MAX14830_IRQEN_REG               (0x01) // IRQ enable
#define MAX14830_IRQSTS_REG              (0x02) // IRQ status
#define MAX14830_LSR_IRQEN_REG           (0x03) // LSR IRQ enable
#define MAX14830_LSR_IRQSTS_REG          (0x04) // LSR IRQ status
#define MAX14830_REG_05                  (0x05)
#define MAX14830_SPCHR_IRQEN_REG         MAX14830_REG_05 // Special char IRQ en
#define MAX14830_SPCHR_IRQSTS_REG        (0x06) // Special char IRQ status
#define MAX14830_STS_IRQEN_REG           (0x07) // Status IRQ enable
#define MAX14830_STS_IRQSTS_REG          (0x08) // Status IRQ status
#define MAX14830_MODE1_REG               (0x09) // MODE1
#define MAX14830_MODE2_REG               (0x0a) // MODE2
#define MAX14830_LCR_REG                 (0x0b) // LCR
#define MAX14830_RXTO_REG                (0x0c) // RX timeout
#define MAX14830_HDPIXDELAY_REG          (0x0d) // Auto transceiver delays
#define MAX14830_IRDA_REG                (0x0e) // IRDA settings
#define MAX14830_FLOWLVL_REG             (0x0f) // Flow control levels
#define MAX14830_FIFOTRIGLVL_REG         (0x10) // FIFO IRQ trigger levels
#define MAX14830_TXFIFOLVL_REG           (0x11) // TX FIFO level
#define MAX14830_RXFIFOLVL_REG           (0x12) // RX FIFO level
#define MAX14830_FLOWCTRL_REG            (0x13) // Flow control
#define MAX14830_XON1_REG                (0x14) // XON1 character
#define MAX14830_XON2_REG                (0x15) // XON2 character
#define MAX14830_XOFF1_REG               (0x16) // XOFF1 character
#define MAX14830_XOFF2_REG               (0x17) // XOFF2 character
#define MAX14830_GPIOCFG_REG             (0x18) // GPIO config
#define MAX14830_GPIODATA_REG            (0x19) // GPIO data
#define MAX14830_PLLCFG_REG              (0x1a) // PLL config
#define MAX14830_BRGCFG_REG              (0x1b) // Baud rate generator conf
#define MAX14830_BRGDIVLSB_REG           (0x1c) // Baud rate divisor LSB
#define MAX14830_BRGDIVMSB_REG           (0x1d) // Baud rate divisor MSB
#define MAX14830_CLKSRC_REG              (0x1e) // Clock source
#define MAX14830_REG_1F                  (0x1f)

#define MAX14830_REVID_REG               MAX14830_REG_1F // Revision ID

#define MAX14830_GLOBALIRQ_REG           MAX14830_REG_1F // Global IRQ (RO)
#define MAX14830_GLOBALCMD_REG           MAX14830_REG_1F // Global Command (WO)

// Extended registers
#define MAX14830_REVID_EXTREG            MAX14830_REG_05 // Revision ID

// IRQ register bits
#define MAX14830_IRQ_LSR_BIT             (1 << 0) // LSR interrupt
#define MAX14830_IRQ_SPCHR_BIT           (1 << 1) // Special char interrupt
#define MAX14830_IRQ_STS_BIT             (1 << 2) // Status interrupt
#define MAX14830_IRQ_RXFIFO_BIT          (1 << 3) // RX FIFO interrupt
#define MAX14830_IRQ_TXFIFO_BIT          (1 << 4) // TX FIFO interrupt
#define MAX14830_IRQ_TXEMPTY_BIT         (1 << 5) // TX FIFO empty interrupt
#define MAX14830_IRQ_RXEMPTY_BIT         (1 << 6) // RX FIFO empty interrupt
#define MAX14830_IRQ_CTS_BIT             (1 << 7) // CTS interrupt

// LSR register bits
#define MAX14830_LSR_RXTO_BIT            (1 << 0) // RX timeout
#define MAX14830_LSR_RXOVR_BIT           (1 << 1) // RX overrun
#define MAX14830_LSR_RXPAR_BIT           (1 << 2) // RX parity error
#define MAX14830_LSR_FRERR_BIT           (1 << 3) // Frame error
#define MAX14830_LSR_RXBRK_BIT           (1 << 4) // RX break
#define MAX14830_LSR_RXNOISE_BIT         (1 << 5) // RX noise
#define MAX14830_LSR_CTS_BIT             (1 << 7) // CTS pin state

// Special character register bits
#define MAX14830_SPCHR_XON1_BIT          (1 << 0) // XON1 character
#define MAX14830_SPCHR_XON2_BIT          (1 << 1) // XON2 character
#define MAX14830_SPCHR_XOFF1_BIT         (1 << 2) // XOFF1 character
#define MAX14830_SPCHR_XOFF2_BIT         (1 << 3) // XOFF2 character
#define MAX14830_SPCHR_BREAK_BIT         (1 << 4) // RX break
#define MAX14830_SPCHR_MULTIDROP_BIT     (1 << 5) // 9-bit multidrop addr char

// Status register bits
#define MAX14830_STS_GPIO0_BIT           (1 << 0) // GPIO 0 interrupt
#define MAX14830_STS_GPIO1_BIT           (1 << 1) // GPIO 1 interrupt
#define MAX14830_STS_GPIO2_BIT           (1 << 2) // GPIO 2 interrupt
#define MAX14830_STS_GPIO3_BIT           (1 << 3) // GPIO 3 interrupt
#define MAX14830_STS_CLKREADY_BIT        (1 << 5) // Clock ready

// MODE1 register bits
#define MAX14830_MODE1_RXDIS_BIT         (1 << 0) // RX disable
#define MAX14830_MODE1_TXDIS_BIT         (1 << 1) // TX disable
#define MAX14830_MODE1_TXHIZ_BIT         (1 << 2) // TX pin three-state
#define MAX14830_MODE1_RTSHIZ_BIT        (1 << 3) // RTS pin three-state
#define MAX14830_MODE1_TRNSCVCTRL_BIT    (1 << 4) // Transceiver ctrl enable
#define MAX14830_MODE1_IRQSEL_BIT        (1 << 7) // IRQ pin enable

// MODE2 register bits
#define MAX14830_MODE2_RST_BIT           (1 << 0) // Chip reset
#define MAX14830_MODE2_FIFORST_BIT       (1 << 1) // FIFO reset
#define MAX14830_MODE2_RXTRIGINV_BIT     (1 << 2) // RX FIFO INT invert
#define MAX14830_MODE2_RXEMPTINV_BIT     (1 << 3) // RX FIFO empty INT invert
#define MAX14830_MODE2_SPCHR_BIT         (1 << 4) // Special chr detect enable
#define MAX14830_MODE2_LOOPBACK_BIT      (1 << 5) // Internal loopback enable
#define MAX14830_MODE2_MULTIDROP_BIT     (1 << 6) // 9-bit multidrop enable
#define MAX14830_MODE2_ECHOSUPR_BIT      (1 << 7) // ECHO suppression enable

// LCR register bits
#define MAX14830_LCR_LENGTH0_BIT         (1 << 0) // Word length bit 0
#define MAX14830_LCR_LENGTH1_BIT         (1 << 1) // Word length bit 1
                                                 /*
                                                  * Word length bits table:
                                                  * 00 -> 5 bit words
                                                  * 01 -> 6 bit words
                                                  * 10 -> 7 bit words
                                                  * 11 -> 8 bit words
                                                  */
#define MAX14830_LCR_STOPLEN_BIT         (1 << 2) // STOP length bit
                                                 /*
                                                  * STOP length bit table:
                                                  * 0 -> 1 stop bit
                                                  * 1 -> 1-1.5 stop bits if
                                                  *      word length is 5,
                                                  *      2 stop bits otherwise
                                                  */
#define MAX14830_LCR_PARITY_BIT          (1 << 3) // Parity bit enable
#define MAX14830_LCR_EVENPARITY_BIT      (1 << 4) // Even parity bit enable
#define MAX14830_LCR_FORCEPARITY_BIT     (1 << 5) // 9-bit multidrop parity
#define MAX14830_LCR_TXBREAK_BIT         (1 << 6) // TX break enable
#define MAX14830_LCR_RTS_BIT             (1 << 7) // RTS pin control
#define MAX14830_LCR_WORD_LEN_5          (0x00)
#define MAX14830_LCR_WORD_LEN_6          (0x01)
#define MAX14830_LCR_WORD_LEN_7          (0x02)
#define MAX14830_LCR_WORD_LEN_8          (0x03)

// Flow control trigger level register masks
#define MAX14830_FLOWLVL_HALT_MASK       (0x000f)	// Flow control halt level
#define MAX14830_FLOWLVL_RES_MASK        (0x00f0)	// Flow control resume level */
#define MAX14830_FLOWLVL_HALT(words)     ((words / 8) & 0x0f)
#define MAX14830_FLOWLVL_RES(words)      (((words / 8) & 0x0f) << 4)

// FIFO interrupt trigger level register masks
#define MAX14830_FIFOTRIGLVL_TX_MASK     (0x0f)		// TX FIFO trigger level
#define MAX14830_FIFOTRIGLVL_RX_MASK     (0xf0)		//RX FIFO trigger level
#define MAX14830_FIFOTRIGLVL_TX(words)   ((words / 8) & 0x0f)
#define MAX14830_FIFOTRIGLVL_RX(words)   (((words / 8) & 0x0f) << 4)
 
// Flow control register bits
#define MAX14830_FLOWCTRL_AUTORTS_BIT    (1 << 0) // Auto RTS flow ctrl enable
#define MAX14830_FLOWCTRL_AUTOCTS_BIT    (1 << 1) // Auto CTS flow ctrl enable
#define MAX14830_FLOWCTRL_GPIADDR_BIT    (1 << 2) /* Enables that GPIO inputs
                                                  * are used in conjunction with
                                                  * XOFF2 for definition of
                                                  * special character */
#define MAX14830_FLOWCTRL_SWFLOWEN_BIT   (1 << 3) // Auto SW flow ctrl enable
#define MAX14830_FLOWCTRL_SWFLOW0_BIT    (1 << 4) // SWFLOW bit 0
#define MAX14830_FLOWCTRL_SWFLOW1_BIT    (1 << 5) /* SWFLOW bit 1
                                                  *
                                                  * SWFLOW bits 1 & 0 table:
                                                  * 00 -> no transmitter flow
                                                  *       control
                                                  * 01 -> receiver compares
                                                  *       XON2 and XOFF2
                                                  *       and controls
                                                  *       transmitter
                                                  * 10 -> receiver compares
                                                  *       XON1 and XOFF1
                                                  *       and controls
                                                  *       transmitter
                                                  * 11 -> receiver compares
                                                  *       XON1, XON2, XOFF1 and
                                                  *       XOFF2 and controls
                                                  *       transmitter
                                                  */
#define MAX14830_FLOWCTRL_SWFLOW2_BIT    (1 << 6) // SWFLOW bit 2
#define MAX14830_FLOWCTRL_SWFLOW3_BIT    (1 << 7) /* SWFLOW bit 3
                                                  *
                                                  * SWFLOW bits 3 & 2 table:
                                                  * 00 -> no received flow
                                                  *       control
                                                  * 01 -> transmitter generates
                                                  *       XON2 and XOFF2
                                                  * 10 -> transmitter generates
                                                  *       XON1 and XOFF1
                                                  * 11 -> transmitter generates
                                                  *       XON1, XON2, XOFF1 and
                                                  *       XOFF2
                                                  */

// Baud rate generator configuration register bits
#define MAX14830_BRGCFG_2XMODE_BIT       (1 << 4) // Double baud rate
#define MAX14830_BRGCFG_4XMODE_BIT       (1 << 5) // Quadruple baud rate
#define MAX14830_BRGCFG_CLKDIS_BIT		 (1 << 6) // Clock Disable
 
// Clock source register bits
#define MAX14830_CLKSRC_CRYST_BIT        (1 << 1) // Crystal osc enable/
#define MAX14830_CLKSRC_PLL_BIT          (1 << 2) // PLL enable
#define MAX14830_CLKSRC_PLLBYP_BIT       (1 << 3) // PLL bypass
#define MAX14830_CLKSRC_CLK2RTS_BIT      (1 << 7) // Baud clk to RTS pin

// Global commands
#define MAX14830_EXTREG_ENBL             (0xce)
#define MAX14830_EXTREG_DSBL             (0xcd)

// Misc definitions
#define MAX14830_FIFO_SIZE               (128)

// io60_com4 platform data structure
struct io60_com4_pdata {
	// flags global to UART port
	const u8 uart_flags[UART_NR];
#define IO60_COM4_LOOPBACK        (0x00000001)    // loopback mode enable
#define IO60_COM4_ECHO_SUPRESS    (0x00000002)    // enable echo supress
#define IO60_COM4_AUTO_DIR_CTRL   (0x00000004)    // enable auto direction control (RS-485)
};
