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
//	Name	 : io60-dio48.h
//
//	Project	 : IO60-DIO48 Linux Device Driver
//
//	Author	 : Brett DeMetrotion
//
////***************************************************************************
//
//	  Date		Revision	                Description
//	--------	--------	---------------------------------------------
//	11/3/14	      1.0		  Original Release	
//
////***************************************************************************

#include <linux/ioctl.h> 

#define	IO60_DIO48_NAME		"io60_dio48"
#define	IO60_DIO48_MAJ		211
#define MAX_CHIPS			1
#define IOCTL_NUM			't'

char *device_id ="/dev/io60_dio48";

// io60_dio48 platform data structure
//struct io60_dio48_pdata {
// };

// Ioctl Definitions
#define IOCTL_READ_PORT _IOWR(IOCTL_NUM, 1, int)

//Write to the DIO48 Port
#define IOCTL_WRITE_PORT _IOWR(IOCTL_NUM, 2, int)

// Read a specific bit function
#define IOCTL_READ_BIT _IOWR(IOCTL_NUM, 3, int)	

// Set a specific bit function
#define IOCTL_SET_BIT _IOWR(IOCTL_NUM, 4, int)

// Clear a specific bit function
#define IOCTL_CLR_BIT _IOWR(IOCTL_NUM, 5, int)

// Enable a specific bit that enables Rising Edge Interrupt bit
#define IOCTL_ENBL_RE_INT _IOWR(IOCTL_NUM, 6, int)

// Enable a specific bit that enables Falling Edge Interrupt bit
#define IOCTL_ENBL_FE_INT _IOWR(IOCTL_NUM, 7, int)

// Disable a specific bit that disables that Rising Edge Interrupt bit
#define IOCTL_DSBL_RE_INT _IOWR(IOCTL_NUM, 8, int)

// Disable a specific bit that disables that Falling Edge Interrupt bit
#define IOCTL_DSBL_FE_INT _IOWR(IOCTL_NUM, 9, int)

// Read Rising Edge Interrupt bits
#define IOCTL_READ_RE_INT _IOWR(IOCTL_NUM, 10, int)

// Read Falling Edge Interrupt bits
#define IOCTL_READ_FE_INT _IOWR(IOCTL_NUM, 11, int)

// Searches through registers for an interrupt
#define IOCTL_GET_INT _IOWR(IOCTL_NUM, 12, int)

// Clears an interrupt bit
#define IOCTL_CLR_INT _IOWR(IOCTL_NUM, 13, int)		

// Waits for an interrupt
#define IOCTL_WAIT_INT _IOWR(IOCTL_NUM, 14, int)

// Checks if a interrupt bit is up and clears that bit
#define IOCTL_INT_STATUS _IOWR(IOCTL_NUM, 15, int)

// Checks the Board Revision number
#define READ_REV _IOWR(IOCTL_NUM, 16, int)

// Controls onboard LED
#define LED_CONTROL _IOWR(IOCTL_NUM, 17, int)

// register addresses
#define IO60_DIO48_PORT0			0x00
#define IO60_DIO48_PORT1			0x01
#define IO60_DIO48_PORT2			0x02
#define IO60_DIO48_PORT3			0x03
#define IO60_DIO48_PORT4			0x04
#define IO60_DIO48_PORT5			0x05
#define IO60_DIO48_PORT0_RE_IRQ_EN	0x10
#define IO60_DIO48_PORT1_RE_IRQ_EN	0x11
#define IO60_DIO48_PORT2_RE_IRQ_EN	0x12
#define IO60_DIO48_PORT3_RE_IRQ_EN	0x13
#define IO60_DIO48_PORT4_RE_IRQ_EN	0x14
#define IO60_DIO48_PORT5_RE_IRQ_EN	0x15
#define IO60_DIO48_PORT0_FE_IRQ_EN	0x20
#define IO60_DIO48_PORT1_FE_IRQ_EN	0x21
#define IO60_DIO48_PORT2_FE_IRQ_EN	0x22
#define IO60_DIO48_PORT3_FE_IRQ_EN	0x23
#define IO60_DIO48_PORT4_FE_IRQ_EN	0x24
#define IO60_DIO48_PORT5_FE_IRQ_EN	0x25
#define IO60_DIO48_PORT0_IRQ_CLR	0x30
#define IO60_DIO48_PORT1_IRQ_CLR	0x31
#define IO60_DIO48_PORT2_IRQ_CLR	0x32
#define IO60_DIO48_PORT3_IRQ_CLR	0x33
#define IO60_DIO48_PORT4_IRQ_CLR	0x34
#define IO60_DIO48_PORT5_IRQ_CLR	0x35
#define IO60_DIO48_PORT0_INT_STAT	0x40
#define IO60_DIO48_PORT1_INT_STAT	0x41
#define IO60_DIO48_PORT2_INT_STAT	0x42
#define IO60_DIO48_PORT3_INT_STAT	0x43
#define IO60_DIO48_PORT4_INT_STAT	0x44
#define IO60_DIO48_PORT5_INT_STAT	0x45
