///****************************************************************************
//
//    Copyright 2015 by WinSystems Inc.
//
//    Permission is hereby granted to the purchaser of WinSystems GPIO cards
//    and CPU products incorporating a GPIO device, to distribute any binary
//    file or files compiled using this source code directly or in any work
//    derived by the user from this file. In no case may the source code,
//    original or derived from this file, be distributed to any third party
//    except by explicit permission of WinSystems. This file is distributed
//    on an "As-is" basis and no warranty as to performance or fitness of pur-
//    poses is expressed or implied. In no case shall WinSystems be liable for
//    any direct or indirect loss or damage, real or consequential resulting
//    from the usage of this source code. It is the user's sole responsibility
//    to determine fitness for any considered purpose.
//
///****************************************************************************
//
//    Name       : io60_spi_drv.h
//
//    Project    : SBC35-C398 ARM Linux
//
//    Author     : Patrick Philp
//
//    Definitions & typedefs for the IO60-SPI device driver
//
///****************************************************************************
//
//      Date      Revision    Description
//    --------    --------    ---------------------------------------------
//    07/28/15      0.1       PJP - Original
//
///****************************************************************************

#ifndef _WINSYS_SPI_MASTER_H
#define _WINSYS_SPI_MASTER_H

#define WS_VERSION_MAJOR                   0x01
#define WS_VERSION_MINOR                   0x02
#define WS_BUILD_NUMBER                    0x0004

#define WS_VERSION             ( ( ( ( uint32_t ) WS_VERSION_MAJOR ) << 24 ) \
                                    | ( ( ( uint32_t ) WS_VERSION_MINOR ) << 16 ) \
                                    | ( ( uint32_t ) WS_BUILD_NUMBER ) )


#define FIRST_MINOR            0            // first device
#define MINOR_COUNT            4            // needs to be the same value as number of chip selects below...
//
// the name by which the driver may be accessed, after appending the
// chip select number...

#define IO60_SPI_BASE_DEVICE_NAME              "io60_spi_cs"
#define IO60_SPI_DRIVER_CLASS_NAME             "io60_spi"

///////////////////////////////////////////////////////////////////////////////

#define C398_IO60_SPI_BUS_NUM            2

#define IO60_SPI_NUM_CHIP_SELECTS        4
#define IO60_SPI_MIN_CHIP_SELECT         0
#define IO60_SPI_MAX_CHIP_SELECT         ( IO60_SPI_NUM_CHIP_SELECTS - 1 )

#define SPI_MAX_CLOCK_RATE               33000000

///////////////////////////////////////////////////////////////////////////////
//
// a "delay" value used with the timeout logic on
// status bits TxInProgress, TxRdy, & RxRdy
//
#define WAIT_USEC_DELAY_PERIOD             10

//
// timeout values to wait for TxInProgress, TxRdy,
// & RxRdy
//

#define TIMEOUT_1_MSEC                     1
#define TIMEOUT_2_MSECS                    2
#define TIMEOUT_5_MSECS                    5
#define TIMEOUT_10_MSECS                  10
#define TIMEOUT_25_MSECS                  25
#define TIMEOUT_50_MSECS                  50
#define TIMEOUT_100_MSECS                100
#define TIMEOUT_250_MSECS                250
#define TIMEOUT_500_MSECS                500

#define TIMEOUT_VALUE                   TIMEOUT_1_MSEC
#define USECS_PER_MILLISEC              1000            // 1000 uSecs = 1 mSec

#endif
