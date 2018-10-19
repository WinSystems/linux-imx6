///****************************************************************************
//
//    Copyright 2014 by WinSystems Inc.
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
//    Name       : public.h
//
//    Project    : SBC35-C405
//
//    Author     : pjp
//
//    Public definitions for use with the SBC35-C398 IO60 M410 device driver
//
///****************************************************************************
//
//      Date      Revision    Description
//    --------    --------    ---------------------------------------------
//    07/29/15      0.1       PJP - Original
//
///****************************************************************************

#ifndef _PUBLIC_IO60_SPI_API_H
#define _PUBLIC_IO60_SPI_API_H

///////////////////////////////////////////////////////////////////////////////

#define IOCTL_TYPE              'p'

//
// Define the command set for the device. These are the device IO control codes
// for direct access to the SPI registers in the Lattice Mach XO2 device
//

#define IOCTL_GET_VERSION                 _IOR( IOCTL_TYPE, 0, pvoid )
#define IOCTL_SPI_WRITE                   _IOW( IOCTL_TYPE, 1, pvoid )
#define IOCTL_SPI_WRITE_THEN_READ         _IOR( IOCTL_TYPE, 2, pvoid )
#define IOCTL_SPI_DAC_REG_READ            _IOR( IOCTL_TYPE, 3, pvoid )
#define IOCTL_SPI_READ_WHILE_WRITING      _IOR( IOCTL_TYPE, 4, pvoid )

///////////////////////////////////////////////////////////////////////////////
//
// Define the errors that the device driver may produce
//

#define STATUS_SUCCESS                          0
#define STATUS_REQUEST_IO_REGION_ERROR          2
#define STATUS_DEVICE_CONFIGURATION_ERROR       3
#define STATUS_INSUFFICIENT_RESOURCES           4
#define STATUS_BAD_DATA_COPY                    6
#define STATUS_INVALID_DEVICE_REQUEST           7
#define STATUS_HARDWARE_TIMEOUT                 8
#define STATUS_IOCTL_FAILURE                    9
#define STATUS_OUT_OF_BOUNDS                   10
#define STATUS_BAD_PARAMETER                   11
#define STATUS_WAIT_TIMEOUT                    12
#define STATUS_NULL_POINTER                    13
#define STATUS_BAD_DEV_HANDLE                  14
#define STATUS_GENERIC_ERROR                   15

/////////////////////////////////////////////////////////////////////////////////

typedef struct _SPI_XFER_STRUCT {
                                   puint8_t    pTxBuffer,
                                               pRxBuffer;
                                   uint8_t     TxLen,
                                               RxLen;

                                } SPI_XFER_STRUCT, *P_SPI_XFER_STRUCT;

/////////////////////////////////////////////////////////////////////////////////
#endif
