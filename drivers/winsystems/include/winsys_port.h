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
//    Name       : winsys_port.h
//
//    Project    : generic 
//
//    Author     : pjp
//
///****************************************************************************
//
//      Date      Revision    Description
//    --------    --------    ---------------------------------------------
//    06/11/14      0.1       PJP - Original
//
///****************************************************************************

#ifndef _WINSYS_PORT_H
#define _WINSYS_PORT_H

typedef void *  PVOID;
typedef void *  pvoid;

typedef uint8_t  *  puint8_t;
typedef uint16_t *  puint16_t;
typedef uint32_t *  puint32_t;
typedef uint64_t *  puint64_t;
typedef int8_t   *  pint8_t;
typedef int16_t  *  pint16_t;
typedef int32_t  *  pint32_t;
typedef int64_t  *  pint64_t;

#define true	( 1 == 1 )
#define TRUE	true
#define false	( 1 == 0 )
#define FALSE	false

#define null    NULL

#endif
