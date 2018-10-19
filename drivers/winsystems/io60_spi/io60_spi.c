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
//    Name       : io60_m410.c
//
//    Project    : IO60-M410 driver for SBC35-C398 ARM Linux
//
//    Author     : Patrick Philp
//
///****************************************************************************
//
//      Date      Revision    Description
//    --------    --------    ---------------------------------------------
//    07/28/15      0.1       PJP
//
///****************************************************************************

#define __KERNEL_BUILD

//
// Necessary includes for this device driver
//
#include <linux/module.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>       // printk()
#include <linux/delay.h>
#include <linux/fs.h>           // everything...
#include <linux/errno.h>        // error codes
#include <linux/interrupt.h>
#include <linux/types.h>        // size_t
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        // O_ACCMODE
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>        // copy_from/to_user
#include <linux/compat.h>

//
// "local" includes
//

#include <local_optimize.h>
#include "../include/winsys_port.h"
#include "io60_spi_api.h"
#include "io60_spi_drv.h"
#include "printk_directives.h"
//#include "io60_m410_dio24.h"        // for the "virtual" chip select definition

MODULE_LICENSE("Dual BSD/GPL");

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// standard typedefs...
//

////////////////////////////////////////////////////////////////////////////////
//
// local defines...
//

#define AD5755_NOP_1             0x1C
#define AD5755_NOP_2             0xE0

////////////////////////////////////////////////////////////////////////////////
//
// prototypes of the functions declared in this device drive
//

int     io60_spi_init( void );
int     io60_spi_open( struct inode *inode, struct file *filp );
int     io60_spi_release( struct inode *inode, struct file *filp );
void    io60_spi_uninit( void );
long    io60_spi_ioctl( struct file *filp, unsigned int cmd, unsigned long data );

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Global variables exported by the driver
//


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// local (static) variables
//

static const uint8_t  *DeviceName[ IO60_SPI_NUM_CHIP_SELECTS ] = {
                                                                    IO60_SPI_BASE_DEVICE_NAME"0",
                                                                    IO60_SPI_BASE_DEVICE_NAME"1",
                                                                    IO60_SPI_BASE_DEVICE_NAME"2",
                                                                    IO60_SPI_BASE_DEVICE_NAME"3"
                                                                 };


static const uint8_t    DriverClassName[] = IO60_SPI_DRIVER_CLASS_NAME;
static const uint32_t   Version = WS_VERSION;

#define ENUMERATED_DEVICE_STRING                IO60_SPI_BASE_DEVICE_NAME"%d"

///////////////////////////
//
// some driver related structures...
//

//
// a pointer to the structure that contains the master's info...
//
struct spi_master *pSpiMaster;

struct IO60_SPI_CDEV {
                        dev_t              Dev_t;            // The major & minor device numbers. The minor number is
                                                             // the SPI chip select.
                        struct cdev        CharDev;
                        struct semaphore   Sema4;

                        //
                        // some SPI controller variables...
                        //

                        struct spi_device *pSpiDevice;
                        uint8_t            ChipSelect;

                        //
                        //
                        //

                        struct device     *pDevice;
                    };

//
// allocate the above structure
//

static struct IO60_SPI_CDEV           Io60CdevArray[ IO60_SPI_NUM_CHIP_SELECTS ];
static struct class                  *pIo60SpiDevClass;

////////////////////////////////////////
//
// Linux SPI driver stuff...
//
static struct spi_driver io60_spi_driver[ IO60_SPI_NUM_CHIP_SELECTS ] = {
                                                                           {
                                                                              .driver = {
                                                                                           .name   = IO60_SPI_BASE_DEVICE_NAME"0",
                                                                                           .owner  = THIS_MODULE,
                                                                                        },
                                                                           },

                                                                           {
                                                                              .driver = {
                                                                                           .name   = IO60_SPI_BASE_DEVICE_NAME"1",
                                                                                           .owner  = THIS_MODULE,
                                                                                        },
                                                                           },

                                                                           {
                                                                              .driver = {
                                                                                           .name   = IO60_SPI_BASE_DEVICE_NAME"2",
                                                                                           .owner  = THIS_MODULE,
                                                                                        },
                                                                           },

                                                                           {
                                                                              .driver = {
                                                                                           .name   = IO60_SPI_BASE_DEVICE_NAME"3",
                                                                                           .owner  = THIS_MODULE,
                                                                                        },
                                                                           },
                                                                        };
//
// spi device info
//
static struct spi_board_info io60_spi_dev_info[ IO60_SPI_NUM_CHIP_SELECTS ]  __initdata = {
                                                                                             {
                                                                                                .modalias       = IO60_SPI_BASE_DEVICE_NAME"0",
                                                                                                .max_speed_hz   = 11000000,
                                                                                                .mode           = SPI_MODE_0,
                                                                                                .bus_num        = C398_IO60_SPI_BUS_NUM,
                                                                                                .chip_select    = 0,
                                                                                             },

                                                                                             {
                                                                                                .modalias       = IO60_SPI_BASE_DEVICE_NAME"1",
                                                                                                .max_speed_hz   = 11000000,
                                                                                                .mode           = SPI_MODE_0,
                                                                                                .bus_num        = C398_IO60_SPI_BUS_NUM,
                                                                                                .chip_select    = 1,
                                                                                             },

                                                                                             {
                                                                                                .modalias       = IO60_SPI_BASE_DEVICE_NAME"2",
                                                                                                .max_speed_hz   = 11000000,
                                                                                                .mode           = SPI_MODE_0,
                                                                                                .bus_num        = C398_IO60_SPI_BUS_NUM,
                                                                                                .chip_select    = 2,
                                                                                             },

                                                                                             {
                                                                                                .modalias       = IO60_SPI_BASE_DEVICE_NAME"3",
                                                                                                .max_speed_hz   = 11000000,
                                                                                                .mode           = SPI_MODE_0,
                                                                                                .bus_num        = C398_IO60_SPI_BUS_NUM,
                                                                                                .chip_select    = 3,
                                                                                             }
                                                                                          };

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Structure that declares this driver's file access functions
//

struct file_operations IO60_SPI_Fops = {
                                          owner: THIS_MODULE,
                                          open: io60_spi_open,
                                          release: io60_spi_release,
                                          unlocked_ioctl: io60_spi_ioctl
                                       };


//
// Declaration of the init and exit functions
//

module_init( io60_spi_init );
module_exit( io60_spi_uninit );

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// local (static) functions
//

///////////////////////////////////////////////////////////////////////////////
//
// Function: SpiUnregsiterDrivers
//
// Description:
//
// Parameters:
//    <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
static __SET_LOCAL_OPTIMIZATION_LEVEL void SpiUnregsiterDrivers( struct spi_driver *pSpiDriver )
{
   uint8_t i;

#ifdef __DETAILED_DEBUG
   printk( "%s: entered\n", __func__ );
#endif

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      if ( pSpiDriver )
      {
         spi_unregister_driver( pSpiDriver++ );
      }
   }

#ifdef __DETAILED_DEBUG
   printk( "%s: exited\n", __func__ );
#endif
}

///////////////////////////////////////////////////////////////////////////////
//
// Function: SpiUnregsiterDevices
//
// Description:
//
// Parameters:
//    <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
static __SET_LOCAL_OPTIMIZATION_LEVEL void SpiUnregsiterDevices( struct IO60_SPI_CDEV *pCDevs )
{
   uint8_t i;

#ifdef __DETAILED_DEBUG
   printk( "%s: entered, pCDev = 0x%08x \n", __func__, ( uint32_t ) pCDevs );
#endif

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      if ( pCDevs->pSpiDevice )
      {
         spi_unregister_device( ( pCDevs++ )->pSpiDevice );
      }
   }

   SpiUnregsiterDrivers( &io60_spi_driver[ 0 ] );

#ifdef __DETAILED_DEBUG
   printk( "%s: exited\n", __func__ );
#endif
}

///////////////////////////////////////////////////////////////////////////////
//
// Function: MakeDevices
//
// Description:
//  Iteritively create the minor devices for each DIO pin
//
// Parameters:
//      <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL static bool MakeDevices( struct IO60_SPI_CDEV *p_Cdev, struct class *pClass, uint8_t Count )
{
   bool     Success = true;
   uint8_t  i;
   uint32_t Err;

   for ( i = 0; i < Count; i++ )
   {
      //
      // initialize the minor device's cdev...
      //

      cdev_init( &p_Cdev[ i ].CharDev, &IO60_SPI_Fops );

      //
      // initialize any other fields in the current c_dev struct...
      //

      sema_init( &p_Cdev[ i ].Sema4, 1 );
      p_Cdev[ i ].CharDev.owner = THIS_MODULE;
      p_Cdev[ i ].CharDev.ops = &IO60_SPI_Fops;

      //
      // add the cdev to the kernel's cdev internal list
      //

      Err = cdev_add( &p_Cdev[ i ].CharDev, p_Cdev[ i ].Dev_t , 1 );

      if ( Err )
      {
         printk( "CdevSetup: Error %d adding device minor %d", Err, i );
         Success = false;
         break;
      }

      p_Cdev[ i ].pDevice = device_create( pIo60SpiDevClass, NULL, p_Cdev[ i ].Dev_t, NULL, ENUMERATED_DEVICE_STRING, i );
   }

   return Success;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function: destroy_devices
//
// Description:
//  Iteritively create the minor devices for each DIO pin
//
// Parameters:
//      <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL static void DestroyDevices( struct class *pClass, struct IO60_SPI_CDEV *p_Cdev, uint8_t Count )
{
   uint8_t  i;

#ifdef __DETAILED_DEBUG
   printk( "%s: entered\n", __func__ );
#endif

   //
   // remove the individual cdevs...
   //

   for ( i = 0; i < Count; i++ )
   {
      device_destroy( pClass, p_Cdev[ i ].Dev_t );
      cdev_del( &p_Cdev[ i ].CharDev );
   }

#ifdef __DETAILED_DEBUG
   printk( "%s: exited\n", __func__ );
#endif

}

///////////////////////////////////////////////////////////////////////////////
//
// Function: DestroyChrDevRegions
//
// Description:
//
//
// Parameters:
//      <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL static void DestroyChrDevRegions( struct IO60_SPI_CDEV *pDevArray )
{
   uint8_t i;

#ifdef __DETAILED_DEBUG
   printk( "%s: entered\n", __func__ );
#endif

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      unregister_chrdev_region( ( pDevArray++ )->Dev_t, 1 );
   }

#ifdef __DETAILED_DEBUG
   printk( "%s: exited\n", __func__ );
#endif
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// global functions
//

///////////////////////////////////////////////////////////////////////////////
//
// Function: io60_spi_release
//
// Description:
//
// Parameters:
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL int io60_spi_release( struct inode *inode, struct file *filp )
{
   struct IO60_SPI_CDEV    *pSpiCharDev = container_of( inode->i_cdev, struct IO60_SPI_CDEV, CharDev );

#ifdef __DEBUG_OPEN_CLOSE
   printk( "%s: Closing device\n",  __func__ );
#endif
   //
   // release the semaphore...
   //

   up( &( pSpiCharDev->Sema4 ) );

   return STATUS_SUCCESS;
}

//EXPORT_SYMBOL(io60_spi_release);

///////////////////////////////////////////////////////////////////////////////
//
// Function: io60_spi_open
//
// Description:
//
// Parameters:
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL int io60_spi_open( struct inode *inode, struct file *filp )
{
   int                      ReturnValue = STATUS_SUCCESS;
   struct IO60_SPI_CDEV    *pSpiCharDev = container_of( inode->i_cdev, struct IO60_SPI_CDEV, CharDev );

#ifdef __DEBUG_OPEN_CLOSE
   printk( "%s: Opening device\n",  __func__ );
#endif

   //
   // try to lock the semaphore for exclusive access to the driver's resources...
   //
   if ( down_trylock( &( pSpiCharDev->Sema4 ) ) != 0 )
       {
          ReturnValue = -ERESTARTNOHAND;
          printk( "%s: Failure obtaining device semaphore\n",  __func__ );
       }
   else
       {
          //
          // got the semaphore...
          //

       }

   return ReturnValue;
}

//EXPORT_SYMBOL(io60_spi_open);

///////////////////////////////////////////////////////////////////////////////
//
// Function: io60_spi_init
//
// Description:
//    Standard device driver initialization function.
//
// Parameters:
//    <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL int io60_spi_init( void )
{
   int16_t  RetVal;
   uint8_t  i;

#ifdef __DEBUG_LOAD_UNLOAD
   printk( "io60_spi driver loading, v%02x.%02x.%04x\n",  ( uint8_t )( ( Version & 0xff000000 ) >> 24 ), ( uint8_t )( ( Version & 0x00ff0000 ) >> 16 ), ( uint16_t )( Version & 0x0000ffff ) );
#endif
   ////////////////////////////////////////////////////////////////////////
   //
   // hook up the SPI stuff
   //
   // register spi driver
   //

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      RetVal = spi_register_driver( &io60_spi_driver[ i ] );
      if ( RetVal )
      {
         //
         // cannot register SPI driver
         //

         printk( "%s: cannot register spi driver for chip select %01d. Error code = %d\n", __func__, i, RetVal );
         return RetVal;
      }
   }

   //
   // populate bus
   //

   pSpiMaster = spi_busnum_to_master( C398_IO60_SPI_BUS_NUM );
   if ( ! pSpiMaster )
       {
          //
          // no such bus
          //

          printk( "%s: invalid spi bus\n", __func__ );
          SpiUnregsiterDrivers( &io60_spi_driver[ 0 ] );
          return -ENODEV;
       }
#ifdef __DETAILED_DEBUG
   else
       {
          printk( "%s: 'spi_busnum_to_master' returned a pointer of 0x%08x\n", __func__, ( uint32_t ) pSpiMaster );
       }
#endif

   //
   // add devices, one for each IO60 SPI chip select...
   //

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      Io60CdevArray[ i ].pSpiDevice = spi_new_device( pSpiMaster, &io60_spi_dev_info[ i ] );
      Io60CdevArray[ i ].ChipSelect = i;
      if ( ! Io60CdevArray[ i ].pSpiDevice )
          {
             //
             // device registration failed
             //

             printk( "%s: unable to add spi device\n", __func__ );
             SpiUnregsiterDrivers( &io60_spi_driver[ 0 ] );
             return -ENODEV;
          }
#ifdef __DETAILED_DEBUG
      else
          {
             printk( "%s: For Chip Select %1d, 'spi_new_device' allocated a pointer to a SPI device at 0x%08x\n", __func__, i, ( uint32_t ) Io60CdevArray[ i ].pSpiDevice );
          }
#endif
   }

   ////////////////////////////////////////////////////////
   //
   // SPI stuff is all hooked up
   //
   // get char device regions of major & minor device numbers...
   //
   //  major will be upper 16 bits of Io60CdevArray[ i ].Dev_t, minor will be lower...
   //

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      if ( alloc_chrdev_region( &Io60CdevArray[ i ].Dev_t, i, 1, DeviceName[ i ] ) != 0 )
          {
             uint8_t j;

             //
             // dealloc any that have been created...
             //

             for ( j = 0; j < i; j++ )
             {
                unregister_chrdev_region( Io60CdevArray[ j ].Dev_t, 1 );
             }

             printk( "%s: can't alloc_chrdev_region\n", __func__ );
             SpiUnregsiterDevices( &Io60CdevArray[ 0 ] );
             return -ENOMEM;
          }
#ifdef __DETAILED_DEBUG
      else
          {
             printk( "%s: 'alloc_chrdev_region' allocated device %s with a Dev_t of 0x%08x\n", __func__, DeviceName[ i ], Io60CdevArray[ i ].Dev_t );
          }
#endif
   }

   //
   // automatically create the device class under the /sys direrctory
   //

   pIo60SpiDevClass = class_create( THIS_MODULE, DriverClassName );
   if ( pIo60SpiDevClass == NULL )
       {
          printk( "%s: can't create class", __func__ );
          DestroyChrDevRegions( &Io60CdevArray[ 0 ] );
          SpiUnregsiterDevices( &Io60CdevArray[ 0 ] );
          return -ENOMEM;
       }
#ifdef __DETAILED_DEBUG
   else
       {
          printk( "%s: 'class_create' returned a class pointer of 0x%08x\n", __func__, ( uint32_t ) pIo60SpiDevClass );
       }
#endif

   //
   // create the devices and register them with the filesystem...
   //

   if ( MakeDevices( ( struct IO60_SPI_CDEV *) &Io60CdevArray, pIo60SpiDevClass, IO60_SPI_NUM_CHIP_SELECTS ) == false )
   {
      printk( "%s: failed to create & register dio devices\n", __func__ );
      DestroyChrDevRegions( ( struct IO60_SPI_CDEV *) &Io60CdevArray );
      return -ENOMEM;
   }

   //
   // initialize the semaphores used to protect the driver resources
   //

   for ( i = 0; i < IO60_SPI_NUM_CHIP_SELECTS; i++ )
   {
      sema_init( &Io60CdevArray[ i ].Sema4, 1 );
   }

#ifdef __DEBUG_LOAD_UNLOAD
   printk( "%s: initialized\n", __func__ );
#endif

   return STATUS_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function: io60_spi_uninit
//
// Description:
//     Standard driver unload function
//
// Parameters:
//     <void>
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL void io60_spi_uninit( void )
{
#ifdef __DETAILED_DEBUG
   printk( "%s: entered\n", __func__ );
#endif
   DestroyDevices( pIo60SpiDevClass, ( struct IO60_SPI_CDEV *) &Io60CdevArray, IO60_SPI_NUM_CHIP_SELECTS );
   class_destroy( pIo60SpiDevClass );
   DestroyChrDevRegions( &Io60CdevArray[ 0 ] );
   SpiUnregsiterDevices( &Io60CdevArray[ 0 ] );
#ifdef __DEBUG_LOAD_UNLOAD
   printk( "io60_spi drivers v%02x.%02x.%04x unloaded\n",  ( uint8_t )( ( Version & 0xff000000 ) >> 24 ),
            ( uint8_t )( ( Version & 0x00ff0000 ) >> 16 ), ( uint16_t )( Version & 0x0000ffff ) );
#endif
}

///////////////////////////////////////////////////////////////////////////////
//
// Function: io60_spi_ioctl
//
// Description:
//    Top level IOCTL code handler
//
// Parameters:
//
// Notes:
//
///////////////////////////////////////////////////////////////////////////////
__SET_LOCAL_OPTIMIZATION_LEVEL long io60_spi_ioctl( struct file *pFile, unsigned int ioctl_num, unsigned long ioctl_param )
{
   long                     RetVal = 0;
   struct inode            *pInode;
   struct IO60_SPI_CDEV    *pSpiCdev;     // for device information...

#ifdef __DEBUG_IOCTL_CALLS
    printk("\n%s: ioctl code:%08x\n", __func__, ioctl_num );
#endif

   //
   // Find the device
   //

   pInode = pFile->f_path.dentry->d_inode;
   pSpiCdev = container_of( pInode->i_cdev, struct IO60_SPI_CDEV, CharDev );

#ifdef __DEBUG_IOCTL_CALLS
   printk("%s: IOCTL code targeting device %s\n", __func__, DeviceName[ pSpiCdev->ChipSelect ] );
#endif

   switch ( ioctl_num )
   {
      case IOCTL_GET_VERSION:                  // tested
      {
#ifdef __DEBUG_IOCTL_CALLS
         printk("IOCTL_GET_VERSION\n" );
#endif
         RetVal = copy_to_user( ( pvoid ) ioctl_param, ( pvoid ) &Version, sizeof( uint32_t ) );
         break;
      }

      case IOCTL_SPI_WRITE:
      {
         SPI_XFER_STRUCT XferData;

#ifdef __DEBUG_SPI_OP_IOCTL_CALLS
         printk("IOCTL_SPI_WRITE\n" );
#endif
         RetVal = copy_from_user( ( pvoid ) &XferData, ( pvoid ) ioctl_param, sizeof( SPI_XFER_STRUCT ) );
         if ( RetVal != 0 )
             {
                //
                // couldn't read the user data, so flag an error and exit...
                //

                printk("%s: error copying SPI transfer structure from user space into kernel space.\n", __func__ );
                RetVal = -STATUS_BAD_DATA_COPY;
             }
         else
             {
                //
                // got the user space data, so perform the SPI transfer...
                //

                puint8_t    pLocalBuffer;
                uint8_t     k;

#ifdef __DEBUG_DISPLAY_XFER_DATA
                pLocalBuffer = XferData.pTxBuffer;

                printk( "\n" );
                printk( " TxBuffer Contents:\n" );
                for ( k = 0; k < XferData.TxLen; k++ )
                {
                   printk( " %01x         %02x\n", k, *pLocalBuffer++ );
                }
                printk( "\n" );
#endif
                pLocalBuffer = kmalloc( XferData.TxLen, ( GFP_DMA | GFP_KERNEL ) );

                if ( pLocalBuffer )
                {
                   for ( k = 0; k < XferData.TxLen; k++ )
                   {
                      *( pLocalBuffer + k ) = *( XferData.pTxBuffer + k );
                   }

                   RetVal = spi_write( pSpiCdev->pSpiDevice, pLocalBuffer, XferData.TxLen );
                }

                kfree( pLocalBuffer );
             }

         break;
      }

      case IOCTL_SPI_WRITE_THEN_READ:
      {
         SPI_XFER_STRUCT XferData;

#ifdef __DEBUG_SPI_OP_IOCTL_CALLS
         printk("IOCTL_SPI_WRITE_THEN_READ\n" );
#endif
         RetVal = copy_from_user( ( pvoid ) &XferData, ( pvoid ) ioctl_param, sizeof( SPI_XFER_STRUCT ) );
         if ( RetVal != 0 )
             {
                //
                // couldn't read the user data, so flag an error and exit...
                //

                printk("%s: error copying SPI transfer structure from user space into kernel space.\n", __func__ );
                RetVal = -STATUS_BAD_DATA_COPY;
             }
         else
             {
                //
                // got the user space data, so perform the SPI transfer...
                //
                uint8_t     k;
                puint8_t    pLocalTxBuffer,
                            pLocalRxBuffer;

#ifdef __DEBUG_DISPLAY_XFER_DATA
                pLocalTxBuffer = XferData.pTxBuffer;
                printk( "\n" );
                printk( " TxBuffer Contents:\n" );
                for ( k = 0; k < XferData.TxLen; k++ )
                {
                   printk( " %01x         %02x\n", k, *pLocalTxBuffer++ );
                }
                printk( "\n" );
#endif
                pLocalTxBuffer = kmalloc( XferData.TxLen, ( GFP_DMA | GFP_KERNEL ) );
                pLocalRxBuffer = kmalloc( XferData.RxLen, ( GFP_DMA | GFP_KERNEL ) );

                if ( pLocalTxBuffer && pLocalRxBuffer )
                    {
                       for ( k = 0; k < XferData.TxLen; k++ )
                       {
                          *( pLocalTxBuffer + k ) = *( XferData.pTxBuffer + k );
                       }

                       RetVal = spi_write_then_read( pSpiCdev->pSpiDevice, pLocalTxBuffer, XferData.TxLen, pLocalRxBuffer, XferData.RxLen );

                       for ( k = 0; k < XferData.RxLen; k++ )
                       {
                          *( XferData.pRxBuffer + k ) = *( pLocalRxBuffer + k );
                       }

#ifdef __DEBUG_DISPLAY_XFER_DATA
                       if ( XferData.RxLen )
                       {
                          printk( " RxBuffer Contents:\n" );
                          for ( k = 0; k < XferData.RxLen; k++ )
                          {
                             printk( " %01x         %02x\n", k, *pLocalRxBuffer++ );
                          }
                          printk( "\n" );
                       }
#endif
                    }
                else
                    {
                       printk("%s: error allocating local buffers\n", __func__ );
                       RetVal = -STATUS_INSUFFICIENT_RESOURCES;
                    }

                kfree( pLocalTxBuffer );
                kfree( pLocalRxBuffer );
             }

         break;
      }

      case IOCTL_SPI_DAC_REG_READ:
      {
         SPI_XFER_STRUCT XferData;

#ifdef __DEBUG_SPI_OP_IOCTL_CALLS
         printk("IOCTL_SPI_SYNC_XFER\n" );
#endif
         RetVal = copy_from_user( ( pvoid ) &XferData, ( pvoid ) ioctl_param, sizeof( SPI_XFER_STRUCT ) );
         if ( RetVal != 0 )
             {
                //
                // couldn't read the user data, so flag an error and exit...
                //

                printk("%s: error copying SPI transfer structure from user space into kernel space.\n", __func__ );
                RetVal = -STATUS_BAD_DATA_COPY;
             }
         else
             {
                //
                // got the user space data, so perform the SPI transfer...
                //

                uint8_t             k,
                                    RxBuffer[ 4 ],
                                    TxBuffer_1[ 4 ],
                                    TxBuffer_2[ 4 ];

                struct spi_message  SpiMsg;
                struct spi_transfer SpiXfers[ 2 ] = {
                                                       {
                                                          .tx_buf = &TxBuffer_1,
                                                          .len = 4,
                                                          .cs_change = 1,
                                                       },

                                                       {
                                                          .tx_buf = &TxBuffer_2,
                                                          .rx_buf = RxBuffer,
                                                          .len = 4,
                                                       },
                                                    };

#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( " TxBuffer Contents:\n" );
#endif
                for ( k = 0; k < 4; k++ )
                {
                   TxBuffer_1[ k ] = *( XferData.pTxBuffer + k );
#ifdef __DEBUG_DISPLAY_XFER_DATA
                   printk( " %01x         %02x\n", k, TxBuffer_1[ k ] );
#endif
                }

#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( "\n" );
#endif
                TxBuffer_2[ 0 ] = TxBuffer_1[ 0 ];           // copy virtual chip select...
                TxBuffer_2[ 1 ] = AD5755_NOP_1;
                TxBuffer_2[ 2 ] = AD5755_NOP_2;
                TxBuffer_2[ 3 ] = 0;

                spi_message_init( &SpiMsg );
                spi_message_add_tail( &SpiXfers[ 0 ], &SpiMsg );
                spi_message_add_tail( &SpiXfers[ 1 ], &SpiMsg );

                RetVal = spi_sync( pSpiCdev->pSpiDevice, &SpiMsg );

#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( " RxBuffer Contents:\n" );
#endif
                for ( k = 0; k < XferData.TxLen; k++ )
                {
                   *( XferData.pRxBuffer + k ) = RxBuffer[ k ];
#ifdef __DEBUG_DISPLAY_XFER_DATA
                   printk( " %01x         %02x\n", k, RxBuffer[ k ] );
#endif
                }
#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( "\n" );
#endif
             }

         break;
      }

      case IOCTL_SPI_READ_WHILE_WRITING:
      {
         SPI_XFER_STRUCT XferData;

#ifdef __DEBUG_SPI_OP_IOCTL_CALLS
         printk("IOCTL_SPI_SYNC_XFER\n" );
#endif
         RetVal = copy_from_user( ( pvoid ) &XferData, ( pvoid ) ioctl_param, sizeof( SPI_XFER_STRUCT ) );
         if ( RetVal != 0 )
             {
                //
                // couldn't read the user data, so flag an error and exit...
                //

                printk("%s: error copying SPI transfer structure from user space into kernel space.\n", __func__ );
                RetVal = -STATUS_BAD_DATA_COPY;
             }
         else
             {
                //
                // got the user space data, so perform the SPI transfer...
                //

                uint8_t             k,
                                    RxBuffer[ 4 ],
                                    TxBuffer[ 4 ];

                struct spi_message  SpiMsg;
                struct spi_transfer SpiXfers[ 1 ] = {
                                                       {
                                                          .tx_buf = &TxBuffer,
                                                          .rx_buf = &RxBuffer,
                                                          .len = 4,
                                                       },
                                                    };

#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( " TxBuffer Contents:\n" );
#endif
                for ( k = 0; k < 4; k++ )
                {
                   TxBuffer[ k ] = *( XferData.pTxBuffer + k );
#ifdef __DEBUG_DISPLAY_XFER_DATA
                   printk( " %01x         %02x\n", k, TxBuffer[ k ] );
#endif
                }
#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( "\n" );
#endif
                spi_message_init( &SpiMsg );
                spi_message_add_tail( &SpiXfers[ 0 ], &SpiMsg );

                RetVal = spi_sync( pSpiCdev->pSpiDevice, &SpiMsg );

#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( " RxBuffer Contents:\n" );
#endif
                for ( k = 0; k < XferData.TxLen; k++ )
                {
                   *( XferData.pRxBuffer + k ) = RxBuffer[ k ];
#ifdef __DEBUG_DISPLAY_XFER_DATA
                   printk( " %01x         %02x\n", k, RxBuffer[ k ] );
#endif
                }
#ifdef __DEBUG_DISPLAY_XFER_DATA
                printk( "\n" );
#endif
             }
      }

      default:
      {
         printk("%s: undefined ioctl of 0x%08x\n", __func__, ioctl_num );
         RetVal = -EINVAL;
      }

   }        // switch ( ioctl_num )

#ifdef __DETAILED_DEBUG
    printk("%s: exiting with a return value of %08x\n", __func__, ( unsigned int ) RetVal );
#endif

   return RetVal;
}
