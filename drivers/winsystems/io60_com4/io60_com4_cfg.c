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
//	Name	 : io60_com4_cfg.c
//
//	Project	 : IO60-COM4 Configuration Linux Device Driver
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

#ifndef __KERNEL__
#  define __KERNEL__
#endif

#ifndef MODULE
#  define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>

#define SUCCESS					0
#define MAX7310_1_ADDR			0x18
#define MAX7310_2_ADDR			0x19
#define MAX7310_3_ADDR			0x1A
#define	IO60_COM4_I2C_CLK_EN			IMX_GPIO_NR(7, 0)
#define IO60_COM4_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(9, 0)
#define IO60_COM4_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(9, 8)
#define IO60_COM4_MAX7310_3_BASE_ADDR	IMX_GPIO_NR(9, 16)

// max7310 default sttings
static int max7310_1_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/* 0 UART1_SLEW */
	/* 1 UART1_TERM */
	/* 2 UART1_M1 */
	/* 3 UART1_M0 */
	/* 4 UART2_SLEW */
	/* 5 UART2_TERM */
	/* 6 UART2_M1 */
	/* 7 UART2_M0 */

	int max7310_gpio_value[] = {
		0, 0, 0, 1, 0, 0, 0, 1,
	};
	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 1 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_1_platdata = {
	.gpio_base	= IO60_COM4_MAX7310_1_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_1_setup,
};

static int max7310_2_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/* 0 UART3_SLEW */
	/* 1 UART3_TERM */
	/* 2 UART3_M1 */
	/* 3 UART3_M0 */
	/* 4 UART4_SLEW */
	/* 5 UART4_TERM */
	/* 6 UART4_M1 */
	/* 7 UART4_M0 */

	int max7310_gpio_value[] = {
		0, 0, 0, 1, 0, 0, 0, 1,
	};
	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 2 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_2_platdata = {
	.gpio_base	= IO60_COM4_MAX7310_2_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_2_setup,
};

static int max7310_3_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	/* 0 UART1_EN */
	/* 1 UART2_EN */
	/* 2 UART3_EN */
	/* 3 UART4_EN */
	/* 4 UNUSED */
	/* 5 UNUSED */
	/* 6 UNUSED */
	/* 7 UNUSED */

	int max7310_gpio_value[] = {
		1, 1, 1, 1, 0, 0, 0, 0,
	};
	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 3 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_3_platdata = {
	.gpio_base	= IO60_COM4_MAX7310_3_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_3_setup,
};

// i2c info
struct i2c_board_info io60_com4_cfg_info[] = {
	{
		I2C_BOARD_INFO("max7310", MAX7310_1_ADDR),
		.platform_data = &max7310_1_platdata
	},
	{
		I2C_BOARD_INFO("max7310", MAX7310_2_ADDR),
		.platform_data = &max7310_2_platdata
	},
	{
		I2C_BOARD_INFO("max7310", MAX7310_3_ADDR),
		.platform_data = &max7310_3_platdata
	},
};

// probe search list
unsigned short const io60_com4_addr_list[4] = {
	MAX7310_1_ADDR,
	MAX7310_2_ADDR,
	MAX7310_3_ADDR,
	I2C_CLIENT_END
};

struct i2c_client *client[3];

// Driver Declarations
struct i2c_driver io60_com4_cfg_driver = {
	.driver = {
		.name   = "io60_com4_cfg",
	},
};

// Init Module
static int __init io60_com4_cfg_init(void)
{
	struct i2c_adapter *bus;
	int i, status;

	// register the i2c chip driver
	status = i2c_add_driver(&io60_com4_cfg_driver); 
	if (status) {
		// cannot register driver
		printk("<1>io60_com4_cfg: cannot register driver\n");
		return status;
	}

	// populate bus
	bus = i2c_get_adapter(2);
	if (!bus) {
		// no such bus
		printk("<1>io60_com4_cfg: invalid i2c adapter\n");
		status = -ENODEV;
		goto nobus;
	}

	// add devices
	for (i=0; i<3; i++)
	{	
		client[i] = i2c_new_probed_device(bus, &io60_com4_cfg_info[i], io60_com4_addr_list,
											i2c_probe_func_quick_read);

		if (!client[i]) {
			// device registration failed
			printk("<1>io60_com4_cfg: unable to add i2c device %d\n", i);
			status = -ENODEV;
			goto nodev;
		}
	}

	// enable gpio for i2c clock
	status = gpio_request(IO60_COM4_I2C_CLK_EN, "i2c_clk_enbl");
	if (status) {
		// cannot register driver
		printk("<1>io60_com4_cfg: cannot obtain gpio\n");
		goto nogpio;
	}

	gpio_direction_output(IO60_COM4_I2C_CLK_EN, 1);
	gpio_export(IO60_COM4_I2C_CLK_EN, 0);

	printk("<1>io60_com4_cfg: init completed!\n");

	return SUCCESS;

nogpio:
	for (i = 0; i < 24; ++i) {
		gpio_unexport(IO60_COM4_MAX7310_1_BASE_ADDR + i);
		gpio_free(IO60_COM4_MAX7310_1_BASE_ADDR + i);
	}

	for (i=0; i<3; i++)
	{	
		i2c_unregister_device(client[i]);
	}
nodev:
nobus:
	i2c_del_driver(&io60_com4_cfg_driver); 

	return status;
}

// Cleanup Module
static void __exit io60_com4_cfg_exit(void)
{
	int i;

	printk("<1>io60_com4_cfg: exit executed\n");

	// free gpio
	gpio_unexport(IO60_COM4_I2C_CLK_EN);
	gpio_free(IO60_COM4_I2C_CLK_EN);

	for (i = 0; i < 24; ++i) {
		gpio_unexport(IO60_COM4_MAX7310_1_BASE_ADDR + i);
		gpio_free(IO60_COM4_MAX7310_1_BASE_ADDR + i);
	}

	// unregister devices
	for (i=0; i<3; i++)
	{	
		i2c_unregister_device(client[i]);
	}

	// remove driver
	i2c_del_driver(&io60_com4_cfg_driver); 
}  

module_init(io60_com4_cfg_init);
module_exit(io60_com4_cfg_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WinSystems,Inc. IO60-COM4-CFG Driver");
MODULE_AUTHOR("Paul DeMetrotion");
