/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module
 *   @details This module contains the configurations needed for IIO application
********************************************************************************
 * Copyright (c) 2021-24 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include <math.h>
#include "app_config.h"
#include "common.h"
#include "no_os_error.h"
#include "no_os_uart.h"
#include "no_os_irq.h"
#include "no_os_gpio.h"
#include "no_os_i2c.h"
#include "no_os_eeprom.h"
#include "no_os_tdm.h"
#include "no_os_pwm.h"
#if (ACTIVE_IIO_CLIENT == IIO_CLIENT_LOCAL)
#include "pl_gui_events.h"
#include "pl_gui_views.h"
#endif

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* UART init parameters */
struct no_os_uart_init_param uart_init_params = {
	.device_id = NULL,
	.baud_rate = IIO_UART_BAUD_RATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.platform_ops = &uart_ops,
	.extra = &uart_extra_init_params
};


/* UART descriptor */
struct no_os_uart_desc *uart_desc;


/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/




/**
 * @brief 	Initialize the UART peripheral
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t init_uart(void)
{
	return no_os_uart_init(&uart_desc, &uart_init_params);
}



/**
 * @brief 	Initialize the system peripherals
 * @return	0 in case of success, negative error code otherwise
 */
int32_t init_system(void)
{
	int32_t ret = 0;

#if (ACTIVE_PLATFORM == STM32_PLATFORM)
	stm32_system_init();
#endif

	//ret = init_gpio();
	if (ret) {
		return ret;
	}

	ret = init_uart();
	if (ret) {
		return ret;
	}

	//ret = gpio_trigger_init();
	if (ret) {
		return ret;
	}

	//ret = init_interrupt();
	if (ret) {
		return ret;
	}

#if (INTERFACE_MODE == TDM_MODE)
	//ret = init_tdm();
	if (ret) {
		return ret;
	}
#endif

#if defined(USE_SDRAM)
	//ret = sdram_init();
	if (ret) {
		return ret;
	}
#endif

	//ret = eeprom_init(&eeprom_desc, &eeprom_init_params);
	if (ret) {
		return ret;
	}

	//ret = tx_trigger_init();
	if (ret) {
		return ret;
	}

	return 0;
}


