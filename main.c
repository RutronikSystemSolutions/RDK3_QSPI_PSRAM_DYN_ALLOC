/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3_PSRAM_DYN_ALLOC
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-08-19
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdlib.h"
#include "cy_retarget_io.h"
#include "cycfg_qspi_memslot.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     /* SMIF interrupt priority */
#define NUM_BYTES_PER_LINE		(16u)	 /* Used when array of data is printed on the console */
#define TIMEOUT_1_MS            (1000ul) /* 1 ms timeout for all blocking functions */
#define QSPI_FREQ            	(80000000ul)

/*LED indication control*/
#define LED_IND_OFF          	(0x03)
#define LED_IND_RED          	(0x01)
#define LED_IND_GREEN        	(0x02)

#define MIN_ALLOC_BYTES			(1u)
#define MAX_ALLOC_BYTES			(102400u)

#define TEST_ALLOC_MEM
//#define ALLOC_FIXED_SIZE		(524288u)

void handle_error(void);
void Isr_SMIF(void);
void Init_SMIF(void);
void status_led (uint32_t status);

/*QSPI PSRAM object*/
cyhal_qspi_t qspi_psram_obj;

/* SMIF configuration parameters */
cy_stc_smif_config_t SMIFConfig =
{
    /* .mode           */ CY_SMIF_NORMAL,      /* Mode of operation */
    /* .deselectDelay  */ 2U,      			/* Minimum duration of SPI deselection */
    /* .rxClockSel     */ CY_SMIF_SEL_INVERTED_INTERNAL_CLK,     /* Clock source for the receiver clock */
    /* .blockEvent     */ CY_SMIF_BUS_ERROR    /* What happens when there is a read
                                                * to an empty RX FIFO or write to a full TX FIFO
                                                */
};

/*Random Number Generator object*/
cyhal_trng_t trng_obj;

/*QSPI pins configuration structure*/
cyhal_qspi_slave_pin_config_t qspi_pins =
{
		.io[0] = QSPI_IO0,
		.io[1] = QSPI_IO1,
		.io[2] = QSPI_IO2,
		.io[3] = QSPI_IO3,
		.io[4] = NC,
		.io[5] = NC,
		.io[6] = NC,
		.io[7] = NC,
		.ssel = PSRAM_SSEL
};

int main(void)
{
    cy_rslt_t result;
    uint8_t *addr = NULL;
    uint32_t rnd_num = 0;
#ifdef TEST_ALLOC_MEM
    uint32_t i;
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize QSPI PSRAM Interface*/
    result = cyhal_qspi_init(&qspi_psram_obj, QSPI_CLK, &qspi_pins, QSPI_FREQ, 0, NULL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initializes SMIF block*/
    Init_SMIF();

    /*Enter XIP mode*/
    Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_MEMORY);

    /*DYNAMIC MEMORY ALLOCATION IN PSRAM IS READY*/

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RutDevKit-PSoC62 QSPI PSRAM Dynamic Allocation Example.\r\n");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /* Initialize the true random number generator block */
    result = cyhal_trng_init(&trng_obj);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    for (;;)
    {
    	/*Select between random or fixed memory allocation*/
#ifndef ALLOC_FIXED_SIZE
        /* Generate a limited range true random number */
    	while((rnd_num < MIN_ALLOC_BYTES) || (rnd_num > MAX_ALLOC_BYTES))
    	{
    		rnd_num = cyhal_trng_generate(&trng_obj);
    		/*Make the generation faster*/
    		rnd_num &= 0xFFFFFF;
    	}
#else
    	rnd_num = ALLOC_FIXED_SIZE;
#endif

    	/*Allocate random amount of bytes*/
    	addr = malloc(rnd_num);

    	if(addr != NULL) /*Allocation successful*/
    	{
    		status_led (LED_IND_GREEN);
    		printf("Bytes allocated: %u @ 0x%X08\r\n", (unsigned int)rnd_num, (unsigned int)addr);

#ifdef TEST_ALLOC_MEM
    		/*Test the allocated memory with a random pattern*/
    		memset(addr, rnd_num&0x000000FF, rnd_num);
    		for(i = 0; i < rnd_num; i++)
    		{
    			if(addr[i] != (rnd_num&0x000000FF))
    			{
    				status_led (LED_IND_RED);
    				printf("Memory ERROR @ 0x%X08\r\n", (unsigned int)addr);
    				break;
    			}
    		}
#endif
    	}
    	else /*Allocation failure*/
    	{
    		status_led (LED_IND_RED);
    		printf("FAILED to allocate %u bytes\r\n", (unsigned int)rnd_num);
    	}

    	rnd_num = 0;
    	free(addr);
    }
}

/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(qspi_psram_obj.base, &qspi_psram_obj.context);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
void Init_SMIF(void)
{
	/* Initialize SMIF interrupt */

	      cy_stc_sysint_t smifIntConfig =
		  {
		   .intrSrc = 2u,
		   .intrPriority = SMIF_PRIORITY
		  };

	      cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);

	      if(0u != intrStatus)
		    {
		        status_led (LED_IND_RED);

		        for(;;)
		          {
		             /*Waits forever when SMIF initialization error occurs*/
		          }
		    }

		  /* Initialize SMIF */
		  cy_en_smif_status_t smifStatus;
		  smifStatus = Cy_SMIF_Init(qspi_psram_obj.base, &SMIFConfig, TIMEOUT_1_MS, &qspi_psram_obj.context);

		  if(0u != smifStatus)
		    {
		        status_led (LED_IND_RED);

		   for(;;)
		         {
		             /*Waits forever when SMIF initialization error occurs*/
		         }
		     }
	     /* Configure slave select and data select. These settings depend on the pins
	      * selected in the Device and QSPI configurators.
	      */
		   Cy_SMIF_SetDataSelect(qspi_psram_obj.base, qspi_psram_obj.slave_select, APS1604M_3SQR_ZR_SlaveSlot_0.dataSelect);
	       Cy_SMIF_Enable(qspi_psram_obj.base, &qspi_psram_obj.context);
	       Cy_SMIF_Memslot_Init(qspi_psram_obj.base, (cy_stc_smif_block_config_t *)&smifBlockConfig, &qspi_psram_obj.context);
	       Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_NORMAL);
	       NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */
}

/*******************************************************************************
* Function Name: status_led
********************************************************************************
*
* This function drives the red/green status of RGB
* *
* \param status - The led status (RGB_GLOW_OFF, RGB_GLOW_RED, or RGB_GLOW_GREEN)
*
*******************************************************************************/
void status_led (uint32_t status)
{
	if (status == LED_IND_RED)
	{
		cyhal_gpio_write(LED3, CYBSP_LED_STATE_ON);
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
	}
	else if (status == LED_IND_GREEN)
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
		cyhal_gpio_write(LED3, CYBSP_LED_STATE_OFF);
	}
	else
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
		cyhal_gpio_write(LED3, CYBSP_LED_STATE_OFF);
	}
}

/*SysLib processing errors report*/
void Cy_SysLib_ProcessingFault(void)
{
	printf("SysLib FAULT\r\n");
	return;
}


/*If initialization fails, program ends up here.*/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
