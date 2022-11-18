/*
 * sbrk.c
 *
 *  Created on: 2021-10-20
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdlib.h"
#include "errno.h"

#define PSRAM_START_ADDR 	0x18000000
#define PSRAM_END_ADDR		0x187FFFFF

void *_sbrk(int incr)
{
    static uint32_t *heap_end = NULL;
    uint32_t *prev_heap_end = NULL;

    /* Initialize first time round */
	if (heap_end == 0)
	{
		heap_end = (uint32_t*)PSRAM_START_ADDR;
	}

	/*Expand the heap*/
	prev_heap_end = heap_end;
	heap_end += incr;

	/*Out of memory check*/
	if( heap_end > (uint32_t*)PSRAM_END_ADDR)
	{
		errno = ENOMEM;
		return (char*)-1;
	}

	/*Return previous heap end*/
	return (void *) prev_heap_end;
}

