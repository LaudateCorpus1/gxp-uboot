// SPDX-License-Identifier: GPL-2.0
/*
 * lowlevel.c
 *
 * (C) Copyright 2019 Hewlett Packard Enterprise Development LP.
 * Author: Gilbert Chen <gilbert.chen@hpe.com>
 */
#include <asm/io.h>

void __attribute__((naked)) __attribute__((no_instrument_function)) lowlevel_init(void)
{
	//turn LED1 on
	writeb(0x01, 0xd1000004);

#ifndef CONFIG_GXP_ECC
	//turn off ecc
	writel(0x0, 0xc0000064);
#endif
}

