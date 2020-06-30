// SPDX-License-Identifier: GPL-2.0
/*
 * GXP timer driver
 *
 * (C) Copyright 2019 Hewlett Packard Enterprise Development LP.
 * Author: Gilbert Chen <gilbert.chen@hpe.com>
 */

#include <common.h>
#include <asm/io.h>

#define GXP_CCR	0xC0000000

void reset_cpu(ulong ignored)
{
	writel(1, GXP_CCR);

	while (1)
		;	/* loop forever till reset */
}
