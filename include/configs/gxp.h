
// SPDX-License-Identifier: GPL-2.0
/*
 * GXP board
 *
 * (C) Copyright 2019 Hewlett Packard Enterprise Development LP.
 * Author: Gilbert Chen <gilbert.chen@hpe.com>
 */

#ifndef _GXP_H_
#define _GXP_H_

#define CONFIG_SKIP_LOWLEVEL_INIT

#define CONFIG_SYS_MALLOC_LEN   0x4000000
#define CONFIG_SYS_INIT_SP_ADDR 0x2000000

#ifdef CONFIG_TARGET_GXP
#ifdef CONFIG_GXP_ECC
    #define CONFIG_SYS_SDRAM_SIZE       0x0f800000
#else
    #define CONFIG_SYS_SDRAM_SIZE       0x1f000000
#endif

#ifdef CONFIG_GXP_VROM_64MB
    #undef CONFIG_SYS_SDRAM_SIZE
    #ifdef CONFIG_GXP_ECC
        #define CONFIG_SYS_SDRAM_SIZE   0x0c000000
    #else
        #define CONFIG_SYS_SDRAM_SIZE   0x18000000
    #endif
#endif

#ifdef CONFIG_GXP_VROM_32MB
    #undef CONFIG_SYS_SDRAM_SIZE
    #ifdef CONFIG_GXP_ECC
        #define CONFIG_SYS_SDRAM_SIZE   0x0e000000
    #else
        #define CONFIG_SYS_SDRAM_SIZE   0x1c000000
    #endif
#endif
#endif

#ifdef CONFIG_TARGET_GXP2
    #define CONFIG_SYS_SDRAM_SIZE   0x1b200000

#ifdef CONFIG_GXP_VROM_64MB
    #undef CONFIG_SYS_SDRAM_SIZE
    #define CONFIG_SYS_SDRAM_SIZE   0x14000000
#endif

#ifdef CONFIG_GXP_VROM_32MB
    #undef CONFIG_SYS_SDRAM_SIZE
    #define CONFIG_SYS_SDRAM_SIZE   0x18000000
#endif
#endif

#define CONFIG_SYS_SDRAM_BASE   0x40000000
#define CONFIG_SYS_LOAD_ADDR    0x40100000
#define CONFIG_BOOTCOMMAND  "run spiboot"

#define CONFIG_EXTRA_ENV_SETTINGS \
    "recover_file=openbmc-hpe-recovery-image.mtd\0" \
    "recover_cmd=usb start; " \
    "mw.b 0xD100000D 0x40; " \
    "if fatload usb 0 0x50000000 $recover_file 0x4C0000 0x80000; then " \
        "setenv bootargs console=ttyS0,115200 recovery; " \
        "setenv force_recovery; " \
        "saveenv; " \
        "bootm  0x50000000; " \
    "else " \
        "while itest 0 < 1; do " \
            "mw.b 0xd1000005 0xc0; " \
            "sleep .1; " \
            "mw.b 0xd1000005 0x00; " \
            "sleep .1; " \
        "done; " \
    "fi; " \
    "reset;\0" \
    "spiboot=if itest.b *0xD10000B2 == 6; then " \
        "run recover_cmd;" \
    "fi;" \
    "if printenv force_recovery; then " \
        "run recover_cmd; " \
    "else " \
        "bootm 0xfc080000; " \
        "run recover_cmd; " \
    "fi;\0"

/*--------------------------------------------------------------------------*/
/*              Network Configuration             */
/*--------------------------------------------------------------------------*/
#define CONFIG_PHY_ADDR             0
#define CONFIG_SYS_I2C_SPEED            100000
#define CONFIG_I2C_MULTI_BUS

#endif
