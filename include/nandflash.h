/*
 * Copyright (C) 2012 Microchip Technology Inc. and its subsidiaries
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __NANDFLASH_H__
#define __NANDFLASH_H__

extern int load_nandflash(struct image_info *image);
extern int nand_get_boot_flag(unsigned char *flag);
extern int nand_get_boot_b_marker(unsigned char *marker);
extern int nand_get_boot_b_tries(unsigned char *tries);
extern int nand_write_boot_control(unsigned char flag,
				   unsigned char marker,
				   unsigned char tries);

#endif /* #ifndef __NANDFLASH_H__ */
