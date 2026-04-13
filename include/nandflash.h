/*
 * Copyright (C) 2012 Microchip Technology Inc. and its subsidiaries
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __NANDFLASH_H__
#define __NANDFLASH_H__

extern int load_nandflash(struct image_info *image);
extern int nand_get_boot_flag(unsigned char *flag);

#endif /* #ifndef __NANDFLASH_H__ */
