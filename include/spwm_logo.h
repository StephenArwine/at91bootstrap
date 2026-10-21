// Copyright (C) 2026 ZonedIn
//
// SPDX-License-Identifier: MIT

#ifndef __SPWM_LOGO_H__
#define __SPWM_LOGO_H__

/* Put the boot logo on the SM16169 HUB75 panel and leave the hardware scan
 * engine (PWM ch0-3 + XDMAC1 ch31) running. Call after hw_postinit(). */
void spwm_logo_show(void);

#endif /* __SPWM_LOGO_H__ */
