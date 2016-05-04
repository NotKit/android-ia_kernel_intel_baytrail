/*
 * Copyright (C) 2015 Jolla Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Kalle Jokiniemi <kalle.jokiniemi@jolla.com>
 *
 */

#ifndef _EDP_BRIDGE_TC358860_H_
#define _EDP_BRIDGE_TC358860_H_

#include <drm/drmP.h>
#include "intel_drv.h"

#ifdef CONFIG_SUPPORT_EDP_BRIDGE_TC358860
int tc358860_has_hw(void);
void tc358860_bridge_enable(struct drm_device *dev);
void tc358860_bridge_disable(struct drm_device *dev);
int tc358860_init(struct drm_device *dev);
void tc358860_send_init_cmd1(struct intel_dp *intel_dp);
void tc358860_send_init_cmd2(struct intel_dp *intel_dp);
void tc358860_send_init_cmd3(void);
void tc358860_cmd3_work_fboot(void);
struct edid *tc358860_get_edid(void);
#else
static inline int tc358860_has_hw(void) { return 0; }
static inline void tc358860_bridge_enable(struct drm_device *dev) {}
static inline void tc358860_bridge_disable(struct drm_device *dev) {}
static inline int tc358860_init(struct drm_device *dev) { return 0; }
static inline void tc358860_send_init_cmd1(struct intel_dp *intel_dp) {}
static inline void tc358860_send_init_cmd2(struct intel_dp *intel_dp) {}
static inline void tc358860_send_init_cmd3(void) {}
static inline void tc358860_cmd3_work_fboot(void) {}
static inline struct edid *tc358860_get_edid(void) { return NULL; }
#endif


#endif
