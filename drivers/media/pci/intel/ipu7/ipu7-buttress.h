/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_BUTTRESS_H
#define IPU7_BUTTRESS_H

#include <linux/completion.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include "../ipu/ipu.h"

int ipu_buttress_ipc_reset(struct ipu_device *isp,
			   struct ipu_buttress_ipc *ipc);
int ipu_buttress_power(struct device *dev, struct ipu_buttress_ctrl *ctrl, bool on);
int ipu_buttress_powerup(struct device *dev,
			 const struct ipu_buttress_ctrl *ctrl);
int ipu_buttress_powerdown(struct device *dev,
			   const struct ipu_buttress_ctrl *ctrl);
bool ipu_buttress_get_secure_mode(struct ipu_device *isp);
int ipu_buttress_authenticate(struct ipu_device *isp);
int ipu_buttress_reset_authentication(struct ipu_device *isp);
bool ipu_buttress_auth_done(struct ipu_device *isp);
int ipu_buttress_get_isys_freq(struct ipu_device *isp, u32 *freq);
int ipu_buttress_get_psys_freq(struct ipu_device *isp, u32 *freq);
int ipu_buttress_start_tsc_sync(struct ipu_device *isp);

irqreturn_t ipu_buttress_isr(int irq, void *isp_ptr);
irqreturn_t ipu_buttress_isr_threaded(int irq, void *isp_ptr);
int ipu_buttress_init(struct ipu_device *isp);
void ipu_buttress_exit(struct ipu_device *isp);
void ipu_buttress_csi_port_config(struct ipu_device *isp,
				  u32 legacy, u32 combo);
void ipu_buttress_restore(struct ipu_device *isp);
void ipu_buttress_wakeup_is_uc(const struct ipu_device *isp);
void ipu_buttress_wakeup_ps_uc(const struct ipu_device *isp);
#endif /* IPU7_BUTTRESS_H */
