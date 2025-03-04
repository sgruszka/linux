/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013--2024 Intel Corporation */

#ifndef IPU6_BUTTRESS_H
#define IPU6_BUTTRESS_H

#include <linux/completion.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/mutex.h>

struct device;
struct firmware;

struct ipu_device;
struct ipu_bus_device;
struct ipu_buttress_ipc;
struct ipu_buttress_ctrl;

#define BUTTRESS_PS_FREQ_STEP		25U
#define BUTTRESS_MIN_FORCE_PS_FREQ	(BUTTRESS_PS_FREQ_STEP * 8)
#define BUTTRESS_MAX_FORCE_PS_FREQ	(BUTTRESS_PS_FREQ_STEP * 32)

#define BUTTRESS_IS_FREQ_STEP		25U
#define BUTTRESS_MIN_FORCE_IS_FREQ	(BUTTRESS_IS_FREQ_STEP * 8)
#define BUTTRESS_MAX_FORCE_IS_FREQ	(BUTTRESS_IS_FREQ_STEP * 22)

int ipu6_buttress_ipc_reset(struct ipu_device *isp,
			    struct ipu_buttress_ipc *ipc);
int ipu6_buttress_map_fw_image(struct ipu_bus_device *sys,
			       const struct firmware *fw,
			       struct sg_table *sgt);
void ipu6_buttress_unmap_fw_image(struct ipu_bus_device *sys,
				  struct sg_table *sgt);
int ipu6_buttress_power(struct device *dev, struct ipu_buttress_ctrl *ctrl,
			bool on);
bool ipu6_buttress_get_secure_mode(struct ipu_device *isp);
int ipu6_buttress_authenticate(struct ipu_device *isp);
int ipu6_buttress_reset_authentication(struct ipu_device *isp);
bool ipu6_buttress_auth_done(struct ipu_device *isp);
int ipu6_buttress_start_tsc_sync(struct ipu_device *isp);
void ipu6_buttress_tsc_read(struct ipu_device *isp, u64 *val);
u64 ipu6_buttress_tsc_ticks_to_ns(u64 ticks, const struct ipu_device *isp);

irqreturn_t ipu6_buttress_isr(int irq, void *isp_ptr);
irqreturn_t ipu6_buttress_isr_threaded(int irq, void *isp_ptr);
int ipu6_buttress_init(struct ipu_device *isp);
void ipu6_buttress_exit(struct ipu_device *isp);
void ipu6_buttress_csi_port_config(struct ipu_device *isp,
				   u32 legacy, u32 combo);
void ipu6_buttress_restore(struct ipu_device *isp);
#endif /* IPU6_BUTTRESS_H */
