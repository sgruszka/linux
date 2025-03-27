/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_BUS_H
#define IPU7_BUS_H

#include <linux/auxiliary_bus.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "abi/ipu7_fw_boot_abi.h"

#include "ipu7-syscom.h"

struct pci_dev;
struct ipu_buttress_ctrl;
struct ipu7_mmu;
struct ipu_device;

enum ipu7_subsys {
	IPU_IS = 0,
	IPU_PS = 1,
	IPU_SUBSYS_NUM = 2,
};

struct ipu_bus_device {
	struct auxiliary_device auxdev;
	const struct auxiliary_driver *auxdrv;
	const struct ipu_auxdrv_data *auxdrv_data;
	struct list_head list;
	enum ipu7_subsys subsys;
	void *pdata;
	struct ipu7_mmu *mmu;
	struct ipu_device *isp;
	const struct ipu_buttress_ctrl *ctrl;
	u64 dma_mask;
	struct sg_table fw_sgt;
	u32 fw_entry;
	struct ipu7_syscom_context *syscom;
	struct ia_gofo_boot_config *boot_config;
	dma_addr_t boot_config_dma_addr;
	u32 boot_config_size;
};

struct ipu_auxdrv_data {
	irqreturn_t (*isr)(struct ipu_bus_device *adev);
	irqreturn_t (*isr_threaded)(struct ipu_bus_device *adev);
	bool wake_isr_thread;
};

#define to_ipu_bus_device(_dev)					\
	container_of(to_auxiliary_dev(_dev), struct ipu_bus_device, auxdev)
#define auxdev_to_adev(_auxdev)					\
	container_of(_auxdev, struct ipu_bus_device, auxdev)
#define ipu7_bus_get_drvdata(adev) dev_get_drvdata(&(adev)->auxdev.dev)

struct ipu_bus_device *
ipu7_bus_initialize_device(struct pci_dev *pdev, struct device *parent,
			   void *pdata, const struct ipu_buttress_ctrl *ctrl,
			   char *name);
int ipu7_bus_add_device(struct ipu_bus_device *adev);
void ipu7_bus_del_devices(struct pci_dev *pdev);
#endif
