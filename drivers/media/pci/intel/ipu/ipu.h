/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef INTEL_IPU_H
#define INTEL_IPU_H

#include <linux/auxiliary_bus.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

struct firmware;
struct pci_dev;

struct ipu_buttress_ctrl {
	u32 freq_ctl, pwr_sts_shift, pwr_sts_mask, pwr_sts_on, pwr_sts_off;
	u32 ratio;

	/* IPU6 */
	unsigned int qos_floor;
	bool started;

	/* IPU7 */
	u32 subsys_id;
	u32 ratio_shift;
	u32 cdyn;
	u32 cdyn_shift;
	u32 ovrd_clk;
	u32 own_clk_ack;
};

struct ipu_buttress_ipc {
	struct completion send_complete;
	struct completion recv_complete;
	u32 nack;
	u32 nack_mask;
	u32 recv_data;
	u32 csr_out;
	u32 csr_in;
	u32 db0_in;
	u32 db0_out;
	u32 data0_out;
	u32 data0_in;
};

struct ipu_buttress {
	struct mutex power_mutex, auth_mutex, cons_mutex, ipc_mutex;
	struct ipu_buttress_ipc cse;
	struct list_head constraints;
	u32 wdt_cached_value;
	bool force_suspend;
	u32 ref_clk;
};

struct ipu_ipc_buttress_bulk_msg {
	u32 cmd;
	u32 expected_resp;
	bool require_resp;
	u8 cmd_size;
};

struct ipu_device {
	struct pci_dev *pdev;
	struct list_head devices;
	struct ipu_bus_device *isys;
	struct ipu_bus_device *psys;
	struct ipu_buttress buttress;

	const struct firmware *cpd_fw;
	const char *cpd_fw_name;
	u32 cpd_metadata_cmpnt_size;

	void __iomem *base;
	bool need_ipc_reset;
	bool secure_mode;
	u8 hw_ver;
	bool bus_ready_to_probe;
};

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
	void *pdata;

	struct ipu_mmu *mmu;
	struct ipu_device *isp;
	struct ipu_buttress_ctrl *ctrl;
	u64 dma_mask;
	struct sg_table fw_sgt;

	/* IPU6 */
	const struct firmware *fw;
	u64 *pkg_dir;
	dma_addr_t pkg_dir_dma_addr;
	unsigned int pkg_dir_size;

	/* IPU7 */
	u32 fw_entry;
	struct ipu7_syscom_context *syscom;
	struct ia_gofo_boot_config *boot_config;
	dma_addr_t boot_config_dma_addr;
	u32 boot_config_size;
	enum ipu7_subsys subsys;
};

struct ipu_auxdrv_data {
	irqreturn_t (*isr)(struct ipu_bus_device *adev);
	irqreturn_t (*isr_threaded)(struct ipu_bus_device *adev);
	int (*buttress_power)(struct device *dev, struct ipu_buttress_ctrl *ctrl, bool on);
	bool wake_isr_thread;
};

#define to_ipu_bus_device(_dev) \
	container_of(to_auxiliary_dev(_dev), struct ipu_bus_device, auxdev)
#define auxdev_to_adev(_auxdev) \
	container_of(_auxdev, struct ipu_bus_device, auxdev)

struct ipu_bus_device *
ipu_bus_initialize_device(struct pci_dev *pdev, struct device *parent,
			   void *pdata, struct ipu_buttress_ctrl *ctrl,
			   char *name);
int ipu_bus_add_device(struct ipu_bus_device *adev);
void ipu_bus_del_devices(struct pci_dev *pdev);

#endif
