// SPDX-License-Identifier: GPL-2.0
/*
 * pcie-sg2042 - PCIe controller driver for Sophgo SG2042 SoC
 *
 * Copyright (C) 2024 Sophgo Technology Inc.
 * Copyright (C) 2024 Chen Wang <unicorn_wang@outlook.com>
 */

#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "pcie-cadence.h"

#define MAX_MSI_IRQS		8
#define MAX_MSI_IRQS_PER_CTRL	1
#define MAX_MSI_CTRLS		(MAX_MSI_IRQS / MAX_MSI_IRQS_PER_CTRL)
#define MSI_DEF_NUM_VECTORS	MAX_MSI_IRQS
#define BYTE_NUM_PER_MSI_VEC	4

// mango sideband signals
#define IRS_REG0804	0x0804
#define IRS_REG080C	0x080C
#define IRS_REG0810	0x0810
#define IRS_REG085C	0x085C
#define IRS_REG0860	0x0860
#define IRS_REG0864	0x0864
#define IRS_REG0868	0x0868
#define IRS_REG086C	0x086C

#define IRS_REG0804_CLR_LINK0_MSI_IN_BIT	2
#define IRS_REG0804_CLR_LINK1_MSI_IN_BIT	3
#define IRS_REG0810_ST_LINK0_MSI_IN_BIT		2
#define IRS_REG0810_ST_LINK1_MSI_IN_BIT		3

#define CDNS_PLAT_CPU_TO_BUS_ADDR	0xCFFFFFFFFF

struct sg2042_pcie {
	struct cdns_pcie *cdns_pcie;

	struct regmap *syscon;

	u32 pcie_id; // FIXME dts 设置了，但是代码中用不到
	u32 link_id;
	u32 top_intc_used;

	struct irq_domain *msi_domain;
	int msi_irq;
	dma_addr_t msi_data;
	void *msi_page;
	u32 num_applied_vecs; /* number of applied vectors, used to speed up in ISR */
	raw_spinlock_t lock;
	DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_IRQS);
};

/*
 * We use the usual two domain structure, the top one being a generic PCI/MSI
 * domain, the bottom one being SG2042-specific and handling the actual HW
 * interrupt allocation.
 * At the same time, bottom chip uses a chained handler to handle the controller's
 * MSI IRQ edge triggered.
 */
static void sg2042_pcie_msi_top_irq_ack(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void sg2042_pcie_msi_top_irq_mask(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void sg2042_pcie_msi_top_irq_unmask(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip sg2042_pcie_msi_top_chip = {
	.name = "SG2042 PCIe MSI",
	.irq_ack = sg2042_pcie_msi_top_irq_ack,
	.irq_mask = sg2042_pcie_msi_top_irq_mask,
	.irq_unmask = sg2042_pcie_msi_top_irq_unmask,
};

static struct msi_domain_info sg2042_pcie_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.chip	= &sg2042_pcie_msi_top_chip,
};

static struct irq_domain *sg2042_pcie_get_parent_irq_domain(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *parent;
	struct irq_domain *domain;

	if (!of_find_property(np, "interrupt-parent", NULL)) {
		dev_err(dev, "Can't find interrupt-parent!\n");
		return NULL;
	}

	parent = of_irq_find_parent(np);
	if (!parent) {
		dev_err(dev, "Can't find parent node!\n");
		return ERR_PTR(-ENXIO);
	}

	domain = irq_find_host(parent);
	of_node_put(parent);
	if (!domain) {
		dev_err(dev, "Can't find domain of interrupt-parent!\n");
		return ERR_PTR(-ENXIO);
	}

	return domain;
}


static int sg2042_pcie_setup_top_intc(struct sg2042_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct fwnode_handle *fwnode = of_node_to_fwnode(dev->of_node);
	struct irq_domain *parent_domain = sg2042_pcie_get_parent_irq_domain(dev);

	pcie->msi_domain = pci_msi_create_irq_domain(fwnode,
						     &sg2042_pcie_msi_domain_info,
						     parent_domain);

	if (!pcie->msi_domain) {
		dev_err(dev, "create msi irq domain failed\n");
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t sg2042_pcie_handle_msi_irq(struct sg2042_pcie *pcie)
{
	u32 i, pos;
	unsigned long val;
	u32 status, num_vectors;
	irqreturn_t ret = IRQ_NONE;

	num_vectors = pcie->num_applied_vecs;
	for (i = 0; i <= num_vectors; i++) {
		status = readl((void *)(pcie->msi_page + i * BYTE_NUM_PER_MSI_VEC));
		if (!status)
			continue;

		ret = IRQ_HANDLED;
		val = status;
		pos = 0;
		while ((pos = find_next_bit(&val, MAX_MSI_IRQS_PER_CTRL,
					    pos)) != MAX_MSI_IRQS_PER_CTRL) {
			generic_handle_domain_irq(pcie->msi_domain->parent,
						  (i * MAX_MSI_IRQS_PER_CTRL) +
						  pos);
			pos++;
		}
		writel(0, ((void *)(pcie->msi_page) + i * BYTE_NUM_PER_MSI_VEC));
	}
	return ret;
}

static void sg2042_pcie_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct sg2042_pcie *pcie;
	u32 status = 0;
	u32 st_msi_in_bit = 0;
	u32 clr_msi_in_bit = 0;

	chained_irq_enter(chip, desc);

	pcie = irq_desc_get_handler_data(desc);
	if (pcie->link_id == 1) {
		st_msi_in_bit = IRS_REG0810_ST_LINK1_MSI_IN_BIT;
		clr_msi_in_bit = IRS_REG0804_CLR_LINK1_MSI_IN_BIT;
	} else {
		st_msi_in_bit = IRS_REG0810_ST_LINK0_MSI_IN_BIT;
		clr_msi_in_bit = IRS_REG0804_CLR_LINK0_MSI_IN_BIT;
	}

	regmap_read(pcie->syscon, IRS_REG0810, &status);
	if ((status >> st_msi_in_bit) & 0x1) {
		WARN_ON(!IS_ENABLED(CONFIG_PCI_MSI));

		regmap_read(pcie->syscon, IRS_REG0804, &status);
		status |= ((u32)0x1 << clr_msi_in_bit);
		regmap_write(pcie->syscon, IRS_REG0804, status);

		/* need write 0 to reset, hardware will not reset automaticly */
		status &= ~((u32)0x1 << clr_msi_in_bit);
		regmap_write(pcie->syscon, IRS_REG0804, status);

		sg2042_pcie_handle_msi_irq(pcie);
	}

	chained_irq_exit(chip, desc);
}

static int sg2042_pcie_msi_irq_set_affinity(struct irq_data *d,
					    const struct cpumask *mask,
					    bool force)
{
	return -EINVAL;
}

static void sg2042_pcie_msi_irq_compose_msi_msg(struct irq_data *d,
						struct msi_msg *msg)
{
	struct sg2042_pcie *pcie = irq_data_get_irq_chip_data(d);
	struct device *dev = pcie->cdns_pcie->dev;
	u64 msi_target;

	msi_target = (u64)pcie->msi_data;

	msg->address_lo = lower_32_bits(msi_target) + BYTE_NUM_PER_MSI_VEC * d->hwirq;
	msg->address_hi = upper_32_bits(msi_target);
	msg->data = 1;

	pcie->num_applied_vecs = d->hwirq;

	dev_err(dev, "msi#%d address_hi %#x address_lo %#x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo);
}

/* Just a dummy function to make irq_chip_xxx_parent happy in top chip callbacks */
static void sg2042_pcie_msi_irq_dummy(struct irq_data *d)
{
}

static struct irq_chip sg2042_pcie_msi_bottom_chip = {
	.name = "SG2042 MSI",
	.irq_ack = sg2042_pcie_msi_irq_dummy,
	.irq_compose_msi_msg = sg2042_pcie_msi_irq_compose_msi_msg,
	.irq_set_affinity = sg2042_pcie_msi_irq_set_affinity,
	.irq_mask = sg2042_pcie_msi_irq_dummy,
	.irq_unmask = sg2042_pcie_msi_irq_dummy,
};

static int sg2042_pcie_irq_domain_alloc(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs,
					void *args)
{
	struct sg2042_pcie *pcie = domain->host_data;
	unsigned long flags;
	u32 i;
	int bit;

	raw_spin_lock_irqsave(&pcie->lock, flags);

	bit = bitmap_find_free_region(pcie->msi_irq_in_use, MSI_DEF_NUM_VECTORS,
				      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pcie->lock, flags);

	if (bit < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, bit + i,
				    &sg2042_pcie_msi_bottom_chip,
				    pcie, handle_edge_irq,
				    NULL, NULL);

	return 0;
}

static void sg2042_pcie_irq_domain_free(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct sg2042_pcie *pcie = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pcie->lock, flags);

	bitmap_release_region(pcie->msi_irq_in_use, d->hwirq,
			      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pcie->lock, flags);
}

static const struct irq_domain_ops sg2042_pcie_msi_domain_ops = {
	.alloc	= sg2042_pcie_irq_domain_alloc,
	.free	= sg2042_pcie_irq_domain_free,
};

static int sg2042_pcie_allocate_domains(struct sg2042_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct fwnode_handle *fwnode = of_node_to_fwnode(dev->of_node);
	struct irq_domain *parent;

	parent = irq_domain_create_linear(fwnode, MSI_DEF_NUM_VECTORS,
					  &sg2042_pcie_msi_domain_ops, pcie);
	if (!parent) {
		dev_err(dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(parent, DOMAIN_BUS_NEXUS);

	pcie->msi_domain = pci_msi_create_irq_domain(fwnode,
						     &sg2042_pcie_msi_domain_info,
						     parent);
	if (!pcie->msi_domain) {
		dev_err(dev, "Failed to create MSI domain\n");
		irq_domain_remove(parent);
		return -ENOMEM;
	}

	return 0;
}

static void sg2042_pcie_free_msi(struct sg2042_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;

	if (pcie->msi_irq) {
		irq_set_chained_handler(pcie->msi_irq, NULL);
		irq_set_handler_data(pcie->msi_irq, NULL);
	}

	irq_domain_remove(pcie->msi_domain);
	irq_domain_remove(pcie->msi_domain->parent);

	if (pcie->msi_page)
		dma_free_coherent(dev, 1024, pcie->msi_page, pcie->msi_data);

}

static int sg2042_pcie_msi_init(struct sg2042_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	u64 msi_target = 0;
	u32 value = 0;

	// support 512 msi vectors
	pcie->msi_page = dma_alloc_coherent(dev, 2048, &pcie->msi_data,
					  (GFP_KERNEL|GFP_DMA32|__GFP_ZERO));
	if (pcie->msi_page == NULL)
		return -1;

	dev_info(dev, "msi_data is 0x%llx\n", pcie->msi_data);
	msi_target = (u64)pcie->msi_data;

	if (pcie->link_id == 1) {
		/* Program the msi_data */
		regmap_write(pcie->syscon, IRS_REG0868,
				 lower_32_bits(msi_target));
		regmap_write(pcie->syscon, IRS_REG086C,
				 upper_32_bits(msi_target));

		regmap_read(pcie->syscon, IRS_REG080C, &value);
		value = (value & 0xffff0000) | MAX_MSI_IRQS;
		regmap_write(pcie->syscon, IRS_REG080C, value);
	} else {
		/* Program the msi_data */
		regmap_write(pcie->syscon, IRS_REG0860,
				 lower_32_bits(msi_target));
		regmap_write(pcie->syscon, IRS_REG0864,
				 upper_32_bits(msi_target));

		regmap_read(pcie->syscon, IRS_REG085C, &value);
		value = (value & 0x0000ffff) | (MAX_MSI_IRQS << 16);
		regmap_write(pcie->syscon, IRS_REG085C, value);
	}

	return 0;
}

static int sg2042_pcie_setup_msi(struct sg2042_pcie *pcie, struct platform_device *pdev)
{
	struct device *dev = pcie->cdns_pcie->dev;
	int ret = 0;

	ret = sg2042_pcie_msi_init(pcie);
	if (ret) {
		dev_err(dev, "Failed to initialize msi!\n");
		return ret;
	}

	ret = platform_get_irq_byname(pdev, "msi");
	if (ret <= 0) {
		dev_err(dev, "failed to get MSI irq\n");
		return ret;
	}
	pcie->msi_irq = ret;

	// 初始化一把 lock，这把锁会用于 bitmap_find_free_region/bitmap_release_region
	// FIMXE? why 需要这把锁？
	raw_spin_lock_init(&pcie->lock);

	ret = sg2042_pcie_allocate_domains(pcie);
	if (ret)
		return ret;

	if (pcie->msi_irq)
		irq_set_chained_handler_and_data(pcie->msi_irq,
						 sg2042_pcie_chained_msi_isr, pcie);

	return 0;
}

// FIMXE: what's this?
static u64 sg2042_pcie_cpu_addr_fixup(struct cdns_pcie *pcie, u64 cpu_addr)
{
	return cpu_addr & CDNS_PLAT_CPU_TO_BUS_ADDR;
}

static const struct cdns_pcie_ops sg2042_pcie_ops = {
	.cpu_addr_fixup = sg2042_pcie_cpu_addr_fixup,
};

/*
 * SG2042 only support 4-byte aligned access, so for the rootbus (i.e. to read
 * the PCIe controller itself, read32 is required. For non-rootbus (i.e. to read
 * the PCIe peripheral registers, supports 1/2/4 byte aligned access, so
 * directly use read should be fine.
 * The same is true for write.
 */
static int sg2042_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 *value)
{
	if (pci_is_root_bus(bus))
		return pci_generic_config_read32(bus, devfn, where, size,
						 value);

	return pci_generic_config_read(bus, devfn, where, size, value);
}

static int sg2042_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 value)
{
	if (pci_is_root_bus(bus))
		return pci_generic_config_write32(bus, devfn, where, size,
						  value);

	return pci_generic_config_write(bus, devfn, where, size, value);
}

static struct pci_ops sg2042_pcie_host_ops = {
	.map_bus	= cdns_pci_map_bus,
	.read		= sg2042_pcie_config_read,
	.write		= sg2042_pcie_config_write,
};

static const struct of_device_id sg2042_pcie_of_match[] = {
	{ .compatible = "sophgo,cdns-pcie-host" },
	{},
};

static int sg2042_pcie_host_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct pci_host_bridge *bridge;
	struct device_node *np_syscon;
	struct cdns_pcie *cdns_pcie;
	struct sg2042_pcie *pcie;
	struct cdns_pcie_rc *rc;
	struct regmap *syscon;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	if (!IS_ENABLED(CONFIG_PCIE_CADENCE_HOST))
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
	if (!bridge)
		return -ENOMEM;
	bridge->ops = &sg2042_pcie_host_ops;
	rc = pci_host_bridge_priv(bridge);

	cdns_pcie = &rc->pcie;
	cdns_pcie->dev = dev;
	cdns_pcie->ops = &sg2042_pcie_ops;
	pcie->cdns_pcie = cdns_pcie;

	np_syscon = of_parse_phandle(np, "pcie-syscon", 0);
	if (!np_syscon) {
		dev_err(dev, "Failed to get pcie-syscon node\n");
		return -ENOMEM;
	}
	syscon = syscon_node_to_regmap(np_syscon);
	if (IS_ERR(syscon)) {
		dev_err(dev, "Failed to get regmap for pcie-syscon\n");
		return -ENOMEM;
	}
	pcie->syscon = syscon;

	of_property_read_u32(np, "pcie-id", &pcie->pcie_id);
	of_property_read_u32(np, "link-id", &pcie->link_id);
	of_property_read_u32(np, "top-intc-used", &pcie->top_intc_used);

	platform_set_drvdata(pdev, pcie);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	if (pcie->top_intc_used == 1) {
		ret = sg2042_pcie_setup_top_intc(pcie);
		if (ret < 0)
			goto err_pcie_setup;
	} else {
		ret = sg2042_pcie_setup_msi(pcie, pdev);
		if (ret < 0)
			goto err_sg2042_pcie_setup_msi;
	}

	ret = cdns_pcie_init_phy(dev, cdns_pcie);
	if (ret) {
		dev_err(dev, "Failed to init phy\n");
		goto err_get_sync;
	}
	
	ret = cdns_pcie_host_setup(rc);
	if (ret < 0) {
		goto err_pcie_setup;
	}

	return 0;

 err_sg2042_pcie_setup_msi:
	sg2042_pcie_free_msi(pcie);

 err_pcie_setup:
	cdns_pcie_disable_phy(cdns_pcie);

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

// FIXME：参考 dw 的做法，在 remove/shutdown 时还有很多需要清理
static void cdns_pcie_shutdown(struct platform_device *pdev)
{
	struct sg2042_pcie *pcie = platform_get_drvdata(pdev);
	struct cdns_pcie *cdns_pcie = pcie->cdns_pcie;
	struct device *dev = &pdev->dev;

	cdns_pcie_disable_phy(cdns_pcie);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
}

static struct platform_driver cdns_pcie_host_driver = {
	.driver = {
		.name = "cdns-pcie-host",
		.of_match_table = sg2042_pcie_of_match,
		.pm = &cdns_pcie_pm_ops,
	},
	.probe = sg2042_pcie_host_probe,
	.shutdown = cdns_pcie_shutdown,
};
builtin_platform_driver(cdns_pcie_host_driver);
