#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define MAX_IRQ_NUMBER 32

/*
 * here we assume all plic hwirq and tic hwirq should
 * be contiguous.
 * topc_intc hwirq is index of bitmap (both software and
 * hardware), and starts from 0.
 * so we use tic hwirq as index to get plic hwirq and its
 * irq data.
 * when used as a msi parent, tic hwirq is written to Top
 * reg for triggering irq by a PCIe device.
 *
 * now we pre-requested plic interrupt, but may try request
 * plic interrupt when needed, like gicp_irq_domain_alloc.
 * see drivers/irqchip/irq-mvebu-gicp.c
 */
struct top_intc_data {
	struct platform_device *pdev;
	int irq_num;
	struct irq_domain *domain;
	struct irq_chip *chip;
	int reg_bitwidth;

	DECLARE_BITMAP(irq_bitmap, MAX_IRQ_NUMBER);
	spinlock_t lock; // FIXME: mutex?

	void __iomem *reg_sta; /* status reg, see TRM, 10.1.31, GP_INTR_REGISTER_0 */
	void __iomem *reg_set; /* set reg, see TRM, 10.1.32, GP_INTR0_SET */
	void __iomem *reg_clr; /* clear reg, see TRM, 10.1.33, GP_INTR0_CLR */

	phys_addr_t reg_set_phys; /* MSI physical address */

	irq_hw_number_t plic_hwirqs[MAX_IRQ_NUMBER];
	int plic_irqs[MAX_IRQ_NUMBER];
	struct irq_data *plic_irq_datas[MAX_IRQ_NUMBER];
	int tic_to_plic[MAX_IRQ_NUMBER]; /* mapping from tic hwirq to plic hwirq */
};

static int top_intc_domain_alloc(struct irq_domain *domain,
				unsigned int virq, unsigned int nr_irqs,
				void *args)
{
	unsigned long flags;
	irq_hw_number_t hwirq;
	int i, ret = -1;
	struct top_intc_data *data = domain->host_data;

	spin_lock_irqsave(&data->lock, flags);
	ret = bitmap_find_free_region(data->irq_bitmap, data->irq_num,
				      order_base_2(nr_irqs));
	spin_unlock_irqrestore(&data->lock, flags);

	if (ret < 0) {
		pr_err("%s failed to alloc irq %d, total %d\n", __func__, virq, nr_irqs);
		return -ENOSPC;
	}

	hwirq = ret;
	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    data->chip,
				    data, handle_edge_irq,
				    NULL, NULL);
		data->tic_to_plic[hwirq + i] = data->plic_hwirqs[hwirq + i];
	}

	pr_info("----> %s hwirq %ld, irq %d, plic irq %d, total %d\n", __func__,
		hwirq, virq, data->plic_irqs[hwirq], nr_irqs);
	//dump_stack();
	return 0;
}

static void top_intc_domain_free(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct top_intc_data *data = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	pr_debug("%s hwirq %ld, irq %d, total %d\n", __func__, d->hwirq, virq, nr_irqs);

	spin_lock_irqsave(&data->lock, flags);
	bitmap_release_region(data->irq_bitmap, d->hwirq,
				order_base_2(nr_irqs));
	spin_unlock_irqrestore(&data->lock, flags);
}

static const struct irq_domain_ops top_intc_domain_ops = {
	.alloc	= top_intc_domain_alloc,
	.free	= top_intc_domain_free,
};

static void top_intc_ack_irq(struct irq_data *d)
{
	struct top_intc_data *data  = irq_data_get_irq_chip_data(d);
	int reg_off, bit_off;
	struct irq_data *plic_irq_data = data->plic_irq_datas[d->hwirq];

	// FIXME: hwirq 是一个 [0, 32), reg_bitwidth 是 32，那么 reg_off 永远是 0？
	// A: 可以优化
	reg_off = d->hwirq / data->reg_bitwidth;
	// 所以 bit_off 就等于 hwirq
	bit_off = d->hwirq - data->reg_bitwidth * reg_off;
	// reg_clr 本质上就是一个 32 位的寄存器
	writel(1 << bit_off, (unsigned int *)data->reg_clr + reg_off);

	pr_debug("%s %ld, parent %s/%ld\n", __func__, d->hwirq,
		plic_irq_data->domain->name, plic_irq_data->hwirq);
	if (plic_irq_data->chip->irq_ack)
		plic_irq_data->chip->irq_ack(plic_irq_data);
}

static void top_intc_mask_irq(struct irq_data *d)
{
	struct top_intc_data *data  = irq_data_get_irq_chip_data(d);
	struct irq_data *plic_irq_data = data->plic_irq_datas[d->hwirq];

	pr_debug("%s %ld, parent %s/%ld\n", __func__, d->hwirq,
		plic_irq_data->domain->name, plic_irq_data->hwirq);
	if (plic_irq_data->chip->irq_mask)
		plic_irq_data->chip->irq_mask(plic_irq_data);
}

static void top_intc_unmask_irq(struct irq_data *d)
{
	struct top_intc_data *data  = irq_data_get_irq_chip_data(d);
	struct irq_data *plic_irq_data = data->plic_irq_datas[d->hwirq];

	pr_debug("%s %ld, parent %s/%ld\n", __func__, d->hwirq,
		plic_irq_data->domain->name, plic_irq_data->hwirq);
	if (plic_irq_data->chip->irq_unmask)
		plic_irq_data->chip->irq_unmask(plic_irq_data);
}

static void top_intc_setup_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct top_intc_data *data  = irq_data_get_irq_chip_data(d);

	msg->address_lo = lower_32_bits(data->reg_set_phys);
	msg->address_hi = upper_32_bits(data->reg_set_phys);
	msg->data = 1 << d->hwirq;

	pr_debug("%s msi#%d: address_hi %#x, address_lo %#x, data %#x\n", __func__,
		(int)d->hwirq, msg->address_hi, msg->address_lo, msg->data);
}

static int top_intc_set_affinity(struct irq_data *d,
				 const struct cpumask *mask, bool force)
{
	struct top_intc_data *data  = irq_data_get_irq_chip_data(d);
	struct irq_data *plic_irq_data = data->plic_irq_datas[d->hwirq];

	irq_data_update_effective_affinity(d, mask);
	if (plic_irq_data->chip->irq_set_affinity)
		return plic_irq_data->chip->irq_set_affinity(plic_irq_data, mask, force);
	else
		return -EINVAL;
}

static int top_intc_set_type(struct irq_data *d, u32 type)
{
	/*
	 * dummy function, so __irq_set_trigger can continue to set
	 * correct trigger type.
	 */
	return 0;
}

static struct irq_chip top_intc_irq_chip = {
	.name = "PCIE MSI",
	.irq_ack = top_intc_ack_irq,
	.irq_mask = top_intc_mask_irq,
	.irq_unmask = top_intc_unmask_irq,
	.irq_compose_msi_msg = top_intc_setup_msi_msg,
	.irq_set_affinity = top_intc_set_affinity,
	.irq_set_type = top_intc_set_type,
};

static void top_intc_irq_handler(struct irq_desc *plic_desc)
{
	struct irq_chip *plic_chip = irq_desc_get_chip(plic_desc);
	struct top_intc_data *data = irq_desc_get_handler_data(plic_desc);
	irq_hw_number_t plic_hwirq = irq_desc_get_irq_data(plic_desc)->hwirq;
	irq_hw_number_t top_intc_hwirq;
	int top_intc_irq, i, ret;

	chained_irq_enter(plic_chip, plic_desc);

	for (i = 0; i < data->irq_num; i++) {
		if (data->tic_to_plic[i] == plic_hwirq)
			break;
	}
	if (i < data->irq_num) {
		top_intc_hwirq = i;
		top_intc_irq = irq_find_mapping(data->domain, top_intc_hwirq);
		pr_debug("%s plic hwirq %ld, tic hwirq %ld, tic irq %d\n", __func__,
				plic_hwirq, top_intc_hwirq, top_intc_irq);
		if (top_intc_irq)
			ret = generic_handle_irq(top_intc_irq);
		pr_debug("%s handled tic irq %d, %d\n", __func__, top_intc_irq, ret);
	} else {
		pr_debug("%s not found tic hwirq for plic hwirq %ld\n", __func__, plic_hwirq);
		// workaround, ack unexpected(unregistered) interrupt
		writel(1 << (plic_hwirq - data->plic_hwirqs[0]), data->reg_clr);
	}

	chained_irq_exit(plic_chip, plic_desc);
}

static int top_intc_probe(struct platform_device *pdev)
{
	struct fwnode_handle *fwnode = of_node_to_fwnode(pdev->dev.of_node);
	struct top_intc_data *data;
	struct resource *res;
	int i;

	data = devm_kzalloc(&pdev->dev, sizeof(struct top_intc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->pdev = pdev;
	spin_lock_init(&data->lock);

	if (device_property_read_u32(&pdev->dev, "reg-bitwidth", &data->reg_bitwidth))
		data->reg_bitwidth = 32;

	data->reg_sta = devm_platform_ioremap_resource_byname(pdev, "sta");
	if (IS_ERR(data->reg_sta)) {
		dev_err(&pdev->dev, "Failed to map status register\n");
		return PTR_ERR(data->reg_sta);
	}
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "set");
	data->reg_set = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->reg_set)) {
		dev_err(&pdev->dev, "Failed map set register\n");
		return PTR_ERR(data->reg_set);
	}
	data->reg_set_phys = res->start;
	data->reg_clr = devm_platform_ioremap_resource_byname(pdev, "clr");
	if (IS_ERR(data->reg_clr)) {
		dev_err(&pdev->dev, "Failed to map clear register\n");
		return PTR_ERR(data->reg_clr);
	}

	for (i = 0; i < ARRAY_SIZE(data->plic_hwirqs); i++) {
		char msi_name[8];
		int irq;

		snprintf(msi_name, ARRAY_SIZE(msi_name), "msi%d", i);
		irq = platform_get_irq_byname_optional(pdev, msi_name);
		if (irq == -ENXIO)
			break;
		if (irq < 0)
			return dev_err_probe(&pdev->dev, irq,
					     "Failed to parse MSI IRQ '%s'\n",
					     msi_name);

		data->plic_irqs[i] = irq;
		data->plic_irq_datas[i] = irq_get_irq_data(irq);
		data->plic_hwirqs[i] = data->plic_irq_datas[i]->hwirq;
		dev_dbg(&pdev->dev, "%s[%d]: plic hwirq %ld, plic irq %d\n", msi_name, i,
			data->plic_hwirqs[i], data->plic_irqs[i]);
	}
	if (!i) {
		dev_err(&pdev->dev, "No MSI IRQ provided!\n");
		return -ENXIO;
	}
	data->irq_num = i;

	/* create MSI domain */
	data->domain = irq_domain_create_linear(fwnode, data->irq_num,
						&top_intc_domain_ops, data);
	if (!data->domain) {
		dev_err(&pdev->dev, "Failed to create IRQ doamin\n");
		return -ENODEV;
	}
	irq_domain_update_bus_token(data->domain, DOMAIN_BUS_NEXUS);

	data->chip = &top_intc_irq_chip;

	for (i = 0; i < data->irq_num; i++)
		irq_set_chained_handler_and_data(data->plic_irqs[i],
						 top_intc_irq_handler, data);

	platform_set_drvdata(pdev, data);

	return 0;
}

static const struct of_device_id top_intc_of_match[] = {
	{ .compatible = "sophgo,top-intc" },
	{}
};

static struct platform_driver top_intc_driver = {
	.driver = {
		.name = "sophgo,top-intc",
		.of_match_table = of_match_ptr(top_intc_of_match),
	},
	.probe = top_intc_probe,
};
builtin_platform_driver(top_intc_driver);
