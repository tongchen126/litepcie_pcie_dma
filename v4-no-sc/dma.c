// SPDX-License-Identifier: BSD-2-Clause

/*
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/log2.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include "litepcie.h"
#include "csr.h"
#include "config.h"
#include "flags.h"
#include "mem.h"

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32
#define LITEPCIE_DMA_BUF_ORDER (4)
#define LITEPCIE_DMA_BUF_SIZE (1 << (LITEPCIE_DMA_BUF_ORDER + PAGE_SHIFT))

struct litepcie_device {
	struct pci_dev *dev;
	struct platform_device *uart;
	resource_size_t bar0_size;
	phys_addr_t bar0_phys_addr;
	uint8_t *bar0_addr; /* virtual address of BAR0 */
	spinlock_t lock;
	int irqs;
	dma_addr_t dma_addr;
	void* host_dma_addr;
	struct work_struct irq0_work;
	struct work_struct irq1_work;
};


static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
	uint32_t val;

	val = readl(s->bar0_addr + addr - CSR_BASE);
#ifdef DEBUG_CSR
	dev_dbg(&s->dev->dev, "csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
	return val;
}

static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
	dev_dbg(&s->dev->dev, "csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
	return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v |= (1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v &= ~(1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}
static void irq0_work_func(struct work_struct *work)
{
	
	struct litepcie_device *s  = container_of(work,struct litepcie_device,irq0_work);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_SC_DEPTH_ADDR,2);


	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_HOST_ADDR_ADDR,s->dma_addr);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_LENGTH_ADDR,LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_BUS_ADDR_ADDR,MAIN_RAM_BASE);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_RD_ENABLE_ADDR,1);
	

	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_HOST_ADDR_ADDR,s->dma_addr);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_LENGTH_ADDR,LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_BUS_ADDR_ADDR,MAIN_RAM_BASE+LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_PCIE2WB_DMA_RD_ENABLE_ADDR,1);
	
	pci_info(s->dev,"irq0_work_func\n");
	
}
static void irq1_work_func(struct work_struct *work)
{
	struct litepcie_device *s  = container_of(work,struct litepcie_device,irq1_work);

	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_HOST_ADDR_ADDR,s->dma_addr);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_BUS_ADDR_ADDR,MAIN_RAM_BASE);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_WR_ENABLE_ADDR,1);
	

//	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,4*1024);
//	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,4*1024);
//	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,4*1024);
//	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,4*1024);
	
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_HOST_ADDR_ADDR,s->dma_addr);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_LENGTH_ADDR,LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_BUS_ADDR_ADDR,MAIN_RAM_BASE+LITEPCIE_DMA_BUF_SIZE);
	litepcie_writel(s,CSR_PCIE_HOST_WB2PCIE_DMA_WR_ENABLE_ADDR,1);
	
	pci_info(s->dev,"irq1_work_func\n");
}
static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *s = (struct litepcie_device *) data;

	uint32_t irq_vector, irq_enable;
	int i;

	irq_vector = 0;
	for (i = 0; i < s->irqs; i++) {
		if (irq == pci_irq_vector(s->dev, i)) {
			irq_vector = (1 << i);
			break;
		}
	}
	irq_enable = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	pci_info(s->dev,"litepcie: interrupt irq_enable 0x%x, irq_vector 0x%x, irq %d, i %d, dma data 0x%x\n",irq_enable,irq_vector,irq,i, *(int *)s->host_dma_addr);
	//irq_vector &= irq_enable;
	if (i == 0 && !work_busy(&s->irq0_work))
		schedule_work(&s->irq0_work);
	if (i == 1 && !work_busy(&s->irq1_work))
		schedule_work(&s->irq1_work);
	
	return IRQ_HANDLED;
}



static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	int irqs = 0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct litepcie_device *litepcie_dev = NULL;
	struct page *pages;

	dev_info(&dev->dev, "\e[1m[Probing device]\e[0m\n");

	litepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct litepcie_device), GFP_KERNEL);
	if (!litepcie_dev) {
		ret = -ENOMEM;
		goto fail1;
	}
	INIT_WORK(&litepcie_dev->irq0_work,irq0_work_func);
	INIT_WORK(&litepcie_dev->irq1_work,irq1_work_func);
	pci_set_drvdata(dev, litepcie_dev);
	litepcie_dev->dev = dev;
	spin_lock_init(&litepcie_dev->lock);

	ret = pcim_enable_device(dev);
	if (ret != 0) {
		dev_err(&dev->dev, "Cannot enable device\n");
		goto fail1;
	}

	ret = -EIO;

	/* check device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 0) {
		dev_err(&dev->dev, "Unsupported device version %d\n", rev_id);
		goto fail1;
	}

	/* check bar0 config */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
		dev_err(&dev->dev, "Invalid BAR0 configuration\n");
		goto fail1;
	}

	if (pcim_iomap_regions(dev, BIT(0), LITEPCIE_NAME) < 0) {
		dev_err(&dev->dev, "Could not request regions\n");
		goto fail1;
	}

	litepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
	if (!litepcie_dev->bar0_addr) {
		dev_err(&dev->dev, "Could not map BAR0\n");
		goto fail1;
	}

	/* show identifier */
	for (i = 0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i*4);
	dev_info(&dev->dev, "Version %s\n", fpga_identifier);

	pci_set_master(dev);
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->dev, "Failed to set DMA mask\n");
		goto fail1;
	};

	irqs = pci_alloc_irq_vectors(dev, 1, 32, PCI_IRQ_MSI);
	if (irqs < 0) {
		dev_err(&dev->dev, "Failed to enable MSI\n");
		ret = irqs;
		goto fail1;
	}
	dev_info(&dev->dev, "%d MSI IRQs allocated.\n", irqs);

	litepcie_dev->irqs = 0;
	for (i = 0; i < irqs; i++) {
		int irq = pci_irq_vector(dev, i);

		ret = request_irq(irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, litepcie_dev);
		if (ret < 0) {
			dev_err(&dev->dev, " Failed to allocate IRQ %d\n", dev->irq);
			while (--i >= 0) {
				irq = pci_irq_vector(dev, i);
				free_irq(irq, dev);
			}
			goto fail2;
		}
		litepcie_dev->irqs += 1;
	}
	
	litepcie_dev->host_dma_addr = dmam_alloc_coherent(
				&dev->dev,
				LITEPCIE_DMA_BUF_SIZE,
				&litepcie_dev->dma_addr,
				GFP_KERNEL);
	
	/*
	pages = alloc_pages(GFP_KERNEL,LITEPCIE_DMA_BUF_ORDER);
	if (!pages){
		ret = -ENOMEM;
		goto fail2;
	}
	litepcie_dev->host_dma_addr = page_to_virt(pages);
	//litepcie_dev->dma_addr = dma_map_single(&dev->dev,litepcie_dev->host_dma_addr,PAGE_SIZE,DMA_BIDIRECTIONAL);
	litepcie_dev->dma_addr = dma_map_page(&dev->dev,pages,0,LITEPCIE_DMA_BUF_SIZE,DMA_BIDIRECTIONAL);
	*/

	//litepcie_dev->host_dma_addr = dma_alloc_attrs(&dev->dev,LITEPCIE_DMA_BUF_SIZE,&litepcie_dev->dma_addr,GFP_DMA,0);
	if (!litepcie_dev->host_dma_addr){
		ret = -ENOMEM;
		goto fail2;
	}
	memset(litepcie_dev->host_dma_addr,0x12345678,4);
	memset(litepcie_dev->host_dma_addr+4,0xabcdffee,4);
	memset(litepcie_dev->host_dma_addr+8,0x12345678,4);
	memset(litepcie_dev->host_dma_addr+12,0xabcdef09,4);
	memset(litepcie_dev->host_dma_addr+16,0xabcdff09,LITEPCIE_DMA_BUF_SIZE-16);
	pci_info(dev,"dma addr 0x%x, host dma addr 0x%x, PAGE_SHIFT:%d, BUF_SIZE: %d\n",litepcie_dev->dma_addr,litepcie_dev->host_dma_addr,PAGE_SHIFT,LITEPCIE_DMA_BUF_SIZE);
	

	return 0;

fail2:
	pci_free_irq_vectors(dev);
fail1:
	return ret;
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
	int i, irq;
	struct litepcie_device *litepcie_dev;

	litepcie_dev = pci_get_drvdata(dev);

	dev_info(&dev->dev, "\e[1m[Removing device]\e[0m\n");

	/* Disable all interrupts */
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

	/* Free all interrupts */
	for (i = 0; i < litepcie_dev->irqs; i++) {
		irq = pci_irq_vector(dev, i);
		free_irq(irq, litepcie_dev);
	}
	dma_free_coherent(&dev->dev,LITEPCIE_DMA_BUF_SIZE,litepcie_dev->host_dma_addr,litepcie_dev->dma_addr);
	//free_pages(litepcie_dev->host_dma_addr,LITEPCIE_DMA_BUF_ORDER);
	pci_free_irq_vectors(dev);
}

static const struct pci_device_id litepcie_pci_ids[] = {
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X8),  },

	{ 0, }
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
	.name = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe = litepcie_pci_probe,
	.remove = litepcie_pci_remove,
};


static int __init litepcie_module_init(void)
{
	int ret;
	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		pr_err(" Error while registering PCI driver\n");
	}

	return ret;
}

static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
}


module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
