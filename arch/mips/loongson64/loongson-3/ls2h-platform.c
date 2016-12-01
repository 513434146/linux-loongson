// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *  Copyright (C) 2014-2017, Lemote, Inc.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/phy.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/delay.h>
#include <linux/stmmac.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/usb/ohci_pdriver.h>
#include <asm/io.h>

#include <irq.h>
#include <pci.h>
#include <boot_param.h>
#include <loongson-pch.h>

static u64 platform_dma_mask = DMA_BIT_MASK(32);

/*
 * UART
 */
static struct plat_serial8250_port uart8250_data[] = {
	[0] = {
		.mapbase = LS2H_UART0_REG_BASE,
		.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART0_REG_BASE),
		.irq = LS2H_PCH_UART0_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[1] = {
		.mapbase = LS2H_UART1_REG_BASE,
		.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART1_REG_BASE),
		.irq = LS2H_PCH_UART1_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[2] = {
		.mapbase = LS2H_UART2_REG_BASE,
		.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART2_REG_BASE),
		.irq = LS2H_PCH_UART2_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[3] = {
		.mapbase = LS2H_UART3_REG_BASE,
		.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART3_REG_BASE),
		.irq = LS2H_PCH_UART3_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype = UPIO_MEM,
		.regshift = 0,
	},
	{}
};

static struct platform_device uart8250_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM1,
	.dev = {
		.platform_data = uart8250_data,
	}
};

/*
 * NAND
 */
static struct mtd_partition nand_partitions[] = {
	[0] = {
		.name   = "kernel",
		.offset = 0,
		.size   = 0x01400000,
	},
	[1] = {
		.name   = "os",
		.offset = 0x01400000,
		.size   = 0x0,

	},
};

static struct platform_nand_chip nand_parts = {
        .partitions          = nand_partitions,
        .nr_partitions       = ARRAY_SIZE(nand_partitions),
};

static struct resource nand_resources[] = {
	[0] = {
		.start      = 0,
		.end        = 0,
		.flags      = IORESOURCE_DMA,
	},
	[1] = {
		.start      = LS2H_NAND_REG_BASE,
		.end        = LS2H_NAND_REG_BASE + 0x20,
		.flags      = IORESOURCE_MEM,
	},
	[2] = {
		.start      = LS2H_DMA_ORDER_REG_BASE,
		.end        = LS2H_DMA_ORDER_REG_BASE,
		.flags      = IORESOURCE_MEM,
	},
	[3] = {
		.start      = LS2H_PCH_DMA0_IRQ,
		.end        = LS2H_PCH_DMA0_IRQ,
		.flags      = IORESOURCE_IRQ,
	},
};

struct platform_device ls2h_nand_device = {
	.name       = "ls2h-nand",
	.id         = 0,
	.dev        = {
		.platform_data = &nand_parts,
	},
	.num_resources  = ARRAY_SIZE(nand_resources),
	.resource       = nand_resources,
};

/*
 * OHCI
 */
static struct resource ohci_resources[] = {
	[0] = {
		.start = LS2H_OHCI_REG_BASE,
		.end   = (LS2H_OHCI_REG_BASE + 0x1000 - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_OHCI_IRQ,
		.end   = LS2H_PCH_OHCI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct usb_ohci_pdata ohci_platform_data = {
	.num_ports = 6,
};

static struct platform_device ls2h_ohci_device = {
	.name           = "ohci-platform",
	.id             = 0,
	.dev = {
		.platform_data	= &ohci_platform_data,
	},
	.num_resources  = ARRAY_SIZE(ohci_resources),
	.resource       = ohci_resources,
};

/*
 * EHCI
 */
static struct resource ehci_resources[] = {
	[0] = {
		.start = LS2H_EHCI_REG_BASE,
		.end   = (LS2H_EHCI_REG_BASE + 0x100 - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_EHCI_IRQ,
		.end   = LS2H_PCH_EHCI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct usb_ehci_pdata ehci_platform_data = {
};

static struct platform_device ls2h_ehci_device = {
	.name	= "ehci-platform",
	.id	= 0,
	.dev	= {
		.platform_data	= &ehci_platform_data,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};

/*
 * PHY0
 */
static struct stmmac_mdio_bus_data phy0_plat_data = {
	.phy_mask	= 0,
};

/*
 * GMAC0
 */
static struct plat_stmmacenet_data gmac0_plat_dat = {
	.bus_id		= 0,
	.phy_addr	= 1,
	.has_gmac	= 1,
	.enh_desc	= 1,
	.mdio_bus_data = &phy0_plat_data,
};

static struct resource gmac0_resources[] = {
	[0] = {
		.start = LS2H_GMAC0_REG_BASE,
		.end   = (LS2H_GMAC0_REG_BASE + 0x1000 - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_GMAC0_IRQ,
		.end   = LS2H_PCH_GMAC0_IRQ,
		.name  = "macirq",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_gmac0_device = {
	.name	= "stmmaceth",
	.id	= 0,
	.dev	= {
		.platform_data	= &gmac0_plat_dat,
	},
	.num_resources  = ARRAY_SIZE(gmac0_resources),
	.resource       = gmac0_resources,
};

/*
 * PHY1
 */
static struct stmmac_mdio_bus_data phy1_plat_data = {
	.phy_mask	= 0,
};

/*
 * GMAC1
 */
static struct plat_stmmacenet_data gmac1_plat_dat = {
	.bus_id		= 1,
	.phy_addr	= 1,
	.has_gmac	= 1,
	.enh_desc	= 1,
	.mdio_bus_data = &phy1_plat_data,
};

static struct resource gmac1_resources[] = {
	[0] = {
		.start = LS2H_GMAC1_REG_BASE,
		.end   = (LS2H_GMAC1_REG_BASE + 0x1000 - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_GMAC1_IRQ,
		.end   = LS2H_PCH_GMAC1_IRQ,
		.name  = "macirq",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_gmac1_device = {
	.name           = "stmmaceth",
	.id             = 1,
	.dev = {
		.platform_data = &gmac1_plat_dat,
	},
	.num_resources  = ARRAY_SIZE(gmac1_resources),
	.resource       = gmac1_resources,
};

/*
 * AHCI
 */
static struct resource ahci_resources[] = {
	[0] = {
		.start = LS2H_SATA_REG_BASE,
		.end   = LS2H_SATA_REG_BASE + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_SATA_IRQ,
		.end   = LS2H_PCH_SATA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_ahci_device = {
	.name           = "ahci",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ahci_resources),
	.resource       = ahci_resources,
};

/*
 * RTC
 */
static struct resource rtc_resources[] = {
       [0] = {
               .start  = LS2H_RTC_REG_BASE,
               .end    = (LS2H_RTC_REG_BASE + 0xff),
               .flags  = IORESOURCE_MEM,
       },
       [1] = {
               .start  = LS2H_PCH_RTC_INT0_IRQ,
               .end    = LS2H_PCH_TOY_TICK_IRQ,
               .flags  = IORESOURCE_IRQ,
       },
};

static struct platform_device ls2h_rtc_device = {
       .name   = "ls2h-rtc",
       .id     = 0,
       .num_resources  = ARRAY_SIZE(rtc_resources),
       .resource       = rtc_resources,
};


/*
 * DC
 */
static struct resource dc_resources[] = {
	[0] = {
		.start	= LS2H_DC_REG_BASE,
		.end	= LS2H_DC_REG_BASE + 0x2000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LS2H_PCH_DC_IRQ,
		.end	= LS2H_PCH_DC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_dc_device = {
	.name           = "ls2h-fb",
	.id             = 0,
	.num_resources	= ARRAY_SIZE(dc_resources),
	.resource	= dc_resources,
};

/*
 * HD Audio
 */
static struct resource audio_resources[] = {
	[0] = {
		.start = LS2H_HDA_REG_BASE,
		.end   = LS2H_HDA_REG_BASE + 0x17f,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_HDA_IRQ,
		.end   = LS2H_PCH_HDA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_audio_device = {
	.name           = "ls2h-audio",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(audio_resources),
	.resource       = audio_resources,
};

/*
 * I2C
 */
static struct resource i2c0_resources[] = {
	[0] = {
		.start = LS2H_I2C0_REG_BASE,
		.end   = LS2H_I2C0_REG_BASE + 0x8,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_I2C0_IRQ,
		.end   = LS2H_PCH_I2C0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_i2c0_device = {
	.name           = "ls2h-i2c",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(i2c0_resources),
	.resource       = i2c0_resources,
};

static struct resource i2c1_resources[] = {
	[0] = {
		.start = LS2H_I2C1_REG_BASE,
		.end   = LS2H_I2C1_REG_BASE + 0x8,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_PCH_I2C1_IRQ,
		.end   = LS2H_PCH_I2C1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_i2c1_device = {
	.name           = "ls2h-i2c",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(i2c1_resources),
	.resource       = i2c1_resources,
};

/*
 * GPU
 */
static struct resource gpu_resources[] = {
	[0] = {
		.name	= "gpu_base",
		.start	= LS2H_GPU_REG_BASE,
		.end	= LS2H_GPU_REG_BASE + 0x7ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "gpu_irq",
		.start	= LS2H_PCH_GPU_IRQ,
		.end	= LS2H_PCH_GPU_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "gpu_mem",
		.start	= 0xe0004000000,
		.end	= 0xe000bffffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ls2h_gpu_device = {
	.name           = "galcore",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(gpu_resources),
	.resource       = gpu_resources,
};

/*
 * OTG
 */
static struct resource otg_resources[] = {
	[0] = {
		.start = LS2H_OTG_REG_BASE,
		.end   = (LS2H_OTG_REG_BASE + 0x3ffff),
		.flags = IORESOURCE_MEM,
        },
        [1] = {
		.start = LS2H_PCH_OTG_IRQ,
		.end   = LS2H_PCH_OTG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_otg_device = {
	.name           = "dwc2",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(otg_resources),
	.resource       = otg_resources,
};

static struct platform_device *ls2h_platform_devices[] = {
	&uart8250_device,
	&ls2h_i2c0_device,
	&ls2h_i2c1_device,
	&ls2h_nand_device,
	&ls2h_ohci_device,
	&ls2h_ehci_device,
	&ls2h_gmac0_device,
	&ls2h_gmac1_device,
	&ls2h_ahci_device,
	&ls2h_dc_device,
	&ls2h_audio_device,
	&ls2h_rtc_device,
	&ls2h_gpu_device,
	&ls2h_otg_device,
};

const struct i2c_board_info __initdata loongson_eep_info = {
	I2C_BOARD_INFO("eeprom-loongson", 0x50),
};

/*
 * PCI Controller
 */
static int nr_pci_ports;

static struct resource pci_mem_resource[4] = {
	{
		.name	= "pci memory space",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "pci memory space",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "pci memory space",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "pci memory space",
		.flags	= IORESOURCE_MEM,
	}
};

static struct resource pci_io_resource[4] = {
	{
		.name	= "pci io space",
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "pci io space",
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "pci io space",
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "pci io space",
		.flags	= IORESOURCE_IO,
	}
};

static struct pci_controller ls2h_pci_controller[4] = {
	{
		.pci_ops	= &ls2h_pci_ops[0],
		.mem_resource	= &pci_mem_resource[0],
		.io_resource	= &pci_io_resource[0],
		.mem_offset	= 0x00000000UL,
		.io_offset	= 0x00000000UL,
		.io_map_base	= 0x90000e0018000000UL,
	},
	{
		.pci_ops	= &ls2h_pci_ops[1],
		.mem_resource	= &pci_mem_resource[1],
		.io_resource	= &pci_io_resource[1],
		.mem_offset	= 0x00000000UL,
		.io_offset	= 0x00000000UL,
		.io_map_base	= 0x90000e0018000000UL,
	},
	{
		.pci_ops	= &ls2h_pci_ops[2],
		.mem_resource	= &pci_mem_resource[2],
		.io_resource	= &pci_io_resource[2],
		.mem_offset	= 0x00000000UL,
		.io_offset	= 0x00000000UL,
		.io_map_base	= 0x90000e0018000000UL,
	},
	{
		.pci_ops	= &ls2h_pci_ops[3],
		.mem_resource	= &pci_mem_resource[3],
		.io_resource	= &pci_io_resource[3],
		.mem_offset	= 0x00000000UL,
		.io_offset	= 0x00000000UL,
		.io_map_base	= 0x90000e0018000000UL,
	}
};

static void en_ref_clock(void)
{
	unsigned int data;

	data = readl(LS2H_CLK_CTRL3_REG);
	data |= (LS2H_CLK_CTRL3_BIT_PEREF_EN(0)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(1)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(2)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(3));
	writel(data, LS2H_CLK_CTRL3_REG);
}

static int pcie_is_x4_mode(void)
{
	u32 data = readl((u32 *)(LS2H_PCIE_PORT_REG_BASE(0) | LS2H_PCIE_PORT_REG_CTR_STAT));

	return data & LS2H_PCIE_REG_CTR_STAT_BIT_ISX4;
}

static void pcie_port_init(int port)
{
	unsigned int *reg, data;

	reg = (void *)(LS2H_PCIE_PORT_REG_BASE(port) | LS2H_PCIE_PORT_REG_CTR0);
	writel(0x00ff204c, reg);

	reg = (void *)(LS2H_PCIE_PORT_HEAD_BASE(port) | PCI_CLASS_REVISION);
	data = (readl(reg) & 0xffff) | (PCI_CLASS_BRIDGE_PCI << 16);
	writel(data, reg);

	reg = (void *)(LS2H_PCIE_PORT_HEAD_BASE(port) | LS2H_PCI_EXP_LNKCAP);
	data = (readl(reg) & (~0xf)) | 0x1;
	writel(data, reg);
}

static void ls2h_early_config(void)
{
	u32 i, val;

	/*
	 * Loongson-2H chip_config0: 0x1fd00200
	 * bit[5]: 	Loongson-2H bridge mode,0: disable      1: enable
	 * bit[4]:	ac97/hda select,	0: ac97		1: hda
	 * bit[14]:	host/otg select,	0: host         1: otg
	 * bit[26]:	usb reset,		0: enable       1: disable
	 */
	val = readl(LS2H_CHIP_CFG0_REG);
	writel(val | (1 << 5) | (1 << 4) | (1 << 14) | (1 << 26), LS2H_CHIP_CFG0_REG);

	val = readl(LS2H_GPIO_OE_REG);
	writel(val | (1 << 0), LS2H_GPIO_OE_REG);

	en_ref_clock();
	pcie_bus_config = PCIE_BUS_PERFORMANCE;

	val = readl((void *)(LS2H_PCIE_PORT_REG_BASE(0) | LS2H_PCIE_PORT_REG_CTR_STAT));
	val |= LS2H_PCIE_REG_CTR_STAT_BIT_ISRC;  /* Enable RC mode */
	writel(val, (void *)(LS2H_PCIE_PORT_REG_BASE(0) | LS2H_PCIE_PORT_REG_CTR_STAT));

	if (pcie_is_x4_mode())
		nr_pci_ports = 1;
	else
		nr_pci_ports = 4;

	for (i = 0; i < nr_pci_ports; i++)
		pcie_port_init(i);
}

static void __init ls2h_arch_initcall(void)
{
	u64 i, pci_mem_size;

	if (!loongson_sysconf.pci_mem_start_addr)
		loongson_sysconf.pci_mem_start_addr = LOONGSON_PCI_MEM_START;
	if (!loongson_sysconf.pci_mem_end_addr)
		loongson_sysconf.pci_mem_end_addr = LOONGSON_PCI_MEM_START + 0x40000000UL - 1;
	pci_mem_size = loongson_sysconf.pci_mem_end_addr - loongson_sysconf.pci_mem_start_addr + 1;

	ioport_resource.end = 0xffffffff;
	for (i = 0; i < nr_pci_ports; i++) {
		pci_io_resource[i].start = 0x400000*i + 0x100000;
		pci_io_resource[i].end   = 0x400000*i + 0x10ffff;
		pci_mem_resource[i].start =
			loongson_sysconf.pci_mem_start_addr + pci_mem_size*i/nr_pci_ports;
		pci_mem_resource[i].end   =
			loongson_sysconf.pci_mem_start_addr + pci_mem_size*(i+1)/nr_pci_ports - 1;
		register_pci_controller(&ls2h_pci_controller[i]);
	}
}

#define I2C_BUS_0 0
#define I2C_BUS_1 1

static void __init ls2h_device_initcall(void)
{
	int i;

	i2c_register_board_info(I2C_BUS_1, &loongson_eep_info, 1);

	platform_dma_mask = DMA_BIT_MASK(loongson_sysconf.dma_mask_bits);
	for (i=0; i<ARRAY_SIZE(ls2h_platform_devices); i++) {
		ls2h_platform_devices[i]->dev.dma_mask = &platform_dma_mask;
		ls2h_platform_devices[i]->dev.coherent_dma_mask = platform_dma_mask;
	}

	if (loongson_sysconf.vram_type == VRAM_TYPE_UMA) {
		gpu_resources[2].start = loongson_sysconf.uma_vram_addr;
		gpu_resources[2].end = loongson_sysconf.uma_vram_addr + (loongson_sysconf.uma_vram_size << 20) - 1;
	}
	platform_add_devices(ls2h_platform_devices,
			ARRAY_SIZE(ls2h_platform_devices));
}

struct platform_controller_hub ls2h_pch = {
	.type			= LS2H,
	.pcidev_max_funcs 	= 1,
	.early_config		= ls2h_early_config,
	.init_irq		= ls2h_init_irq,
	.irq_dispatch		= ls2h_irq_dispatch,
	.pcibios_map_irq	= ls2h_pcibios_map_irq,
	.pch_arch_initcall	= ls2h_arch_initcall,
	.pch_device_initcall	= ls2h_device_initcall,
};
