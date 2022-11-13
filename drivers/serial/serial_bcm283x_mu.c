// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Stephen Warren <swarren@wwwdotorg.org>
 *
 * Derived from pl01x code:
 *
 * (C) Copyright 2000
 * Rob Taylor, Flying Pig Systems. robt@flyingpig.com.
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 */

/* Simple U-Boot driver for the BCM283x mini UART */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <watchdog.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <serial.h>
#include <dm/platform_data/serial_bcm283x_mu.h>
#include <dm/pinctrl.h>
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <debug_uart.h>

struct bcm283x_mu_regs {
	u32 io;
	u32 iir;
	u32 ier;
	u32 lcr;
	u32 mcr;
	u32 lsr;
	u32 msr;
	u32 scratch;
	u32 cntl;
	u32 stat;
	u32 baud;
};

#define BCM283X_MU_LCR_DATA_SIZE_8	3

#define BCM283X_MU_LSR_TX_IDLE		BIT(6)
/* This actually means not full, but is named not empty in the docs */
#define BCM283X_MU_LSR_TX_EMPTY		BIT(5)
#define BCM283X_MU_LSR_RX_READY		BIT(0)

struct bcm283x_mu_priv {
	struct bcm283x_mu_regs *regs;
};

static int bcm283x_mu_serial_getc(struct udevice *dev);

static int bcm283x_mu_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct bcm283x_mu_serial_plat *plat = dev_get_plat(dev);
	struct bcm283x_mu_priv *priv = dev_get_priv(dev);
	struct bcm283x_mu_regs *regs = priv->regs;
	u32 divider;

	if (plat->skip_init)
		goto out;

	divider = plat->clock / (baudrate * 8);

	writel(BCM283X_MU_LCR_DATA_SIZE_8, &regs->lcr);
	writel(divider - 1, &regs->baud);

out:
	/* Flush the RX queue - all data in there is bogus */
	while (bcm283x_mu_serial_getc(dev) != -EAGAIN) ;

	return 0;
}

static int bcm283x_mu_serial_getc(struct udevice *dev)
{
	struct bcm283x_mu_priv *priv = dev_get_priv(dev);
	struct bcm283x_mu_regs *regs = priv->regs;
	u32 data;

	/* Wait until there is data in the FIFO */
	if (!(readl(&regs->lsr) & BCM283X_MU_LSR_RX_READY))
		return -EAGAIN;

	data = readl(&regs->io);

	return (int)data;
}

static int bcm283x_mu_serial_putc(struct udevice *dev, const char data)
{
	struct bcm283x_mu_priv *priv = dev_get_priv(dev);
	struct bcm283x_mu_regs *regs = priv->regs;

	/* Wait until there is space in the FIFO */
	if (!(readl(&regs->lsr) & BCM283X_MU_LSR_TX_EMPTY))
		return -EAGAIN;

	/* Send the character */
	writel(data, &regs->io);

	return 0;
}

static int bcm283x_mu_serial_pending(struct udevice *dev, bool input)
{
	struct bcm283x_mu_priv *priv = dev_get_priv(dev);
	struct bcm283x_mu_regs *regs = priv->regs;
	unsigned int lsr;

	lsr = readl(&regs->lsr);

	if (input) {
		schedule();
		return (lsr & BCM283X_MU_LSR_RX_READY) ? 1 : 0;
	} else {
		return (lsr & BCM283X_MU_LSR_TX_IDLE) ? 0 : 1;
	}
}

static const struct dm_serial_ops bcm283x_mu_serial_ops = {
	.putc = bcm283x_mu_serial_putc,
	.pending = bcm283x_mu_serial_pending,
	.getc = bcm283x_mu_serial_getc,
	.setbrg = bcm283x_mu_serial_setbrg,
};

#if CONFIG_IS_ENABLED(OF_CONTROL)
static const struct udevice_id bcm283x_mu_serial_id[] = {
	{.compatible = "brcm,bcm2835-aux-uart"},
	{}
};

/*
 * Check if this serial device is muxed
 *
 * The serial device will only work properly if it has been muxed to the serial
 * pins by firmware. Check whether that happened here.
 *
 * Return: true if serial device is muxed, false if not
 */
static bool bcm283x_is_serial_muxed(void)
{
	int serial_gpio = 15;
	struct udevice *dev;

	if (uclass_first_device_err(UCLASS_PINCTRL, &dev))
		return false;

	if (pinctrl_get_gpio_mux(dev, 0, serial_gpio) != BCM2835_GPIO_ALT5)
		return false;

	return true;
}

static int bcm283x_mu_serial_probe(struct udevice *dev)
{
	struct bcm283x_mu_serial_plat *plat = dev_get_plat(dev);
	struct bcm283x_mu_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;

	/* Don't spawn the device if it's not muxed */
	if (!bcm283x_is_serial_muxed())
		return -ENODEV;

	/*
	 * Read the ofdata here rather than in an of_to_plat() method
	 * since we need the soc simple-bus to be probed so that the 'ranges'
	 * property is used.
	 */
	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->base = addr;
	plat->clock = dev_read_u32_default(dev, "clock", 1);

	/*
	 * TODO: Reinitialization doesn't always work for now, just skip
	 *       init always - we know we're already initialized
	 */
	plat->skip_init = true;

	priv->regs = (struct bcm283x_mu_regs *)plat->base;

	return 0;
}
#endif

U_BOOT_DRIVER(serial_bcm283x_mu) = {
	.name = "serial_bcm283x_mu",
	.id = UCLASS_SERIAL,
	.of_match = of_match_ptr(bcm283x_mu_serial_id),
	.plat_auto	= sizeof(struct bcm283x_mu_serial_plat),
	.probe = bcm283x_mu_serial_probe,
	.ops = &bcm283x_mu_serial_ops,
#if !CONFIG_IS_ENABLED(OF_CONTROL) || CONFIG_IS_ENABLED(OF_BOARD)
	.flags = DM_FLAG_PRE_RELOC,
#endif
	.priv_auto	= sizeof(struct bcm283x_mu_priv),
};

#ifdef CONFIG_DEBUG_UART_BCM_MU

#define DEVICE_BASE 0x3F000000
#define PBASE	    (DEVICE_BASE)

#define GPFSEL0	  (PBASE + 0x00200000)
#define GPFSEL1	  (PBASE + 0x00200004)
#define GPFSEL2	  (PBASE + 0x00200008)
#define GPFSEL3	  (PBASE + 0x0020000C)
#define GPFSEL4	  (PBASE + 0x00200010)
#define GPFSEL5	  (PBASE + 0x00200014)
#define GPSET0	  (PBASE + 0x0020001C)
#define GPSET1	  (PBASE + 0x00200020)
#define GPCLR0	  (PBASE + 0x00200028)
#define GPLEV0	  (PBASE + 0x00200034)
#define GPLEV1	  (PBASE + 0x00200038)
#define GPEDS0	  (PBASE + 0x00200040)
#define GPEDS1	  (PBASE + 0x00200044)
#define GPHEN0	  (PBASE + 0x00200064)
#define GPHEN1	  (PBASE + 0x00200068)
#define GPPUD	  (PBASE + 0x00200094)
#define GPPUDCLK0 (PBASE + 0x00200098)
#define GPPUDCLK1 (PBASE + 0x0020009C)

#define AUX_IRQ		(PBASE + 0x00215000)
#define AUX_ENABLES	(PBASE + 0x00215004)
#define AUX_MU_IO_REG	(PBASE + 0x00215040)
#define AUX_MU_IER_REG	(PBASE + 0x00215044)
#define AUX_MU_IIR_REG	(PBASE + 0x00215048)
#define AUX_MU_LCR_REG	(PBASE + 0x0021504C)
#define AUX_MU_MCR_REG	(PBASE + 0x00215050)
#define AUX_MU_LSR_REG	(PBASE + 0x00215054)
#define AUX_MU_MSR_REG	(PBASE + 0x00215058)
#define AUX_MU_SCRATCH	(PBASE + 0x0021505C)
#define AUX_MU_CNTL_REG (PBASE + 0x00215060)
#define AUX_MU_STAT_REG (PBASE + 0x00215064)
#define AUX_MU_BAUD_REG (PBASE + 0x00215068)

void uart_send(char c)
{
	while (1) {
		if (readl(AUX_MU_LSR_REG) & 0x20)
			break;
	}
	writel(c, AUX_MU_IO_REG);
}

static inline void _debug_uart_init(void)
{
	// unsigned int selector;

	// selector = readl(GPFSEL1);
	// selector &= ~(7 << 12); // clean gpio14
	// selector |= 2 << 12; // set alt5 for gpio14
	// selector &= ~(7 << 15); // clean gpio15
	// selector |= 2 << 15; // set alt5 for gpio15
	// writel(GPFSEL1, selector);

	// writel(GPPUD, 0);
	// delay(150);
	// writel(GPPUDCLK0, (1 << 14) | (1 << 15));
	// delay(150);
	// writel(GPPUDCLK0, 0);

	writel(1, AUX_ENABLES); //Enable mini uart (this also enables access to it registers)
	writel(0, AUX_MU_CNTL_REG); //Disable auto flow control and disable receiver and transmitter (for now)
	writel(0, AUX_MU_IER_REG); //Disable receive and transmit interrupts
	writel(3, AUX_MU_LCR_REG); //Enable 8 bit mode
	writel(0, AUX_MU_MCR_REG); //Set RTS line to be always high
	writel(270, AUX_MU_BAUD_REG); //Set baud rate to 115200

	writel(3, AUX_MU_CNTL_REG); //Finally, enable transmitter and receiver
}

static inline void _debug_uart_putc(int ch)
{
	uart_send(ch);
}

DEBUG_UART_FUNCS
#endif
