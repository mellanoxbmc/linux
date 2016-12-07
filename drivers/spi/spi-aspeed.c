#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>

#define ASPEED_SPI_CONFIG_REG			0x00
#define ASPEED_SPI_CONFIG_WRITE_CE1_EN			BIT(17)
#define ASPEED_SPI_CONFIG_WRITE_CE0_EN			BIT(16)
#define ASPEED_SPI_CE_CTRL_REG			0x04
#define ASPEED_SPI_CE_CTRL_CE_SWAP			BIT(31)
#define ASPEED_SPI_CE_CTRL_CE1_HALF_SPEED_EN		BIT(9)
#define ASPEED_SPI_CE_CTRL_CE0_HALF_SPEED_EN		BIT(8)
#define ASPEED_SPI_CE_CTRL_CE1_4_BYTE_ADDR_EN		BIT(1)
#define ASPEED_SPI_CE_CTRL_CE0_4_BYTE_ADDR_EN		BIT(0)
#define ASPEED_SPI_IRQ_REG			0x08
#define ASPEED_SPI_IRQ_CMD_ABORT_STS			BIT(10)
#define ASPEED_SPI_IRQ_WRITE_PROTECTED_STS		BIT(9)
#define ASPEED_SPI_IRQ_CMD_ABORT_EN			BIT(2)
#define ASPEED_SPI_IRQ_WRITE_PROTECTED_EN		BIT(1)
#define ASPEED_SPI_CMD_CTRL_REG			0x0c
#define ASPEED_SPI_CMD_CTRL_ADDR_BYTE_LANE_DIS_MASK	GENMASK(7, 4)
#define ASPEED_SPI_CMD_CTRL_DATA_BYTE_LANE_DIS_MASK	GENMASK(3, 0)
#define ASPEED_SPI_CE0_CTRL_REG			0x10
#define ASPEED_SPI_CE1_CTRL_REG			0x14
#define ASPEED_SPI_CEX_CTRL_IO_MODE_MASK		GENMASK(29, 28)
#define ASPEED_SPI_CEX_CTRL_INACTIVE_PULSE_WIDTH	GENMASK(27, 24)
#define ASPEED_SPI_CEX_CTRL_CMD_DATA			GENMASK(23, 16)
#define ASPEED_SPI_CEX_CTRL_DUMMY_CYCLE_CMD_OUT_EN	BIT(15)
#define ASPEED_SPI_CEX_CTRL_FAST_READ_DUMMY_CYCLE_EN	BIT(14)
#define ASPEED_SPI_CEX_CTRL_DIV_CLK_BY_4		BIT(13)
#define ASPEED_SPI_CEX_CTRL_FLASH_RD_WR_MERGE_DIS	BIT(12)
#define ASPEED_SPI_CEX_CTRL_CLK_DIV_SHIFT		8
#define ASPEED_SPI_CEX_CTRL_CLK_DIV			GENMASK(11, 8)
#define ASPEED_SPI_CEX_CTRL_FAST_READ_DUMMY_CYCLE_NUM	GENMASK(7, 6)
#define ASPEED_SPI_CEX_CTRL_LSB_EN			BIT(5)
#define ASPEED_SPI_CEX_CTRL_CLK_MODE_3_ON_START_EN	BIT(4)
#define ASPEED_SPI_CEX_CTRL_DUAL_DATA_MODE_EN		BIT(3)
#define ASPEED_SPI_CEX_CTRL_STOP_ACTIVE			BIT(2)
#define ASPEED_SPI_CEX_CTRL_USER_CMD_MODE		(BIT(0) | BIT(1))
#define ASPEED_SPI_CE0_ADDR_REG			0x30
#define ASPEED_SPI_CE1_ADDR_REG			0x34
#define ASPEED_SPI_CEX_ADDR_END_HIGH_MASK		GENMASK(31, 28)
#define ASPEED_SPI_CEX_ADDR_END_LOW_MASK		GENMASK(27, 24)
#define ASPEED_SPI_CEX_ADDR_START_HIGH_MASK		GENMASK(23, 20)
#define ASPEED_SPI_CEX_ADDR_START_LOW_MASK		GENMASK(19, 16)
#define ASPEED_SPI_CEX_ADDR_MASK			GENMASK(30, 23)
#define ASPEED_SPI_DUMMY_CYCLE_REG		0x54
#define ASPEED_SPI_DUMMY_CYCLE_DATA			GENMASK(7, 0)
#define ASPEED_SPI_READ_TIME_REG		0x94
#define ASPEED_SPI_READ_TIME_NO_INPUT_DELAY_CYCLE_MASK	GENMASK(19, 0)

#define ASPEED_SPI_ADDRS_TO_CEX_ADDR_REG(start, end)			       \
		((((end) & ASPEED_SPI_CEX_ADDR_MASK) << 1) |		       \
		 (((start) & ASPEED_SPI_CEX_ADDR_MASK) >> 7))

#define ASPEED_SPI_BUS_NUM			2

/*
 * TODO: This is taken from drivers/mtd/spi-nor/aspeed-smc.c we should discuss
 * sharing code between the two implementations. It appears that there is a
 * spi_device implementation of a spi_nor device, so it probably makes sense to
 * just move the entire non-dma implementation here.
 *
 * In user mode all data bytes read or written to the chip decode address
 * range are transferred to or from the SPI bus. The range is treated as a
 * fifo of arbitratry 1, 2, or 4 byte width but each write has to be aligned
 * to its size.  The address within the multiple 8kB range is ignored when
 * sending bytes to the SPI bus.
 *
 * On the arm architecture, as of Linux version 4.3, memcpy_fromio and
 * memcpy_toio on little endian targets use the optimized memcpy routines
 * that were designed for well behavied memory storage.  These routines
 * have a stutter if the source and destination are not both word aligned,
 * once with a duplicate access to the source after aligning to the
 * destination to a word boundary, and again with a duplicate access to
 * the source when the final byte count is not word aligned.
 *
 * When writing or reading the fifo this stutter discards data or sends
 * too much data to the fifo and can not be used by this driver.
 *
 * While the low level io string routines that implement the insl family do
 * the desired accesses and memory increments, the cross architecture io
 * macros make them essentially impossible to use on a memory mapped address
 * instead of a a token from the call to iomap of an io port.
 *
 * These fifo routines use readl and friends to a constant io port and update
 * the memory buffer pointer and count via explicit code. The final updates
 * to len are optimistically suppressed.
 */
static int aspeed_spi_read_from_ahb(void *buf, const void __iomem *src,
				    size_t len)
{
	if ((((unsigned long) src | (unsigned long) buf | len) & 3) == 0) {
		while (len > 3) {
			*(u32 *) buf = readl(src);
			buf += 4;
			src += 4;
			len -= 4;
		}
	}

	while (len--) {
		*(u8 *) buf = readb(src);
		buf += 1;
		src += 1;
	}
	return 0;
}

static int aspeed_spi_write_to_ahb(void __iomem *dst, const void *buf,
				   size_t len)
{
	if ((((unsigned long) dst | (unsigned long) buf | len) & 3) == 0) {
		while (len > 3) {
			u32 val = *(u32 *) buf;

			writel(val, dst);
			buf += 4;
			dst += 4;
			len -= 4;
		}
	}

	while (len--) {
		u8 val = *(u8 *) buf;

		writeb(val, dst);
		buf += 1;
		dst += 1;
	}
	return 0;
}

struct aspeed_spi_bus {
	void __iomem		*cs_ctrl_reg;
	void __iomem		*cs_addr_reg;
	void __iomem		*io_port;
	phys_addr_t		io_port_pa;
	size_t			port_size;
};

struct aspeed_spi_master {
	struct device		*dev;
	void __iomem		*base;
	struct aspeed_spi_bus	bus[ASPEED_SPI_BUS_NUM];
};

static u32 read_reg(struct aspeed_spi_master *master, loff_t offset)
{
	return readl(master->base + offset);
}

static void write_reg(u32 value, struct aspeed_spi_master *master,
		      loff_t offset)
{
	writel(value, master->base + offset);
}

static irqreturn_t aspeed_spi_irq(int irq, void *arg)
{
	struct aspeed_spi_master *master = arg;
	u32 irq_reg;

	dev_err(master->dev, "received error interrupt.\n");

	irq_reg = read_reg(master, ASPEED_SPI_IRQ_REG);

	if (irq_reg & ASPEED_SPI_IRQ_CMD_ABORT_STS)
		dev_err(master->dev, "command aborted.\n");

	/* This error should never occur as we do not use the addressed r/w
	 * features of the controller in this driver.
	 */
	if (irq_reg & ASPEED_SPI_IRQ_WRITE_PROTECTED_STS)
		dev_err(master->dev, "attempted write to protected address.\n");

	/* Clear handled interrupts. */
	write_reg(irq_reg, master, ASPEED_SPI_IRQ_REG);
	return IRQ_HANDLED;
}

static size_t aspeed_spi_max_transfer_size(struct spi_device *device)
{
	struct aspeed_spi_master *master =
			spi_master_get_devdata(device->master);

	if (unlikely(device->chip_select > 1)) {
		WARN(true, "attempted to set invalid chip select: %d\n",
			device->chip_select);
		return 0;
	}

	return master->bus[device->chip_select].port_size;
}

static void aspeed_spi_set_cs(struct spi_device *device, bool cs_logic_level)
{
	struct aspeed_spi_master *master =
			spi_master_get_devdata(device->master);
	struct aspeed_spi_bus *bus;
	u32 cs_ctrl;

	if (unlikely(device->chip_select > 1)) {
		WARN(true, "attempted to set invalid chip select: %d\n",
			device->chip_select);
		return;
	}
	bus = &master->bus[device->chip_select];

	cs_ctrl = readl(bus->cs_ctrl_reg);
	/* device is requesting CS go low (active), so we need to configure the
	 * bus according to that device's settings. cs_ctrl is a per bus/device
	 * register, so it is safe to update these on chip select.
	 */
	if (!cs_logic_level) {
		if ((device->mode & (SPI_CPOL | SPI_CPHA)) == SPI_MODE_3)
			cs_ctrl |= ASPEED_SPI_CEX_CTRL_CLK_MODE_3_ON_START_EN;
		else /* SPI_MODE_0 by default */
			cs_ctrl &= ~ASPEED_SPI_CEX_CTRL_CLK_MODE_3_ON_START_EN;

		if (device->mode & SPI_LSB_FIRST)
			cs_ctrl |= ASPEED_SPI_CEX_CTRL_LSB_EN;
		else /* MSB first by default */
			cs_ctrl &= ~ASPEED_SPI_CEX_CTRL_LSB_EN;

		cs_ctrl &= ~ASPEED_SPI_CEX_CTRL_STOP_ACTIVE;
		cs_ctrl |= ASPEED_SPI_CEX_CTRL_USER_CMD_MODE;

		writel(cs_ctrl, bus->cs_ctrl_reg);
	} else
		writel(cs_ctrl | ASPEED_SPI_CEX_CTRL_STOP_ACTIVE,
		       bus->cs_ctrl_reg);
}

static u8 aspeed_spi_compute_divider(u32 requested_speed, u32 base_speed)
{
	u32 divider;

	if (requested_speed > base_speed)
		return 0xf; /* no divider */

	divider = base_speed / requested_speed;
	if (base_speed % requested_speed > 0)
		divider++;

	if (divider > 16) {
		WARN_ONCE(true,
			  "device requested clock speed, %u, less than minimum clock speed, %u.\n",
			  requested_speed, base_speed / 16);
		divider = 16;
	}

	if (divider % 2 == 0)
		return (16 - divider) / 2;
	else
		return 8 + (15 - divider) / 2;
}

static void aspeed_spi_update_clk_for_xfer(struct spi_master *spi_master,
					  struct spi_device *device,
					  struct spi_transfer *xfer)
{
	struct aspeed_spi_master *master = spi_master_get_devdata(spi_master);
	struct aspeed_spi_bus *bus = &master->bus[device->chip_select];
	u32 requested_speed, base_speed = spi_master->max_speed_hz, reg_val;
	u8 divider;

	if (xfer->speed_hz)
		requested_speed = xfer->speed_hz;
	else
		requested_speed = device->max_speed_hz;
	divider = aspeed_spi_compute_divider(requested_speed, base_speed);

	reg_val = readl(bus->cs_ctrl_reg);
	reg_val &= ~ASPEED_SPI_CEX_CTRL_CLK_DIV;
	reg_val |= divider << ASPEED_SPI_CEX_CTRL_CLK_DIV_SHIFT;
}

static int aspeed_spi_transfer_one(struct spi_master *spi_master,
				   struct spi_device *device,
				   struct spi_transfer *transfer)
{
	struct aspeed_spi_master *master = spi_master_get_devdata(spi_master);
	struct aspeed_spi_bus *bus;
	int ret = 0;

	if (device->chip_select > 1) {
		dev_err(master->dev,
			"attempted to transfer to device with invalid chip select: %d\n",
			device->chip_select);
		return -EINVAL;
	}
	bus = &master->bus[device->chip_select];

	if (transfer->len > bus->port_size) {
		dev_err(master->dev,
			"attempted transfer with larger than supported size: %u\n",
			transfer->len);
		return -EINVAL;
	}

	aspeed_spi_update_clk_for_xfer(spi_master, device, transfer);

	if (transfer->tx_buf)
		aspeed_spi_write_to_ahb(bus->io_port,
					transfer->tx_buf, transfer->len);

	if (transfer->rx_buf)
		aspeed_spi_read_from_ahb(transfer->rx_buf,
					 bus->io_port, transfer->len);

	return ret;
}

static int aspeed_spi_init(struct aspeed_spi_master *master)
{
	u32 start_addr, end_addr, reg_val, partial_reg_val;
	struct aspeed_spi_bus *bus;
	int i;

	reg_val = read_reg(master, ASPEED_SPI_CONFIG_REG);
	reg_val |= ASPEED_SPI_CONFIG_WRITE_CE0_EN |
			ASPEED_SPI_CONFIG_WRITE_CE1_EN;
	write_reg(reg_val, master, ASPEED_SPI_CONFIG_REG);

	reg_val = read_reg(master, ASPEED_SPI_CE_CTRL_REG);
	reg_val &= ~(ASPEED_SPI_CE_CTRL_CE_SWAP |
		     ASPEED_SPI_CE_CTRL_CE1_4_BYTE_ADDR_EN |
		     ASPEED_SPI_CE_CTRL_CE1_4_BYTE_ADDR_EN);
	reg_val |= ASPEED_SPI_CE_CTRL_CE1_HALF_SPEED_EN |
			ASPEED_SPI_CE_CTRL_CE0_HALF_SPEED_EN;
	write_reg(reg_val, master, ASPEED_SPI_CE_CTRL_REG);

	reg_val = read_reg(master, ASPEED_SPI_IRQ_REG);
	reg_val |= ASPEED_SPI_IRQ_CMD_ABORT_EN |
			ASPEED_SPI_IRQ_WRITE_PROTECTED_EN;
	write_reg(reg_val, master, ASPEED_SPI_IRQ_REG);
	/* Clear any interrupt status bits from previous operation. */
	reg_val |= ASPEED_SPI_IRQ_CMD_ABORT_STS |
			ASPEED_SPI_IRQ_WRITE_PROTECTED_STS;
	write_reg(reg_val, master, ASPEED_SPI_IRQ_REG);

	reg_val = read_reg(master, ASPEED_SPI_CMD_CTRL_REG);
	reg_val &= ~(ASPEED_SPI_CMD_CTRL_ADDR_BYTE_LANE_DIS_MASK |
		     ASPEED_SPI_CMD_CTRL_DATA_BYTE_LANE_DIS_MASK);
	write_reg(reg_val, master, ASPEED_SPI_CMD_CTRL_REG);

	for (i = 0; i < ASPEED_SPI_BUS_NUM; i++) {
		bus = &master->bus[i];

		reg_val = readl(bus->cs_ctrl_reg);
		reg_val &= ~(ASPEED_SPI_CEX_CTRL_IO_MODE_MASK |
			     ASPEED_SPI_CEX_CTRL_INACTIVE_PULSE_WIDTH |
			     ASPEED_SPI_CEX_CTRL_CMD_DATA |
			     ASPEED_SPI_CEX_CTRL_DUMMY_CYCLE_CMD_OUT_EN |
			     ASPEED_SPI_CEX_CTRL_FAST_READ_DUMMY_CYCLE_EN |
			     ASPEED_SPI_CEX_CTRL_DIV_CLK_BY_4 |
			     ASPEED_SPI_CEX_CTRL_CLK_DIV |
			     ASPEED_SPI_CEX_CTRL_FAST_READ_DUMMY_CYCLE_NUM |
			     ASPEED_SPI_CEX_CTRL_LSB_EN |
			     ASPEED_SPI_CEX_CTRL_CLK_MODE_3_ON_START_EN |
			     ASPEED_SPI_CEX_CTRL_DUAL_DATA_MODE_EN);
		reg_val |= ASPEED_SPI_CEX_CTRL_FLASH_RD_WR_MERGE_DIS |
				ASPEED_SPI_CEX_CTRL_STOP_ACTIVE |
				ASPEED_SPI_CEX_CTRL_USER_CMD_MODE;

		/* The SPI controller needs to know about where the ports used
		 * to access it are as it is actually designed to be a SPI flash
		 * controller. It provides a range that the ports can live in
		 * and then allows a sub-range within that range to be defined
		 * as the actual ports.
		 *
		 * So we
		 * 1) find the address range we were given for the IO ports
		 * 2) convert the address range to the same format as the
		 *    register
		 * 3) compare the portions which cannot be modified to verify
		 *    the range is valid
		 * 4) write back the portions that can be modified
		 */
		reg_val = readl(bus->cs_addr_reg);
		start_addr = (u32) bus->io_port_pa;
		end_addr = start_addr + bus->port_size;
		partial_reg_val = ASPEED_SPI_ADDRS_TO_CEX_ADDR_REG(start_addr,
								   end_addr);
		if ((partial_reg_val & (ASPEED_SPI_CEX_ADDR_END_HIGH_MASK |
					ASPEED_SPI_CEX_ADDR_START_HIGH_MASK)) !=
		    (reg_val & (ASPEED_SPI_CEX_ADDR_END_HIGH_MASK |
				ASPEED_SPI_CEX_ADDR_START_HIGH_MASK))) {
			dev_err(master->dev,
				"invalid spi port address range requested for CE%d: 0x%06x - 0x%06x\n",
				i, start_addr, end_addr);
			return -EINVAL;
		}
		reg_val &= ~(ASPEED_SPI_CEX_ADDR_END_LOW_MASK |
			     ASPEED_SPI_CEX_ADDR_START_LOW_MASK);
		reg_val |= partial_reg_val &
				(ASPEED_SPI_CEX_ADDR_END_LOW_MASK |
				 ASPEED_SPI_CEX_ADDR_START_LOW_MASK);
		writel(reg_val, bus->cs_addr_reg);
	}

	reg_val = read_reg(master, ASPEED_SPI_DUMMY_CYCLE_REG);
	reg_val &= ~ASPEED_SPI_DUMMY_CYCLE_DATA;
	write_reg(reg_val, master, ASPEED_SPI_DUMMY_CYCLE_REG);

	reg_val = read_reg(master, ASPEED_SPI_READ_TIME_REG);
	reg_val &= ~ASPEED_SPI_READ_TIME_NO_INPUT_DELAY_CYCLE_MASK;
	write_reg(reg_val, master, ASPEED_SPI_READ_TIME_REG);

	return 0;
}

static int aspeed_spi_probe(struct platform_device *pdev)
{
	struct aspeed_spi_master *aspeed_spi_master;
	struct spi_master *spi_master;
	struct aspeed_spi_bus *bus;
	struct resource *res;
	int irq, ret = 0;

	spi_master = spi_alloc_master(&pdev->dev,
				      sizeof(struct aspeed_spi_master));
	if (!spi_master)
		return -ENOMEM;
	aspeed_spi_master = spi_master_get_devdata(spi_master);
	platform_set_drvdata(pdev, spi_master);

	aspeed_spi_master->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "could not get register memory.\n");
		ret = -ENOMEM;
		goto err_out;
	}
	aspeed_spi_master->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(aspeed_spi_master->base)) {
		dev_err(&pdev->dev, "could not ioremap register memory.\n");
		ret = PTR_ERR(aspeed_spi_master->base);
		goto err_out;
	}

	bus = &aspeed_spi_master->bus[0];
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "could not get port memory.\n");
		ret = -ENOMEM;
		goto err_out;
	}
	bus->cs_ctrl_reg = aspeed_spi_master->base + ASPEED_SPI_CE0_CTRL_REG;
	bus->cs_addr_reg = aspeed_spi_master->base + ASPEED_SPI_CE0_ADDR_REG;
	bus->io_port = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bus->io_port)) {
		dev_err(&pdev->dev, "could not ioremap port.\n");
		ret = PTR_ERR(bus->io_port);
		goto err_out;
	}
	bus->io_port_pa = res->start;
	bus->port_size = resource_size(res);

	/* TODO: Maybe we should not require a separate port window to be
	 * specified as a user might not want to use all chip selects. In any
	 * case, we should support the third chipselect for the fmc.
	 */
	bus = &aspeed_spi_master->bus[1];
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "could not get port memory.\n");
		ret = -ENOMEM;
		goto err_out;
	}
	bus->cs_ctrl_reg = aspeed_spi_master->base + ASPEED_SPI_CE1_CTRL_REG;
	bus->cs_addr_reg = aspeed_spi_master->base + ASPEED_SPI_CE1_ADDR_REG;
	bus->io_port = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bus->io_port)) {
		dev_err(&pdev->dev, "could not ioremap port.\n");
		ret = PTR_ERR(bus->io_port);
		goto err_out;
	}
	bus->io_port_pa = res->start;
	bus->port_size = resource_size(res);

	/* This should happen before initializing the IRQ handler as this could
	 * clear left over IRQs from previous operation.
	 */
	ret = aspeed_spi_init(aspeed_spi_master);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not initialize controller.\n");
		goto err_out;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "could not get irq.\n");
		ret = irq;
		goto err_out;
	}
	ret = devm_request_irq(&pdev->dev, irq, aspeed_spi_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), aspeed_spi_master);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request irq.\n");
		goto err_out;
	}

	spi_master->num_chipselect = ASPEED_SPI_BUS_NUM;
	/* TODO: Should also be able to support 2 bit transfers. */
	spi_master->mode_bits = SPI_MODE_0 | SPI_MODE_3 | SPI_LSB_FIRST;
	spi_master->bits_per_word_mask = SPI_BPW_MASK(8);
	/* TODO: This is just the clock speed of the AHB; should we make a fixed
	 * frequency clock for the AHB and use that here?
	 */
	spi_master->max_speed_hz = 100000000;
	spi_master->min_speed_hz = spi_master->max_speed_hz / 16;
	spi_master->flags = SPI_MASTER_HALF_DUPLEX;

	spi_master->max_transfer_size = aspeed_spi_max_transfer_size;
	spi_master->set_cs = aspeed_spi_set_cs;
	spi_master->transfer_one = aspeed_spi_transfer_one;
	spi_master->dev.of_node = pdev->dev.of_node;

	platform_set_drvdata(pdev, spi_master);
	ret = devm_spi_register_master(&pdev->dev, spi_master);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register spi master.\n");
		goto err_out;
	}
	dev_info(&pdev->dev, "registered spi master.\n");
	return 0;

err_out:
	spi_master_put(spi_master);
	return ret;
}

static int aspeed_spi_remove(struct platform_device *pdev)
{
	struct spi_master *spi_master = platform_get_drvdata(pdev);
	struct aspeed_spi_master *master = spi_master_get_devdata(spi_master);
	struct aspeed_spi_bus *bus;
	u32 reg_val;
	int i;

	reg_val = read_reg(master, ASPEED_SPI_IRQ_REG);
	reg_val &= ~(ASPEED_SPI_IRQ_CMD_ABORT_EN |
		     ASPEED_SPI_IRQ_WRITE_PROTECTED_EN);
	write_reg(reg_val, master, ASPEED_SPI_IRQ_REG);

	for (i = 0; i < ASPEED_SPI_BUS_NUM; i++) {
		bus = &master->bus[i];
		reg_val = readl(bus->cs_ctrl_reg);
		writel(reg_val | ASPEED_SPI_CEX_CTRL_STOP_ACTIVE,
		       bus->cs_ctrl_reg);
	}

	spi_master_put(spi_master);

	return 0;
}

static const struct of_device_id aspeed_spi_ids[] = {
	{ .compatible = "aspeed,ast2500-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_spi_ids);

static struct platform_driver aspeed_spi_driver = {
	.driver = {
		.name = "aspeed-spi",
		.of_match_table = aspeed_spi_ids,
	},
	.probe = aspeed_spi_probe,
	.remove = aspeed_spi_remove,
};
module_platform_driver(aspeed_spi_driver);

MODULE_DESCRIPTION("Aspeed 25XX generic half-duplex SPI driver");
MODULE_AUTHOR("Brendan Higgins <brendanhiggins@google.com>");
MODULE_LICENSE("GPL");
