#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>

#include "pcie-designware.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fred Heinecke");
MODULE_DESCRIPTION("RK3588 PCIe 3.0 endpoint controller");
MODULE_VERSION("0.01");
MODULE_SOFTDEP("pre: dw-edma-core");

/*
 * The upper 16 bits of PCIE_CLIENT_CONFIG are a write
 * mask for the lower 16 bits.
 */
#define HIWORD_UPDATE(mask, val) (((mask) << 16) | (val))
#define HIWORD_UPDATE_BIT(val) HIWORD_UPDATE(val, val)
#define HIWORD_DISABLE_BIT(val) HIWORD_UPDATE(val, ~val)

#define PCIE_CLIENT_GENERAL_CONTROL 0x0000
#define PCIE_CLIENT_INTR_MASK_MISC 0x0024
#define PCIE_CLIENT_HOT_RESET_CONTROL 0x0180
#define PCIE_CLIENT_LTSSM_STATUS 0x0300
#define USP_PCIE_TYPE0 0x0004

#define PCIE_CLIENT_INTR_MASK_MISC_ENABLE_DLL_UP HIWORD_UPDATE_BIT(0x2)
#define PCIE_CLIENT_INTR_MASK_MISC_ENABLE_REQ_RST HIWORD_UPDATE_BIT(0x3)
#define PCIE_CLIENT_ENABLE_LTSSM HIWORD_UPDATE_BIT(0x3)
#define PCIE_CLIENT_DISABLE_LTSSM HIWORD_DISABLE_BIT(0x3)
#define PCIE_CLIENT_ENABLE_LTSSM_ENHANCE HIWORD_UPDATE_BIT(0x5)
#define PCIE_CLIENT_SET_EP_MODE HIWORD_DISABLE_BIT(0xF0) // Clear the device_type field, 4'h0 = endpoint
#define USP_PCIE_TYPE0_ENABLE_BUS_MASTER HIWORD_UPDATE_BIT(0x3)
#define USP_PCIE_TYPE0_MEMORY_SPACE HIWORD_UPDATE_BIT(0x2)

#define PCIE_CLIENT_TEST_INTR_MASK_MISC_DLL_UP (0x2)
#define PCIE_CLIENT_TEST_INTR_MASK_MISC_REQ_RST (0x3)
#define PCIE_CLIENT_TEST_LTSSM_STATUS_LINK_UP (0x30000)
#define PCIE_CLIENT_TEST_LTSSM_STATUS GENMASK(5, 0)
#define PCIE_CLIENT_TEST_LTSSM_STATUS_L0 (0x11)
#define PCIE_CLIENT_TEST_LTSSM_STATUS_L0S (0x12)

#define WAKE_DELAY_US 2000 /* 2 ms */

#define dw_ep_to_rk3588_ep(x) dev_get_drvdata((x)->dev)

/**
 * struct rk3588_pcie_ep - private data for PCIe endpoint controller driver
 * @pci: DesignWare PCI controller
 * @apb_base: Base address of the PCIe controller
 * @rst: Internal reset signals
 * @vpcie3v3: Optional regulator that need to be enabled for the link to be established
 * @wake_gpio: WAKE pin, active low
 * @reset_gpio: PERST pin, active low
 * @phy: The PHY associated with the controller
 * @clks: Clocks required by the controller
 * @clk_cnt: The number of clocks registered
 * @perst_irq: Reset IRQ
 * @is_link_enabled: True if link is enabled, false otherewise
 */
struct rk3588_pcie_ep
{
    struct dw_pcie pci;
    void __iomem *apb_base;
    struct reset_control *rst;
    struct regulator *vpcie3v3;
    struct gpio_desc *wake_gpio;
    struct gpio_desc *reset_gpio;
    struct phy *phy;
    struct clk_bulk_data *clks;
    int clk_cnt;
    int perst_irq;
    int sys_irq;
    bool is_link_enabled;
};

static int rk3588_pcie_ep_readl_apb(struct rk3588_pcie_ep *ep,
                                    u32 reg)
{
    return readl_relaxed(ep->apb_base + reg);
}

static void rk3588_pcie_ep_writel_apb(struct rk3588_pcie_ep *ep,
                                      u32 val, u32 reg)
{
    writel_relaxed(val, ep->apb_base + reg);
}

static void rk3588_pcie_ep_enable_ltssm(struct rk3588_pcie_ep *ep)
{
    rk3588_pcie_ep_writel_apb(ep, PCIE_CLIENT_ENABLE_LTSSM,
                              PCIE_CLIENT_GENERAL_CONTROL);
}

static void rk3588_pcie_ep_disable_ltssm(struct rk3588_pcie_ep *ep)
{
    rk3588_pcie_ep_writel_apb(ep, PCIE_CLIENT_DISABLE_LTSSM,
                              PCIE_CLIENT_GENERAL_CONTROL);
}

static int rk3588_pcie_ep_resource_get(struct platform_device *pdev)
{
    struct rk3588_pcie_ep *ep = platform_get_drvdata(pdev);
    struct device *dev = &pdev->dev;

    ep->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
    if (IS_ERR(ep->apb_base))
        return PTR_ERR(ep->apb_base);

    ep->wake_gpio = devm_gpiod_get_optional(dev, "wake", GPIOD_OUT_LOW);
    if (IS_ERR(ep->wake_gpio))
        return PTR_ERR(ep->wake_gpio);

    ep->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(ep->reset_gpio))
        return PTR_ERR(ep->reset_gpio);

    ep->rst = devm_reset_control_array_get_exclusive(dev);
    if (IS_ERR(ep->rst))
        return dev_err_probe(dev, PTR_ERR(ep->rst),
                             "failed to get reset lines\n");

    ep->vpcie3v3 = devm_regulator_get_optional(dev, "regulators");
    if (IS_ERR(ep->vpcie3v3))
    {
        if (PTR_ERR(ep->vpcie3v3) != -ENODEV)
            return dev_err_probe(dev, PTR_ERR(ep->vpcie3v3),
                                 "failed to get vpcie3v3 regulator\n");
        ep->vpcie3v3 = NULL;
    }

    ep->phy = devm_phy_get(dev, "pcie-phy");
    if (IS_ERR(ep->phy))
        return dev_err_probe(dev, PTR_ERR(ep->phy), "missing PHY\n");

    ep->clk_cnt = devm_clk_bulk_get_all(dev, &ep->clks);
    if (ep->clk_cnt < 0)
        return dev_err_probe(dev, ep->clk_cnt, "failed to get clocks\n");

    return 0;
}

static int rk3588_pcie_ep_enable_resources(struct rk3588_pcie_ep *ep)
{
    struct device *dev = ep->pci.dev;
    int ret;

    // Assert the reset signals
    ret = reset_control_assert(ep->rst);
    if (ret)
    {
        dev_err(dev, "failed to assert reset signals\n");
        return ret;
    }

    // Enable the regulator, if required
    if (ep->vpcie3v3 != NULL)
    {
        ret = regulator_enable(ep->vpcie3v3);
        if (ret)
        {
            dev_err(dev, "failed to enable vpcie3v3 regulator\n");
            return ret;
        }
    }

    // Enable phy
    ret = phy_init(ep->phy);
    if (ret < 0)
    {
        dev_err(dev, "failed to initialized the PHY\n");
        goto disable_regulator;
    }

    ret = phy_power_on(ep->phy);
    if (ret)
    {
        dev_err(dev, "failed to power on the PHY\n");
        phy_exit(ep->phy);
        goto disable_phy;
    }

    // De-assert reset lines
    ret = reset_control_deassert(ep->rst);
    if (ret)
    {
        dev_err(dev, "failed to power on the PHY\n");
        goto power_off_phy;
    }

    // Enable clocks
    ret = clk_bulk_prepare_enable(ep->clk_cnt, ep->clks);
    if (ret)
    {
        dev_err(dev, "failed to prepare all clocks\n");
        goto power_off_phy;
    }

    // Call dw init
    ret = dw_pcie_ep_init(&ep->pci.ep);
    if (ret)
    {
        dev_err(dev, "failed to perform endpoint initialization\n");
        goto disable_clocks;
    }

    // Success
    return 0;

    // Error handling
disable_clocks:
    clk_bulk_disable_unprepare(ep->clk_cnt, ep->clks);
power_off_phy:
    phy_power_off(ep->phy);
disable_phy:
    phy_exit(ep->phy);
disable_regulator:
    if (ep->vpcie3v3 != NULL)
        regulator_disable(ep->vpcie3v3);

    return ret;
}

static int rk3588_pcie_ep_disable_resources(struct rk3588_pcie_ep *ep)
{
    struct dw_pcie *pci = &ep->pci;
    int ret;

    dw_pcie_ep_exit(&pci->ep);
    clk_bulk_disable_unprepare(ep->clk_cnt, ep->clks);

    ret = phy_power_off(ep->phy);
    if (ret)
        return ret;

    ret = phy_exit(ep->phy);
    if (ret)
        return ret;

    if (ep->vpcie3v3 != NULL)
    {
        ret = regulator_disable(ep->vpcie3v3);
        if (ret)
            return ret;
    }

    return 0;
}

static irqreturn_t rk3588_pcie_ep_sys_irq_thread(int irq, void *data)
{
    struct rk3588_pcie_ep *ep = data;
    struct dw_pcie *pci = &ep->pci;
    struct device *dev = pci->dev;
    bool matched_event;

    u32 val = rk3588_pcie_ep_readl_apb(ep, PCIE_CLIENT_INTR_MASK_MISC);

    if (FIELD_GET(PCIE_CLIENT_TEST_INTR_MASK_MISC_DLL_UP, val))
    {
        dev_dbg(dev, "Received Linkup event. Enumeration complete!\n");
        dw_pcie_ep_linkup(&pci->ep);
        matched_event = true;
    }

    if (FIELD_GET(PCIE_CLIENT_TEST_INTR_MASK_MISC_REQ_RST, val))
    {
        dev_dbg(dev, "Received Linkdown event\n");
        pci_epc_linkdown(pci->ep.epc);
        matched_event = true;
    }

    if (!matched_event)
        dev_err(dev, "Received unknown event: %d\n", val);

    return IRQ_HANDLED;
}

// Transition from D3 to D0 state
static int rk3588_pcie_ep_enable_link(struct dw_pcie *pci)
{
    struct device *dev = pci->dev;
    struct rk3588_pcie_ep *ep = dev_get_drvdata(dev);
    int ret;

    /* Assert WAKE# to RC to indicate device wants power/clocks restored */
    if (ep->wake_gpio != NULL)
    {
        // Set the signal to high impedance state to ensure that
        // falling edge triggers are hit when the line is asserted
        gpiod_set_value_cansleep(ep->wake_gpio, 1);
        // Sleep to ensure any transitions caused by the possible
        // previous value change are processed
        usleep_range(WAKE_DELAY_US, WAKE_DELAY_US + 500);
        // Assert the signal
        gpiod_set_value_cansleep(ep->wake_gpio, 0);
    }

    ret = rk3588_pcie_ep_enable_resources(ep);
    if (ret)
    {
        dev_err(dev, "Failed to enable resources: %d\n", ret);
        return ret;
    }

    // Configure the controller and enable link training
    rk3588_pcie_ep_writel_apb(ep, PCIE_CLIENT_ENABLE_LTSSM_ENHANCE,
                              PCIE_CLIENT_HOT_RESET_CONTROL);
    rk3588_pcie_ep_writel_apb(ep, PCIE_CLIENT_SET_EP_MODE,
                              PCIE_CLIENT_GENERAL_CONTROL);
    dw_pcie_writel_dbi(pci, USP_PCIE_TYPE0,
                       USP_PCIE_TYPE0_ENABLE_BUS_MASTER | USP_PCIE_TYPE0_MEMORY_SPACE);

    ret = dw_pcie_ep_init_complete(&pci->ep);
    if (ret)
    {
        dev_err(dev, "Failed to complete init: %d\n", ret);
        rk3588_pcie_ep_disable_resources(ep);
        return ret;
    }

    // Notify the EPF device that initialization is complete
    dw_pcie_ep_init_notify(&pci->ep);

    // Notify EPF device that bus manager capability is enabled
    pci_epc_bme_notify(pci->ep.epc);

    // Enable the LTSSM, telling the hardware to establish a PCIe link
    rk3588_pcie_ep_enable_ltssm(ep);

    ep->is_link_enabled = true;
    return 0;
}

// Transition to D3 state
static void rk3588_pcie_ep_disable_link(struct dw_pcie *pci)
{
    struct device *dev = pci->dev;
    struct rk3588_pcie_ep *ep = dev_get_drvdata(dev);

    if (!ep->is_link_enabled)
    {
        dev_dbg(dev, "Link is already disabled\n");
        return;
    }

    rk3588_pcie_ep_disable_ltssm(ep);
    gpiod_set_value_cansleep(ep->wake_gpio, 1);
    rk3588_pcie_ep_disable_resources(ep);
    ep->is_link_enabled = false;
}

static irqreturn_t rk3588_pcie_ep_perst_irq_thread(int irq, void *data)
{
    struct rk3588_pcie_ep *ep = data;
    struct dw_pcie *pci = &ep->pci;
    struct device *dev = pci->dev;
    u32 perst;

    perst = gpiod_get_value(ep->reset_gpio);
    if (perst)
    {
        dev_dbg(dev, "PERST asserted by host. Shutting down the PCIe link!\n");
        rk3588_pcie_ep_disable_link(pci);
    }
    else
    {
        dev_dbg(dev, "PERST de-asserted by host. Starting link training!\n");
        rk3588_pcie_ep_enable_link(pci);
    }

    irq_set_irq_type(gpiod_to_irq(ep->reset_gpio),
                     (perst ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW));

    return IRQ_HANDLED;
}

static int rk3588_pcie_ep_enable_irqs(struct platform_device *pdev)
{
    struct rk3588_pcie_ep *ep = platform_get_drvdata(pdev);
    struct device *dev = &pdev->dev;
    int ret;

    // Enable system IRQ resources
    ep->sys_irq = platform_get_irq_byname(pdev, "sys");
    if (ep->sys_irq < 0)
    {
        dev_err(dev, "Failed to get sys IRQ: %d\n", ep->sys_irq);
        return ep->sys_irq;
    }

    ret = devm_request_threaded_irq(&pdev->dev, ep->sys_irq, NULL,
                                    rk3588_pcie_ep_sys_irq_thread,
                                    IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                                    "sys_irq", ep);
    if (ret)
    {
        dev_err(&pdev->dev, "Failed to request Global IRQ\n");
        return ret;
    }

    // Unmask link up and reset request IRQs
    rk3588_pcie_ep_writel_apb(ep,
                              PCIE_CLIENT_INTR_MASK_MISC_ENABLE_DLL_UP | PCIE_CLIENT_INTR_MASK_MISC_ENABLE_REQ_RST,
                              PCIE_CLIENT_INTR_MASK_MISC);

    // Enable reset IRQ resources
    if (ep->reset_gpio)
    {
        ep->perst_irq = gpiod_to_irq(ep->reset_gpio);
        if (ep->perst_irq < 0)
        {
            dev_err(dev, "Failed to get IRQ for PERST GPIO: %d\n", ep->perst_irq);
            return ep->perst_irq;
        }

        irq_set_status_flags(ep->perst_irq, IRQ_NOAUTOEN);
        ret = devm_request_threaded_irq(&pdev->dev, ep->perst_irq, NULL,
                                        rk3588_pcie_ep_perst_irq_thread,
                                        IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                                        "perst_irq", ep);
        if (ret)
        {
            dev_err(&pdev->dev, "Failed to request PERST IRQ\n");
            goto disable_sys_irq;
        }
    }

    return 0;

disable_sys_irq:
    disable_irq(ep->sys_irq);
    return ret;
}

static void rk3588_pcie_ep_disable_irqs(struct platform_device *pdev)
{
    struct rk3588_pcie_ep *ep = platform_get_drvdata(pdev);

    disable_irq(ep->sys_irq);
    if (ep->reset_gpio)
        disable_irq(ep->perst_irq);
}

static enum dw_pcie_ltssm rk3588_pcie_ep_get_ltssm_state(struct dw_pcie *pci)
{
    struct rk3588_pcie_ep *ep = dw_ep_to_rk3588_ep(pci);
    u32 val = rk3588_pcie_ep_readl_apb(ep, PCIE_CLIENT_LTSSM_STATUS);

    return (enum dw_pcie_ltssm)FIELD_GET(PORT_LOGIC_LTSSM_STATE_MASK, val);
}

static int rk3588_pcie_ep_link_up(struct dw_pcie *pci)
{
    return rk3588_pcie_ep_get_ltssm_state(pci) == DW_PCIE_LTSSM_L0;
}

static int rk3588_pcie_ep_start_link(struct dw_pcie *pci)
{
    struct rk3588_pcie_ep *ep = dw_ep_to_rk3588_ep(pci);
    enable_irq(ep->perst_irq);
    return 0;
}

static void rk3588_pcie_ep_stop_link(struct dw_pcie *pci)
{
    struct rk3588_pcie_ep *ep = dw_ep_to_rk3588_ep(pci);

    if (ep->perst_irq)
        disable_irq(ep->perst_irq);
    rk3588_pcie_ep_disable_ltssm(ep);
}

/* Common DWC controller ops */
static const struct dw_pcie_ops pci_ops = {
    .link_up = rk3588_pcie_ep_link_up,
    .start_link = rk3588_pcie_ep_start_link,
    .stop_link = rk3588_pcie_ep_stop_link,
    .get_ltssm = rk3588_pcie_ep_get_ltssm_state,
};

static void rk3588_pcie_ep_init(struct dw_pcie_ep *ep)
{
    struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
    enum pci_barno bar;

    for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
        dw_pcie_ep_reset_bar(pci, bar);
}

static int rk3588_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
                                    unsigned int type, u16 interrupt_num)
{
    struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

    switch (type)
    {
    case PCI_IRQ_INTX:
        return dw_pcie_ep_raise_intx_irq(ep, func_no);
    case PCI_IRQ_MSI:
        return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
    case PCI_IRQ_MSIX:
        return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
    default:
        dev_err(pci->dev, "Unknown IRQ type\n");
        return -EINVAL;
    }
}

static const struct pci_epc_features rk3588_pcie_epc_features = {
    .linkup_notifier = true,
    .core_init_notifier = true,
    .msi_capable = true,
    .msix_capable = true,
    .align = SZ_4K, // Not actually sure if this is required, but most dw controllers have some value here.
};

static const struct pci_epc_features *
rk3588_pcie_ep_get_features(struct dw_pcie_ep *pci_ep)
{
    return &rk3588_pcie_epc_features;
}

static const struct dw_pcie_ep_ops pci_ep_ops = {
    .init = rk3588_pcie_ep_init,
    .raise_irq = rk3588_pcie_ep_raise_irq,
    .get_features = rk3588_pcie_ep_get_features,
};

static int rk3588_pcie_ep_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct rk3588_pcie_ep *ep;
    int ret;

    // Setup state/ref tracking structs
    ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
    if (!ep)
    {
        dev_err(dev, "Failed to allocate memory for endpoint struct\n");
        return -ENOMEM;
    }

    // Setup the dw struct
    ep->pci.dev = dev;
    ep->pci.ops = &pci_ops;
    ep->pci.ep.ops = &pci_ep_ops;
    ep->pci.edma.nr_irqs = 4; // These are IRQs 301 302 303 304
    platform_set_drvdata(pdev, ep);

    // Get resources
    ret = rk3588_pcie_ep_resource_get(pdev);
    if (ret)
    {
        dev_err(dev, "Failed to get all resources: %d\n", ret);
        return ret;
    }

    ret = rk3588_pcie_ep_enable_irqs(pdev);
    if (ret)
    {
        dev_err(dev, "Failed to enable all IRQs: %d\n", ret);
        return ret;
    }

    if (!ep->perst_irq)
    {
        ret = rk3588_pcie_ep_enable_link(&ep->pci);
        if (ret)
        {
            dev_err(dev, "Failed to enable link: %d\n", ret);
            rk3588_pcie_ep_disable_irqs(pdev);
            return ret;
        }
    }

    return 0;
}

static void rk3588_pcie_ep_remove_new(struct platform_device *pdev)
{
    struct rk3588_pcie_ep *ep = platform_get_drvdata(pdev);
    struct dw_pcie *pci = &ep->pci;

    rk3588_pcie_ep_disable_irqs(pdev);
    rk3588_pcie_ep_disable_link(pci);
}

static const struct of_device_id rk3588_pcie_ep_of_match[] = {
    {.compatible = "rockchip,rk3588-pcie-ep"},
    {},
};

static struct platform_driver rk3588_pcie_ep_driver = {
    .driver = {
        .name = "rk3588-pcie-ep",
        .of_match_table = rk3588_pcie_ep_of_match,
    },
    .probe = rk3588_pcie_ep_probe,
    .remove_new = rk3588_pcie_ep_remove_new,
};

static int __init rk3588_pcie_ep_mod_init(void)
{
    return platform_driver_probe(&rk3588_pcie_ep_driver, rk3588_pcie_ep_probe);
}

static void __exit rk3588_pcie_ep_mod_exit(void)
{
    platform_driver_unregister(&rk3588_pcie_ep_driver);
}

module_init(rk3588_pcie_ep_mod_init);
module_exit(rk3588_pcie_ep_mod_exit);
