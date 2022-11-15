// SPDX-License-Identifier: GPL-2.0

/* Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
 * Copyright (C) 2018-2022 Linaro Ltd.
 */

/* DOC: IPA Interrupts
 *
 * The IPA has an interrupt line distinct from the interrupt used by the GSI
 * code.  Whereas GSI interrupts are generally related to channel events (like
 * transfer completions), IPA interrupts are related to other events related
 * to the IPA.  Some of the IPA interrupts come from a microcontroller
 * embedded in the IPA.  Each IPA interrupt type can be both masked and
 * acknowledged independent of the others.
 *
 * Two of the IPA interrupts are initiated by the microcontroller.  A third
 * can be generated to signal the need for a wakeup/resume when an IPA
 * endpoint has been suspended.  There are other IPA events, but at this
 * time only these three are supported.
 */

#include <linux/bitmap.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>

#include "ipa.h"
#include "ipa_reg.h"
#include "ipa_endpoint.h"
#include "ipa_interrupt.h"
#include "ipa_power.h"
#include "ipa_uc.h"

/*
 * This is the list of interrupts we care about.
 * FIXME: can we get rid of this?
 * define it better somehow if not
 */
#define ENABLED_MASK (BIT(IPA_IRQ_TX_SUSPEND) | BIT(IPA_IRQ_UC_0) | BIT(IPA_IRQ_UC_1))

static void __ipa_clear_irq(struct ipa *ipa, u32 mask)
{
	const struct ipa_reg *reg;
	u32 offset;

	reg = ipa_reg(ipa, IPA_IRQ_CLR);
	offset = ipa_reg_offset(reg);
	dev_info(&ipa->pdev->dev, "%s 0x%08x\n", __func__, mask);
	iowrite32(mask, ipa->reg_virt + offset);
}

void ipa_interrupt_clear_irq(struct ipa *ipa, enum ipa_irq_id irq_id)
{
	__ipa_clear_irq(ipa, BIT(irq_id));
}

/* Get the status of enabled IRQs */
static u32 ipa_irq_status(struct ipa *ipa)
{
	const struct ipa_reg *reg = ipa_reg(ipa, IPA_IRQ_STTS);
	u32 offset = ipa_reg_offset(reg);

	return ioread32(ipa->reg_virt + offset);
}

/* Process the IRQs in the enabled mask and return all
 * other un-cleared IRQs
 */
static u32 process_irqs(struct ipa *ipa, u32 enabled)
{
	const struct ipa_reg *reg;
	u32 offset;
	u32 pending;
	u32 mask;

	/*
	 * The IRQ handlers are responsible for clearing their own IRQs
	 * by calling ipa_interrupt_clear_irq().
	 * 
	 */
	reg = ipa_reg(ipa, IPA_IRQ_STTS);
	offset = ipa_reg_offset(reg);
	mask = pending = ioread32(ipa->reg_virt + offset);
	while ((mask = pending & enabled)) {
		do {
			u32 irq_id = __ffs(mask);
			dev_info(&ipa->pdev->dev, "Handling IRQ: %d\n", irq_id);

			mask ^= BIT(irq_id);

			switch(irq_id) {
			case IPA_IRQ_TX_SUSPEND:
				ipa_suspend_isr_handler(ipa);
				break;
			case IPA_IRQ_UC_0:
				ipa_uc_event_handler(ipa, irq_id);
				break;
			case IPA_IRQ_UC_1:
				ipa_uc_response_hdlr(ipa, irq_id);
				break;
			default:
				dev_info(&ipa->pdev->dev, "Unused interrupt fired: 0x%08x\n",
					irq_id);
				break;
			}
		} while (mask);
		pending = ioread32(ipa->reg_virt + offset);
	}

	return pending;
}

static irqreturn_t ipa_isr_thread(int irq, void *data)
{
	struct ipa *ipa = data;
	struct device *dev = &ipa->pdev->dev;;
	u32 enabled = ENABLED_MASK;
	u32 pending;
	int ret;

	dev_info(dev, "Soft handler");

	ret = pm_runtime_get_sync(dev);
	if (WARN_ON(ret < 0))
		goto out_power_put;

	pending = process_irqs(ipa, enabled);

	/* Clear any remaining interrupts */
	if (pending)
		__ipa_clear_irq(ipa, pending);

out_power_put:
	pm_runtime_mark_last_busy(dev);
	(void)pm_runtime_put_autosuspend(dev);

	return IRQ_HANDLED;
}

/*
 If we suspend whilst handling RX data, the IPA irq will fire and
 cause the endpoints to be resumed. Can this happen whilst for example
 receiving large amounts of data? Would this add latency
*/

/*
 * This hard IRQ handler will cause a system wakeup if currently suspended.
 * Everything else is dealt with by the threaded handler above
 */
static irqreturn_t ipa_isr(int irq, void *dev_id)
{
	struct ipa *ipa = dev_id;
	struct device *dev = &ipa->pdev->dev;
	int usage = 0;
	u32 status;
	bool suspend;
	irqreturn_t ret = IRQ_WAKE_THREAD;

	/*
	 * We can't turn the clock on in atomic context but we can keep
	 * it on. If it's off then we need to run the threaded handler
	 * to boot up the core clock and handle everything there
	 */
	usage = pm_runtime_get_if_active(dev, true);
	if (usage <= 0)
		return IRQ_WAKE_THREAD;

	/* Only process the TX_SUSPEND interrupt in this context */
	status = process_irqs(ipa, BIT(IPA_IRQ_TX_SUSPEND));
	dev_info(dev, "%s: status: 0x%08x, suspend: %d, usage: %d\n", __func__, status, suspend, usage);

	// TODO: need to do the nested while loop stuff to clear
	// IRQs in a loop and re-read them again to make sure they clear
	// bluergh
	// Reading the IRQ_EN register seems to be the trick downstream use?
	// maybe it causes the IRQ_STTS register to update properly or something 

	/* Clear any disable interrupts */
	__ipa_clear_irq(ipa, (status & ~ENABLED_MASK));

	/* If no enabled IRQs are still pending then
	 * we're done. Otherwise we spawn the threaded
	 * handler
	 */
	if (!(status & ENABLED_MASK))
		ret = IRQ_HANDLED;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

/* Common function used to enable/disable TX_SUSPEND for an endpoint */
static void ipa_interrupt_suspend_control(struct ipa *ipa,
					  u32 endpoint_id, bool enable)
{
	u32 mask = BIT(endpoint_id);
	const struct ipa_reg *reg;
	u32 offset;
	u32 val;

	WARN_ON(!(mask & ipa->available));

	/* IPA version 3.0 does not support TX_SUSPEND interrupt control */
	if (ipa->version == IPA_VERSION_3_0)
		return;

	reg = ipa_reg(ipa, IRQ_SUSPEND_EN);
	offset = ipa_reg_offset(reg);
	val = ioread32(ipa->reg_virt + offset);
	if (enable)
		val |= mask;
	else
		val &= ~mask;
	iowrite32(val, ipa->reg_virt + offset);
}

/* Enable TX_SUSPEND for an endpoint */
void
ipa_interrupt_suspend_enable(struct ipa *ipa, u32 endpoint_id)
{
	ipa_interrupt_suspend_control(ipa, endpoint_id, true);
}

/* Disable TX_SUSPEND for an endpoint */
void
ipa_interrupt_suspend_disable(struct ipa *ipa, u32 endpoint_id)
{
	ipa_interrupt_suspend_control(ipa, endpoint_id, false);
}

/* Clear the suspend interrupt for all endpoints that signaled it */
void ipa_interrupt_suspend_clear_all(struct ipa *ipa)
{
	const struct ipa_reg *reg;
	u32 val;

	reg = ipa_reg(ipa, IRQ_SUSPEND_INFO);
	val = ioread32(ipa->reg_virt + ipa_reg_offset(reg));

	/* SUSPEND interrupt status isn't cleared on IPA version 3.0 */
	if (ipa->version == IPA_VERSION_3_0)
		return;

	reg = ipa_reg(ipa, IRQ_SUSPEND_CLR);
	iowrite32(val, ipa->reg_virt + ipa_reg_offset(reg));
}

/* Simulate arrival of an IPA TX_SUSPEND interrupt */
void ipa_interrupt_simulate_suspend(struct ipa *ipa)
{
	ipa_suspend_isr_handler(ipa);
}

/* Configure the IPA interrupt framework */
int ipa_interrupt_config(struct ipa *ipa)
{
	struct device *dev = &ipa->pdev->dev;
	const struct ipa_reg *reg;
	unsigned int irq;
	int ret;

	ret = platform_get_irq_byname(ipa->pdev, "ipa");
	if (ret <= 0) {
		dev_err(dev, "DT error %d getting \"ipa\" IRQ property\n",
			ret);
		return ret ? : -EINVAL;
	}
	irq = ret;

	/* Start with all IPA interrupts enabled */
	reg = ipa_reg(ipa, IPA_IRQ_EN);
	iowrite32(~0, ipa->reg_virt + ipa_reg_offset(reg));

	ret = request_threaded_irq(irq, ipa_isr, ipa_isr_thread, IRQF_ONESHOT, "ipa", ipa);
	if (ret) {
		dev_err(dev, "error %d requesting \"ipa\" IRQ\n", ret);
		return ret;
	}

	ret = dev_pm_set_wake_irq(dev, irq);
	if (ret) {
		dev_err(dev, "error %d enabling wakeup for \"ipa\" IRQ\n", ret);
		return ret;
	}

	ipa->irq = irq;

	return 0;
}

/* Inverse of ipa_interrupt_config() */
void ipa_interrupt_deconfig(struct ipa *ipa)
{
	struct device *dev = &ipa->pdev->dev;
	int ret;

	dev_pm_clear_wake_irq(dev);
	ret = disable_irq_wake(ipa->irq);
	if (ret)
		dev_err(dev, "error %d disabling \"ipa\" IRQ wakeup\n", ret);
}
