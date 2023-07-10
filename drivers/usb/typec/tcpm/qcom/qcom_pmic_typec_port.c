// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>
#include "qcom_pmic_typec_port.h"

struct pmic_typec_port_irq_data {
	int				virq;
	int				irq;
	struct pmic_typec_port		*pmic_typec_port;
};

struct pmic_typec_port {
	struct device			*dev;
	struct tcpm_port		*tcpm_port;
	struct regmap			*regmap;
	struct pmic_typec_regmap_fields	fields;
	u32				base;
	unsigned int			nr_irqs;
	struct pmic_typec_port_irq_data	*irq_data;

	struct regulator		*vdd_vbus;

	int				cc;
	bool				debouncing_cc;
	struct delayed_work		cc_debounce_dwork;

	spinlock_t			lock;	/* Register atomicity */
};

static const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]		= "Open",
	[TYPEC_CC_RA]		= "Ra",
	[TYPEC_CC_RD]		= "Rd",
	[TYPEC_CC_RP_DEF]	= "Rp-def",
	[TYPEC_CC_RP_1_5]	= "Rp-1.5",
	[TYPEC_CC_RP_3_0]	= "Rp-3.0",
};

static const char *rp_unknown = "unknown";

static const char *cc_to_name(enum typec_cc_status cc)
{
	if (cc > TYPEC_CC_RP_3_0)
		return rp_unknown;

	return typec_cc_status_name[cc];
}

static const char * const cc_curr_src_name[] = {
	[CC_SRC_RP_SEL_80UA]		= "Rp-def-80uA",
	[CC_SRC_RP_SEL_180UA]	= "Rp-1.5-180uA",
	[CC_SRC_RP_SEL_330UA]	= "Rp-3.0-330uA",
};

static const char *cc_curr_src_to_name(int rp_sel)
{
	if (rp_sel > CC_SRC_RP_SEL_330UA)
		return rp_unknown;

	return cc_curr_src_name[rp_sel];
}

#define orientation_to_cc(o) (o) ? "cc1" : "cc2"
#define orientation_to_vconn(o) orientation_to_cc(!(o))

static void qcom_pmic_typec_port_cc_debounce(struct work_struct *work)
{
	struct pmic_typec_port *pmic_typec_port =
		container_of(work, struct pmic_typec_port, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);
	pmic_typec_port->debouncing_cc = false;
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(pmic_typec_port->dev, "Debounce cc complete\n");
}

static irqreturn_t pmic_typec_port_isr(int irq, void *dev_id)
{
	struct pmic_typec_port_irq_data *irq_data = dev_id;
	struct pmic_typec_port *pmic_typec_port = irq_data->pmic_typec_port;
	bool vbus_change = false;
	bool cc_change = false;
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	switch (irq_data->virq) {
	case PMIC_TYPEC_VBUS_IRQ:
		vbus_change = true;
		break;
	case PMIC_TYPEC_CC_STATE_IRQ:
	case PMIC_TYPEC_ATTACH_DETACH_IRQ:
		if (!pmic_typec_port->debouncing_cc)
			cc_change = true;
		break;
	}

	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	if (vbus_change)
		tcpm_vbus_change(pmic_typec_port->tcpm_port);

	if (cc_change)
		tcpm_cc_change(pmic_typec_port->tcpm_port);

	return IRQ_HANDLED;
}

bool qcom_pmic_typec_port_is_vbus_vsafe0v(struct pmic_typec_port *pmic_typec_port)
{
	int ret, vsafe0v;

	/*
	 * The TCPM state machine defaults to true if the field is not present.
	 * Do that here, too.
	 */
	if (!pmic_typec_port->fields.has_vbus_vsafe0v)
		return true;

	ret = regmap_field_read(pmic_typec_port->fields.vbus_vsafe, &vsafe0v);

	return !ret && (vsafe0v & VBUS_STATUS_VSAFE0V);
}

int qcom_pmic_typec_port_get_vbus(struct pmic_typec_port *pmic_typec_port)
{
	struct device *dev = pmic_typec_port->dev;
	unsigned int vbus_detect;
	int ret;

	ret = regmap_field_read(pmic_typec_port->fields.vbus_detect, &vbus_detect);
	if (ret)
		vbus_detect = 0;

	dev_dbg(dev, "get_vbus: detect %d\n", vbus_detect);

	return vbus_detect;
}

int qcom_pmic_typec_port_set_vbus(struct pmic_typec_port *pmic_typec_port, bool on)
{
	u32 sm_stat;
	u32 val;
	int ret;

	if (on) {
		ret = regulator_enable(pmic_typec_port->vdd_vbus);
		if (ret)
			return ret;

		val = VBUS_STATUS_VSAFE5V;
	} else {
		ret = regulator_disable(pmic_typec_port->vdd_vbus);
		if (ret)
			return ret;

		val = VBUS_STATUS_VSAFE0V;
	}

	// FIXME: With vbus regulator patches it should be possible to remove
	if (pmic_typec_port->fields.has_vbus_vsafe0v) {
		/* Poll waiting for transition to required vSafe5V or vSafe0V */
		ret = regmap_field_read_poll_timeout(pmic_typec_port->fields.vbus_vsafe,
						     sm_stat, sm_stat == val,
						     100, 250000);

		if (ret)
			dev_warn(pmic_typec_port->dev, "vbus vsafe%dv fail\n",
				 on ? 5 : 0);
	}

	return 0;
}

bool qcom_pmic_typec_port_is_legacy_cable(struct pmic_typec_port *pmic_typec_port)
{
	int ret;
	unsigned int legacy_cable, snk_src_mode;

	ret = regmap_field_read_log(pmic_typec_port->fields.snk_src_mode, &snk_src_mode);
	if (ret) {
		dev_err(pmic_typec_port->dev, "Failed to read snk_src_mode: %d\n", ret);
		return false;
	}

	ret = regmap_field_read(pmic_typec_port->fields.is_legacy_cable, &legacy_cable);
	if (ret) {
		dev_err(pmic_typec_port->dev, "Failed to read legacy cable: %d\n", ret);
		return false;
	}

	if (legacy_cable & LEGACY_CABLE_NONCOMPLIANT)
		dev_dbg(pmic_typec_port->dev, "non-compliant legacy_cable\n");

	dev_info(pmic_typec_port->dev, "legacy_cable: %d, snk_src_mode: %d\n",
		 legacy_cable, snk_src_mode);

	/* Legacy cable detected and in SNK mode (UFP) */
	return !!legacy_cable && !snk_src_mode;
}

int qcom_pmic_typec_port_get_cc(struct pmic_typec_port *pmic_typec_port,
				enum typec_cc_status *cc1,
				enum typec_cc_status *cc2)
{
	struct device *dev = pmic_typec_port->dev;
	unsigned int is_src_mode, val;
	bool attached, orientation;
	int ret = 0;
	unsigned long flags;

	// FIXME: Ask Bryan if this is needed
	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_field_read(pmic_typec_port->fields.cc_status, &val);
	if (ret)
		goto done;

	orientation = !!(val & CC_ORIENTATION);
	attached = !!(val & CC_ATTACHED);
	
	ret = regmap_field_read(pmic_typec_port->fields.snk_src_mode, &is_src_mode);


	if (pmic_typec_port->debouncing_cc) {
		ret = -EBUSY;
		goto done;
	}

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	if (!attached)
		goto done;

	if (is_src_mode) {
		ret = regmap_field_read(pmic_typec_port->fields.src_status, &val);
		if (ret)
			goto done;
		switch (val) {
		case SRC_RD_OPEN:
			val = TYPEC_CC_RD;
			break;
		case SRC_RD_RA_VCONN:
			val = TYPEC_CC_RD;
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			break;
		default:
			dev_warn(dev, "unexpected src status %.2x\n", val);
			val = TYPEC_CC_RD;
			break;
		}
	} else {
		ret = regmap_field_read(pmic_typec_port->fields.snk_status, &val);
		if (ret)
			goto done;
		switch (val) {
		case SNK_RP_STD:
			val = TYPEC_CC_RP_DEF;
			break;
		case SNK_RP_1P5:
			val = TYPEC_CC_RP_1_5;
			break;
		case SNK_RP_3P0:
			val = TYPEC_CC_RP_3_0;
			break;
		default:
			dev_warn(dev, "unexpected snk status %.2x\n", val);
			val = TYPEC_CC_RP_DEF;
			break;
		}
		val = TYPEC_CC_RP_DEF;
	}

	if (orientation)
		*cc2 = val;
	else
		*cc1 = val;

done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "get_cc: cc1 0x%08x %s cc2 0x%08x %s attached %d cc=%s\n",
		*cc1, cc_to_name(*cc1), *cc2, cc_to_name(*cc2), attached,
		orientation_to_cc(orientation));

	return ret;
}

static void qcom_pmic_set_cc_debounce(struct pmic_typec_port *pmic_typec_port)
{
	pmic_typec_port->debouncing_cc = true;
	schedule_delayed_work(&pmic_typec_port->cc_debounce_dwork,
			      msecs_to_jiffies(2));
}

int qcom_pmic_typec_port_set_cc(struct pmic_typec_port *pmic_typec_port,
				enum typec_cc_status cc)
{
	struct device *dev = pmic_typec_port->dev;
	unsigned int mode, currsrc;
	unsigned int misc;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_field_read(pmic_typec_port->fields.misc_dbg, &misc);
	if (ret)
		goto done;

	mode = POWER_ROLE_SRC_ONLY;

	switch (cc) {
	case TYPEC_CC_OPEN:
		currsrc = CC_SRC_RP_SEL_80UA;
		break;
	case TYPEC_CC_RP_DEF:
		currsrc = CC_SRC_RP_SEL_80UA;
		break;
	case TYPEC_CC_RP_1_5:
		currsrc = CC_SRC_RP_SEL_180UA;
		break;
	case TYPEC_CC_RP_3_0:
		currsrc = pmic_typec_port->fields.curr_src_max;
		break;
	case TYPEC_CC_RD:
		currsrc = CC_SRC_RP_SEL_80UA;
		mode = POWER_ROLE_SNK_ONLY;
		break;
	default:
		dev_warn(dev, "unexpected set_cc %d\n", cc);
		ret = -EINVAL;
		goto done;
	}

	if (mode == POWER_ROLE_SRC_ONLY) {
		ret = regmap_field_write(pmic_typec_port->fields.cc_curr_src, currsrc);
		if (ret)
			goto done;
	}

	pmic_typec_port->cc = cc;
	qcom_pmic_set_cc_debounce(pmic_typec_port);
	ret = 0;

done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_cc: currsrc=%x %s mode %s debounce %d attached %d cc=%s\n",
		currsrc, cc_curr_src_to_name(currsrc),
		mode == POWER_ROLE_SRC_ONLY ? "POWER_ROLE_SRC_ONLY" : "POWER_ROLE_SNK_ONLY",
		pmic_typec_port->debouncing_cc, !!(misc & CC_ATTACHED),
		orientation_to_cc(misc));

	return ret;
}

int qcom_pmic_typec_port_set_vconn(struct pmic_typec_port *pmic_typec_port, bool on)
{
	struct device *dev = pmic_typec_port->dev;
	unsigned int orientation, value;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_field_read(pmic_typec_port->fields.cc_status, &value);
	if (ret)
		goto done;

	/* Set VCONN on the inversion of the active CC channel */
	orientation = !(value & CC_ORIENTATION);
	if (on) {
		ret = regmap_field_write(pmic_typec_port->fields.vconn_en_orientation, orientation);
		if (ret)
			goto done;
	}

	ret = regmap_field_write(pmic_typec_port->fields.vconn_en, on);
done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_vconn: orientation %d control 0x%08x state %s cc %s vconn %s\n",
		orientation, value, on ? "on" : "off", orientation_to_vconn(!orientation), orientation_to_cc(!orientation));

	return ret;
}

int qcom_pmic_typec_port_start_toggling(struct pmic_typec_port *pmic_typec_port,
					enum typec_port_type port_type,
					enum typec_cc_status cc)
{
	struct device *dev = pmic_typec_port->dev;
	unsigned int misc;
	u8 mode = 0;
	unsigned long flags;
	int ret;

	/* Get the state machine into the write state if we're booting up with
	 * a legacy cable attached
	 */
	// if (pmic_typec_port->vbus_present && qcom_pmic_typec_port_is_legacy_cable(pmic_typec_port)) {
	// 	tcpm_vbus_change(pmic_typec_port->tcpm_port);
	// 	return 0;
	// }

	switch (port_type) {
	case TYPEC_PORT_SRC:
		mode = POWER_ROLE_SRC_ONLY;
		break;
	case TYPEC_PORT_SNK:
		mode = POWER_ROLE_SNK_ONLY;
		break;
	case TYPEC_PORT_DRP:
		mode = POWER_ROLE_TRY_SINK;
		break;
	}

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_field_read(pmic_typec_port->fields.misc_dbg, &misc);
	if (ret)
		goto done;

	dev_dbg(dev, "start_toggling: misc 0x%08x attached %d port_type %d current cc %d new %d\n",
		misc, !!(misc & CC_ATTACHED), port_type, pmic_typec_port->cc, cc);

	qcom_pmic_set_cc_debounce(pmic_typec_port);

	/* force it to toggle at least once */
	ret = regmap_field_write(pmic_typec_port->fields.power_role, POWER_ROLE_DISABLED);
	if (ret)
		goto done;

	if (mode > POWER_ROLE_TRY_SINK)
		ret = regmap_field_write(pmic_typec_port->fields.power_role, mode);
	else
		ret = regmap_field_write(pmic_typec_port->fields.en_try_snk, 1);

done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	return ret;
}

#define TYPEC_INTR_EN_CFG_1_MASK		  \
	(/*BIT(IRQ_LEGACY_CABLE)		| */ \
	/* BIT(IRQ_NONCOMPLIANT_LEGACY_CABLE)	| */ \
	 BIT(IRQ_TRYSOURCE_DETECT)		| \
	 BIT(IRQ_TRYSINK_DETECT)		| \
	 BIT(IRQ_CCOUT_DETACH)			| \
	 BIT(IRQ_CCOUT_ATTACH)			| \
	 BIT(IRQ_VBUS_DEASSERT)			| \
	 BIT(IRQ_VBUS_ASSERT))

#define TYPEC_INTR_EN_CFG_2_MASK	  \
	(BIT(IRQ_STATE_MACHINE_CHANGE)	| \
	 BIT(IRQ_VBUS_ERROR)		| \
	 BIT(IRQ_DEBOUNCE_DONE))

int qcom_pmic_typec_port_start(struct pmic_typec_port *pmic_typec_port,
			       struct tcpm_port *tcpm_port)
{
	int i;
	int ret;

	/* Configure interrupt sources */
	ret = regmap_field_write(pmic_typec_port->fields.irq_en_cfg1,
				 pmic_typec_port->fields.irq_mask_cfg1);
	if (ret)
		goto done;

	ret = regmap_field_write(pmic_typec_port->fields.irq_en_cfg2,
				 pmic_typec_port->fields.irq_mask_cfg2);
	if (ret)
		goto done;

	/* start in TRY_SNK mode */
	// FIXME: pmi8998 downstream explicitly disables this, maybe enabling it disables legacy cable detection?
	en_try_sink = pmic_typec_port->fields.needs_legacy_cable_en;
	ret = regmap_field_write_log(pmic_typec_port->fields.en_try_snk, en_try_sink);
	if (ret)
		goto done;

	if (pmic_typec_port->fields.needs_legacy_cable_en) {
		ret = regmap_field_write_log(pmic_typec_port->fields.legacy_cable_det, 1);
		if (ret)
			goto done;
	}

	/* Configure VCONN for software control */
	ret = regmap_field_write(pmic_typec_port->fields.vconn_en_src, 1);
	if (ret)
		goto done;
	ret = regmap_field_write(pmic_typec_port->fields.vconn_en, 0);
	if (ret)
		goto done;

	/* Set CC threshold to 1.6 Volts */
	ret = regmap_field_write(pmic_typec_port->fields.cc_src_threshold, 1);
	if (ret)
		goto done;

	/* tPDdebounce = 10-20ms */
	ret = regmap_field_write(pmic_typec_port->fields.cc_src_tpd_debounce, 1);
	if (ret)
		goto done;

	pmic_typec_port->tcpm_port = tcpm_port;

	for (i = 0; i < pmic_typec_port->nr_irqs; i++)
		enable_irq(pmic_typec_port->irq_data[i].irq);

done:
	return ret;
}

void qcom_pmic_typec_port_stop(struct pmic_typec_port *pmic_typec_port)
{
	int i;

	for (i = 0; i < pmic_typec_port->nr_irqs; i++)
		disable_irq(pmic_typec_port->irq_data[i].irq);
}

struct pmic_typec_port *qcom_pmic_typec_port_alloc(struct device *dev)
{
	return devm_kzalloc(dev, sizeof(struct pmic_typec_port), GFP_KERNEL);
}

int qcom_pmic_typec_port_probe(struct platform_device *pdev,
			       struct pmic_typec_port *pmic_typec_port,
			       const struct pmic_typec_port_resources *res,
			       struct regmap *regmap,
			       u32 base)
{
	struct device *dev = &pdev->dev;
	struct pmic_typec_port_irq_data *irq_data;
	int i, ret, irq;

	if (!res->nr_irqs || res->nr_irqs > PMIC_TYPEC_MAX_IRQS)
		return -EINVAL;

	irq_data = devm_kzalloc(dev, sizeof(*irq_data) * res->nr_irqs,
				GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	pmic_typec_port->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(pmic_typec_port->vdd_vbus))
		return PTR_ERR(pmic_typec_port->vdd_vbus);

	// FIXME: lol does this work???
	ret = devm_regmap_field_bulk_alloc(dev, regmap,
					   (struct regmap_field**)&pmic_typec_port->fields,
					   (struct reg_field*)&res->reg_fields->fields,
					   PMIC_TYPEC_NUM_FIELDS);
	if (ret) {
		dev_err_probe(dev, ret, "failed to allocate regmap fields\n");
		return ret;
	}

	pmic_typec_port->fields.has_vbus_vsafe0v = res->reg_fields->has_vbus_vsafe0v;
	pmic_typec_port->fields.needs_legacy_cable_en = res->reg_fields->needs_legacy_cable_en;
	pmic_typec_port->fields.curr_src_max = res->reg_fields->curr_src_max;
	/* Populate IRQ masks */
	for (i = 0; i < IRQ_NUM_IRQS; i++) {
		if (BIT(i) & TYPEC_INTR_EN_CFG_1_MASK)
			pmic_typec_port->fields.irq_mask_cfg1 |= res->reg_fields->irq_map_cfg1[i];
		else if (BIT(i) & TYPEC_INTR_EN_CFG_2_MASK)
			pmic_typec_port->fields.irq_mask_cfg2 |= res->reg_fields->irq_map_cfg2[i];
	}
	dev_info(dev, "irq_mask_cfg1: 0x%x\n", pmic_typec_port->fields.irq_mask_cfg1);
	dev_info(dev, "irq_mask_cfg2: 0x%x\n", pmic_typec_port->fields.irq_mask_cfg2);

	pmic_typec_port->dev = dev;
	pmic_typec_port->base = base;
	pmic_typec_port->regmap = regmap;
	pmic_typec_port->nr_irqs = res->nr_irqs;
	pmic_typec_port->irq_data = irq_data;
	spin_lock_init(&pmic_typec_port->lock);
	INIT_DELAYED_WORK(&pmic_typec_port->cc_debounce_dwork,
			  qcom_pmic_typec_port_cc_debounce);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	for (i = 0; i < res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev,
					      res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->pmic_typec_port = pmic_typec_port;
		irq_data->irq = irq;
		irq_data->virq = res->irq_params[i].virq;
		ret = devm_request_threaded_irq(dev, irq, NULL, pmic_typec_port_isr,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						res->irq_params[i].irq_name,
						irq_data);
		if (ret)
			return ret;
	}

	return 0;
}
