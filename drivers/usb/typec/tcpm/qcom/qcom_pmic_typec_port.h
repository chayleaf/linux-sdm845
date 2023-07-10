/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */
#ifndef __QCOM_PMIC_TYPEC_H__
#define __QCOM_PMIC_TYPEC_H__

#include <linux/platform_device.h>
#include <linux/usb/tcpm.h>
#include <linux/regmap.h>

/* Transposed register bits */

/* cc_curr_src */
#define CC_SRC_RP_SEL_80UA		0
#define CC_SRC_RP_SEL_180UA		1
#define CC_SRC_RP_SEL_330UA		2

/* vbus_status */
#define VBUS_STATUS_VSAFE5V		BIT(0)
#define VBUS_STATUS_VSAFE0V		BIT(1)

/* cc_status */
#define CC_ATTACHED			BIT(0)
#define CC_ORIENTATION			BIT(1)

/* src_status */
#define SRC_RD_OPEN			BIT(3)
#define SRC_RD_RA_VCONN			BIT(2)

/* is_legacy_cable */
#define LEGACY_CABLE			BIT(1)
#define LEGACY_CABLE_NONCOMPLIANT	BIT(0)

/* snk_status */
#define SNK_RP_STD			BIT(2)
#define SNK_RP_1P5			BIT(1)
#define SNK_RP_3P0			BIT(0)

/* power_role */
#define POWER_ROLE_SRC_ONLY		BIT(2)
#define POWER_ROLE_SNK_ONLY		BIT(1)
#define POWER_ROLE_DISABLED		BIT(0)
/* This isn't a real power role, but we pretend */
#define POWER_ROLE_TRY_SINK		0

/* Interrupt numbers */
#define PMIC_TYPEC_OR_RID_IRQ				0x0
#define PMIC_TYPEC_VPD_IRQ				0x1
#define PMIC_TYPEC_CC_STATE_IRQ				0x2
#define PMIC_TYPEC_VCONN_OC_IRQ				0x3
#define PMIC_TYPEC_VBUS_IRQ				0x4
#define PMIC_TYPEC_ATTACH_DETACH_IRQ			0x5
#define PMIC_TYPEC_LEGACY_CABLE_IRQ			0x6
#define PMIC_TYPEC_TRY_SNK_SRC_IRQ			0x7

/* Resources */
#define PMIC_TYPEC_MAX_IRQS				0x08

struct pmic_typec_port_irq_params {
	int				virq;
	char				*irq_name;
};

struct pmic_typec_port_resources {
	unsigned int				nr_irqs;
	struct pmic_typec_port_irq_params	irq_params[PMIC_TYPEC_MAX_IRQS];
	struct pmic_typec_registers		*reg_fields;
};

/*
 * INTR_1_CFG / INTR_2_CFG
 * pmi8998 and pm8150b both use two registers to configure which
 * interrupts are enabled. However pm8150b shuffled the arrangement
 * The same fields enum is used for both, and they're given special
 * handling to get the correct register and bit position.
 */
enum qcom_pmic_typec_intr_cfg_fields {
	IRQ_LEGACY_CABLE = 0,
	IRQ_NONCOMPLIANT_LEGACY_CABLE,
	IRQ_TRYSOURCE_DETECT,
	IRQ_TRYSINK_DETECT,
	IRQ_CCOUT_DETACH,
	IRQ_CCOUT_ATTACH,
	IRQ_VBUS_DEASSERT,
	IRQ_VBUS_ASSERT,
	IRQ_STATE_MACHINE_CHANGE, // These are on INTR_2_CFG on pm8150b
	IRQ_VBUS_ERROR,
	IRQ_DEBOUNCE_DONE,
	IRQ_CC_STATE_CHANGE,

	IRQ_NUM_IRQS,
};

/*
 * These fields represent an abstraction over the type-c registers
 * Where possible they encompass as many bits as possible, but quite
 * a few are just single bits. They're best treated like individual
 * properties, disregarding the register they're in.
 */
#define DEFINE_FIELDS(type) \
	type snk_status; \
	type src_status; \
	type vbus_vsafe; \
	/* MISC_STATUS */ \
	type misc_dbg; \
	type misc_status; \
	type snk_src_mode; \
	type vbus_detect; \
	type vbus_error; \
	type debounce_done; \
	type cc_status; \
	/* MODE_CFG */ \
	type en_try_snk; \
	type en_try_src; \
	type power_role; \
	type is_legacy_cable; \
	/* LEGACY_CABLE (PMI8998 only) */ \
	type legacy_cable_det; \
	/* VCONN_CFG */ \
	type vconn_en_orientation; \
	type vconn_en; \
	type vconn_en_src; \
	/* CURRSRC_CFG */ \
	/* FIXME: USE_TPD_FOR_EXITING_ATTACHSRC */ \
	type cc_src_threshold; \
	type cc_src_tpd_debounce; \
	type cc_curr_src; \
	type cc_src_rp_sel; \
	/* Interrupt configuration */ \
	type irq_en_cfg1; \
	type irq_en_cfg2


/* The (const) register field configuration */
struct pmic_typec_registers {
	struct pmic_typec_reg_fields {
		DEFINE_FIELDS(struct reg_field);
	} fields;

	bool has_vbus_vsafe0v;
	u8 curr_src_max;

	/* IRQ_EN maps */
	u8 irq_map_cfg1[IRQ_NUM_IRQS];
	u8 irq_map_cfg2[IRQ_NUM_IRQS];
};

#define PMIC_TYPEC_NUM_FIELDS (sizeof(struct pmic_typec_reg_fields) / \
			       sizeof(struct reg_field))

/* The runtime allocated regmap_fields */
struct pmic_typec_regmap_fields {
	DEFINE_FIELDS(struct regmap_field *);

	bool has_vbus_vsafe0v;
	/* Some PMICs can't do the full 3A */
	u8 curr_src_max;

	u8 irq_mask_cfg1;
	u8 irq_mask_cfg2;
};

extern struct pmic_typec_registers pmic_typec_fields_pmi8998;
extern struct pmic_typec_registers pmic_typec_fields_pm8150b;

/* API */
struct pmic_typec;

struct pmic_typec_port *qcom_pmic_typec_port_alloc(struct device *dev);

int qcom_pmic_typec_port_probe(struct platform_device *pdev,
			       struct pmic_typec_port *pmic_typec_port,
			       const struct pmic_typec_port_resources *res,
			       struct regmap *regmap,
			       u32 base);

int qcom_pmic_typec_port_start(struct pmic_typec_port *pmic_typec_port,
			       struct tcpm_port *tcpm_port);

void qcom_pmic_typec_port_stop(struct pmic_typec_port *pmic_typec_port);

int qcom_pmic_typec_port_get_cc(struct pmic_typec_port *pmic_typec_port,
				enum typec_cc_status *cc1,
				enum typec_cc_status *cc2);

int qcom_pmic_typec_port_set_cc(struct pmic_typec_port *pmic_typec_port,
				enum typec_cc_status cc);

bool qcom_pmic_typec_port_is_vbus_vsafe0v(struct pmic_typec_port *pmic_typec_port);

int qcom_pmic_typec_port_get_vbus(struct pmic_typec_port *pmic_typec_port);

int qcom_pmic_typec_port_set_vconn(struct pmic_typec_port *pmic_typec_port, bool on);

int qcom_pmic_typec_port_start_toggling(struct pmic_typec_port *pmic_typec_port,
					enum typec_port_type port_type,
					enum typec_cc_status cc);

int qcom_pmic_typec_port_set_vbus(struct pmic_typec_port *pmic_typec_port, bool on);

#endif /* __QCOM_PMIC_TYPE_C_PORT_H__ */
