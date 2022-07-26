// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Michael Srba
 * largely based on pinctrl-lpass-lpi.c
 *
 * This driver gives Linux control over the TLMM block inside the "Sensor Hub"
 * subsystem on (some) qcom SoCs. Please note that most devices are configured
 * such that Linux is not allowed to access this memory, and additionally,
 * most devices are provisioned in a way where the de facto owner is not
 * the person trying to run mainline Linux on their device, but the OEM -
 * that means the access control configuration "blob" on these devices
 * is signed by the respective OEM.
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "../core.h"
#include "../pinctrl-utils.h"

#define SSC_TLMM_GPIO_CFG_REG		0x00
#define SSC_TLMM_GPIO_PULL_MASK		GENMASK(1, 0)
#define SSC_TLMM_GPIO_FUNCTION_MASK	GENMASK(5, 2)
#define SSC_TLMM_GPIO_OUT_STRENGTH_MASK	GENMASK(8, 6)
#define SSC_TLMM_GPIO_OE_MASK		BIT(9)

#define SSC_TLMM_GPIO_VALUE_REG		0x04
#define SSC_TLMM_GPIO_VALUE_IN_MASK	BIT(0)
#define SSC_TLMM_GPIO_VALUE_OUT_MASK	BIT(1)

#define SSC_TLMM_GPIO_BIAS_DISABLE	0x0
#define SSC_TLMM_GPIO_PULL_DOWN		0x1
#define SSC_TLMM_GPIO_KEEPER		0x2
#define SSC_TLMM_GPIO_PULL_UP		0x3
#define SSC_TLMM_GPIO_DS_TO_VAL(v)	(v / 2 - 1)

#define SSC_TLMM_FUNCTION(fname)			\
	[SSC_TLMM_MUX_##fname] = {			\
		.name = #fname,				\
		.groups = fname##_groups,		\
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

#define SSC_TLMM_PINGROUP(id, f1, f2, f3, f4)		\
	{						\
		.name = "gpio" #id,			\
		.pins = gpio##id##_pins,		\
		.pin = id,				\
		.npins = ARRAY_SIZE(gpio##id##_pins),	\
		.funcs = (int[]){			\
			SSC_TLMM_MUX_gpio,		\
			SSC_TLMM_MUX_##f1,		\
			SSC_TLMM_MUX_##f2,		\
			SSC_TLMM_MUX_##f3,		\
			SSC_TLMM_MUX_##f4,		\
		},					\
		.nfuncs = 5,				\
	}

// note: not sure where ".nfuncs = 5" comes from, only 0/1/2 seem to be used
// (same as in pinctrl-lpass-lpi.c), and there are 4 bits so max of 16
// alas, the register layout is practically identical to pinctrl-lpass-lpi.c,
// so I will go with what they did

struct ssc_tlmm_pingroup {
	const char *name;
	const unsigned int *pins;
	unsigned int npins;
	unsigned int pin;
	unsigned int *funcs;
	unsigned int nfuncs;
};

struct ssc_tlmm_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
};

struct ssc_tlmm_pinctrl_variant_data {
	const struct pinctrl_pin_desc *pins;
	const u32 *pin_offset_lut;
	int npins;
	const struct ssc_tlmm_pingroup *groups;
	int ngroups;
	const struct ssc_tlmm_function *functions;
	int nfunctions;
};

struct ssc_tlmm_pinctrl {
	struct device *dev;
	struct pinctrl_dev *ctrl;
	struct gpio_chip chip;
	struct pinctrl_desc desc;
	char __iomem *tlmm_base;
	const struct ssc_tlmm_pinctrl_variant_data *data;
};

/* msm8998 variant specific data */
static const struct pinctrl_pin_desc msm8998_ssc_tlmm_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
	PINCTRL_PIN(19, "gpio19"),
};

// it's unclear to me why these are so nonlinear
static const u32 msm8998_ssc_tlmm_pin_offsets[] = {
	[0] = 0x0000,
	[1] = 0x1000,
	[2] = 0x2000,
	[3] = 0x2008,
	[4] = 0x3000,
	[5] = 0x3008,
	[6] = 0x4000,
	[7] = 0x4008,
	[8] = 0x5000,
	[9] = 0x5008,
	[10] = 0x5010,
	[11] = 0x5018,
	[12] = 0x6000,
	[13] = 0x6008,
	[14] = 0x7000,
	[15] = 0x7008,
	[16] = 0x8000,
	[17] = 0x8008,
	[18] = 0x9000,
	[19] = 0x9008,
};

enum msm8998_ssc_tlmm_functions {
	SSC_TLMM_MUX_gpio,
	SSC_TLMM_MUX_ssc_blsp_i2c1,
	SSC_TLMM_MUX_ssc_blsp_i2c2,
	SSC_TLMM_MUX_ssc_blsp_i2c3,
	SSC_TLMM_MUX_ssc_blsp_spi1,
	SSC_TLMM_MUX_ssc_blsp_spi2,
	SSC_TLMM_MUX_ssc_blsp_spi3,
	SSC_TLMM_MUX_ssc_blsp_uart1,
	SSC_TLMM_MUX_ssc_blsp_uart2,
	SSC_TLMM_MUX_ssc_blsp_uart3,
	SSC_TLMM_MUX__,
};

static const unsigned int gpio0_pins[] = { 0 };
static const unsigned int gpio1_pins[] = { 1 };
static const unsigned int gpio2_pins[] = { 2 };
static const unsigned int gpio3_pins[] = { 3 };
static const unsigned int gpio4_pins[] = { 4 };
static const unsigned int gpio5_pins[] = { 5 };
static const unsigned int gpio6_pins[] = { 6 };
static const unsigned int gpio7_pins[] = { 7 };
static const unsigned int gpio8_pins[] = { 8 };
static const unsigned int gpio9_pins[] = { 9 };
static const unsigned int gpio10_pins[] = { 10 };
static const unsigned int gpio11_pins[] = { 11 };
static const unsigned int gpio12_pins[] = { 12 };
static const unsigned int gpio13_pins[] = { 13 };
static const unsigned int gpio14_pins[] = { 14 };
static const unsigned int gpio15_pins[] = { 15 };
static const unsigned int gpio16_pins[] = { 16 };
static const unsigned int gpio17_pins[] = { 17 };
static const unsigned int gpio18_pins[] = { 18 };
static const unsigned int gpio19_pins[] = { 19 };

static const char * const gpio_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14",
	"gpio15", "gpio16", "gpio17", "gpio18", "gpio19"
};
static const char * const ssc_blsp_i2c1_groups[] = { "gpio8", "gpio9" };
static const char * const ssc_blsp_spi1_groups[] = { "gpio8", "gpio9", "gpio10", "gpio11" };
static const char * const ssc_blsp_uart1_groups[] = { "gpio12", "gpio13" };
static const char * const ssc_blsp_i2c2_groups[] = { "gpio4", "gpio5" };
static const char * const ssc_blsp_spi2_groups[] = { "gpio4", "gpio5", "gpio6", "gpio7" };
static const char * const ssc_blsp_uart2_groups[] = { "gpio14", "gpio15" };
static const char * const ssc_blsp_i2c3_groups[] = { "gpio2", "gpio3" };
static const char * const ssc_blsp_spi3_groups[] = { "gpio16", "gpio17", "gpio18", "gpio19" };
static const char * const ssc_blsp_uart3_groups[] = { "gpio6", "gpio7" };

static const struct ssc_tlmm_pingroup msm8998_groups[] = {
	SSC_TLMM_PINGROUP(0,  _,              _,             _, _),
	SSC_TLMM_PINGROUP(1,  _,              _,             _, _),
	SSC_TLMM_PINGROUP(2,  ssc_blsp_i2c3,  _,             _, _),
	SSC_TLMM_PINGROUP(3,  ssc_blsp_i2c3,  _,             _, _),
	SSC_TLMM_PINGROUP(4,  ssc_blsp_i2c2,  ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(5,  ssc_blsp_i2c2,  ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(6,  ssc_blsp_uart3, ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(7,  ssc_blsp_uart3, ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(8,  ssc_blsp_spi1,  ssc_blsp_i2c1, _, _),
	SSC_TLMM_PINGROUP(9,  ssc_blsp_spi1,  ssc_blsp_i2c1, _, _),
	SSC_TLMM_PINGROUP(10, ssc_blsp_spi1,  _,             _, _),
	SSC_TLMM_PINGROUP(11, ssc_blsp_spi1,  _,             _, _),
	SSC_TLMM_PINGROUP(12, ssc_blsp_uart1, _,             _, _),
	SSC_TLMM_PINGROUP(13, ssc_blsp_uart1, _,             _, _),
	SSC_TLMM_PINGROUP(14, ssc_blsp_uart2, _,             _, _),
	SSC_TLMM_PINGROUP(15, ssc_blsp_uart2, _,             _, _),
	SSC_TLMM_PINGROUP(16, ssc_blsp_spi3,  _,             _, _),
	SSC_TLMM_PINGROUP(17, ssc_blsp_spi3,  _,             _, _),
	SSC_TLMM_PINGROUP(18, ssc_blsp_spi3,  _,             _, _),
	SSC_TLMM_PINGROUP(19, ssc_blsp_spi3,  _,             _, _),
};

static const struct ssc_tlmm_function msm8998_functions[] = {
	SSC_TLMM_FUNCTION(gpio),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c1),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c2),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c3),
	SSC_TLMM_FUNCTION(ssc_blsp_spi1),
	SSC_TLMM_FUNCTION(ssc_blsp_spi2),
	SSC_TLMM_FUNCTION(ssc_blsp_spi3),
	SSC_TLMM_FUNCTION(ssc_blsp_uart1),
	SSC_TLMM_FUNCTION(ssc_blsp_uart2),
	SSC_TLMM_FUNCTION(ssc_blsp_uart3),
};

static struct ssc_tlmm_pinctrl_variant_data msm8998_ssc_tlmm_data = {
	.pins = msm8998_ssc_tlmm_pins,
	.pin_offset_lut = msm8998_ssc_tlmm_pin_offsets,
	.npins = ARRAY_SIZE(msm8998_ssc_tlmm_pins),
	.groups = msm8998_groups,
	.ngroups = ARRAY_SIZE(msm8998_groups),
	.functions = msm8998_functions,
	.nfunctions = ARRAY_SIZE(msm8998_functions),
};

static const struct pinctrl_pin_desc sdm845_ssc_tlmm_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
};

static const u32 sdm845_ssc_tlmm_pin_offsets[] = {
	[0] = 0x0000,
	[1] = 0x1000,
	[2] = 0x2000,
	[3] = 0x3000,
	[4] = 0x4000,
	[5] = 0x5000,
	[6] = 0x6000,
	[7] = 0x7000,
	[8] = 0x8000,
	[9] = 0x9000,
	[10] = 0xa000,
	[11] = 0xb000,
	[12] = 0xc000,
	[13] = 0xd000,
	[14] = 0xe000,
	[15] = 0xf000,
	[16] = 0x10000,
	[17] = 0x11000,
};

static const struct ssc_tlmm_pingroup sdm845_groups[] = {
	SSC_TLMM_PINGROUP(0,  _,              _,             _, _),
	SSC_TLMM_PINGROUP(1,  _,              _,             _, _),
	SSC_TLMM_PINGROUP(2,  ssc_blsp_i2c3,  _,             _, _),
	SSC_TLMM_PINGROUP(3,  ssc_blsp_i2c3,  _,             _, _),
	SSC_TLMM_PINGROUP(4,  ssc_blsp_i2c2,  ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(5,  ssc_blsp_i2c2,  ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(6,  ssc_blsp_uart3, ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(7,  ssc_blsp_uart3, ssc_blsp_spi2, _, _),
	SSC_TLMM_PINGROUP(8,  ssc_blsp_spi1,  ssc_blsp_i2c1, _, _),
	SSC_TLMM_PINGROUP(9,  ssc_blsp_spi1,  ssc_blsp_i2c1, _, _),
	SSC_TLMM_PINGROUP(10, ssc_blsp_spi1,  _,             _, _),
	SSC_TLMM_PINGROUP(11, ssc_blsp_spi1,  _,             _, _),
	SSC_TLMM_PINGROUP(12, ssc_blsp_uart1, _,             _, _),
	SSC_TLMM_PINGROUP(13, ssc_blsp_uart1, _,             _, _),
	SSC_TLMM_PINGROUP(14, ssc_blsp_uart2, _,             _, _),
	SSC_TLMM_PINGROUP(15, ssc_blsp_uart2, _,             _, _),
	SSC_TLMM_PINGROUP(16, ssc_blsp_spi3,  _,             _, _),
	SSC_TLMM_PINGROUP(17, ssc_blsp_spi3,  _,             _, _),
};

static const struct ssc_tlmm_function sdm845_functions[] = {
	SSC_TLMM_FUNCTION(gpio),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c1),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c2),
	SSC_TLMM_FUNCTION(ssc_blsp_i2c3),
	SSC_TLMM_FUNCTION(ssc_blsp_spi1),
	SSC_TLMM_FUNCTION(ssc_blsp_spi2),
	SSC_TLMM_FUNCTION(ssc_blsp_spi3),
	SSC_TLMM_FUNCTION(ssc_blsp_uart1),
	SSC_TLMM_FUNCTION(ssc_blsp_uart2),
	SSC_TLMM_FUNCTION(ssc_blsp_uart3),
};

static struct ssc_tlmm_pinctrl_variant_data sdm845_ssc_tlmm_data = {
	.pins = sdm845_ssc_tlmm_pins,
	.pin_offset_lut = sdm845_ssc_tlmm_pin_offsets,
	.npins = ARRAY_SIZE(sdm845_ssc_tlmm_pins),
	.groups = sdm845_groups,
	.ngroups = ARRAY_SIZE(sdm845_groups),
	.functions = sdm845_functions,
	.nfunctions = ARRAY_SIZE(sdm845_functions),
};

static u32 ssc_tlmm_pin_read_cfg(struct ssc_tlmm_pinctrl *state, unsigned int pin)
{
	return ioread32(state->tlmm_base + state->data->pin_offset_lut[pin] + SSC_TLMM_GPIO_CFG_REG);
}

static int ssc_tlmm_pin_read_gpio_value(struct ssc_tlmm_pinctrl *state, unsigned int pin)
{
	return ioread32(state->tlmm_base + state->data->pin_offset_lut[pin] + SSC_TLMM_GPIO_VALUE_REG) 
			& SSC_TLMM_GPIO_VALUE_IN_MASK;
}

static int ssc_tlmm_pin_write_config(struct ssc_tlmm_pinctrl *state, unsigned int pin, u32 val)
{
	iowrite32(val, state->tlmm_base + state->data->pin_offset_lut[pin] + SSC_TLMM_GPIO_CFG_REG);

	return 0;
}

static int ssc_tlmm_pin_write_gpio_value(struct ssc_tlmm_pinctrl *state, unsigned int pin, unsigned int value)
{
	u32 val = u32_encode_bits(value ? 1 : 0, SSC_TLMM_GPIO_VALUE_OUT_MASK);
	iowrite32(val, state->tlmm_base + state->data->pin_offset_lut[pin] + SSC_TLMM_GPIO_VALUE_REG);

	return 0;
}

static int ssc_tlmm_pin_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->ngroups;
}

static const char *ssc_tlmm_pin_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int group)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->groups[group].name;
}

static int ssc_tlmm_pin_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int group,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctrl->data->groups[group].pins;
	*num_pins = pctrl->data->groups[group].npins;

	return 0;
}

static const struct pinctrl_ops ssc_tlmm_gpio_pinctrl_ops = {
	.get_groups_count	= ssc_tlmm_pin_get_groups_count,
	.get_group_name		= ssc_tlmm_pin_get_group_name,
	.get_group_pins		= ssc_tlmm_pin_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinctrl_utils_free_map,
};

static int ssc_tlmm_pin_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->nfunctions;
}

static const char *ssc_tlmm_pin_get_function_name(struct pinctrl_dev *pctldev,
						   unsigned int function)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->functions[function].name;
}

static int ssc_tlmm_pin_get_function_groups(struct pinctrl_dev *pctldev,
					     unsigned int function,
					     const char *const **groups,
					     unsigned *const num_qgroups)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctrl->data->functions[function].groups;
	*num_qgroups = pctrl->data->functions[function].ngroups;

	return 0;
}

static int ssc_tlmm_pin_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
				 unsigned int group_num)
{
	struct ssc_tlmm_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct ssc_tlmm_pingroup *g = &pctrl->data->groups[group_num];
	u32 val;
	int i, pin = g->pin;

	for (i = 0; i < g->nfuncs; i++) {
		if (g->funcs[i] == function)
			break;
	}

	if (WARN_ON(i == g->nfuncs))
		return -EINVAL;

	val = ssc_tlmm_pin_read_cfg(pctrl, pin);
	u32p_replace_bits(&val, i, SSC_TLMM_GPIO_FUNCTION_MASK);
	ssc_tlmm_pin_write_config(pctrl, pin, val);

	return 0;
}

static const struct pinmux_ops ssc_tlmm_pinmux_ops = {
	.get_functions_count	= ssc_tlmm_pin_get_functions_count,
	.get_function_name	= ssc_tlmm_pin_get_function_name,
	.get_function_groups	= ssc_tlmm_pin_get_function_groups,
	.set_mux		= ssc_tlmm_pin_set_mux,
};

static int ssc_tlmm_config_get(struct pinctrl_dev *pctldev,
			       unsigned int pin, unsigned long *config)
{
	unsigned int param = pinconf_to_config_param(*config);
	struct ssc_tlmm_pinctrl *state = dev_get_drvdata(pctldev->dev);
	unsigned int arg = 0;
	int is_out;
	int pull;
	u32 ctl_reg;

	ctl_reg = ssc_tlmm_pin_read_cfg(state, pin);
	is_out = ctl_reg & SSC_TLMM_GPIO_OE_MASK;
	pull = FIELD_GET(SSC_TLMM_GPIO_PULL_MASK, ctl_reg);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (pull == SSC_TLMM_GPIO_BIAS_DISABLE)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (pull == SSC_TLMM_GPIO_PULL_DOWN)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (pull == SSC_TLMM_GPIO_KEEPER)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pull == SSC_TLMM_GPIO_PULL_UP)
			arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		if (is_out)
			arg = 1;
		break;
	default:
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int ssc_tlmm_config_set(struct pinctrl_dev *pctldev, unsigned int group,
			       unsigned long *configs, unsigned int nconfs)
{
	struct ssc_tlmm_pinctrl *pctrl = dev_get_drvdata(pctldev->dev);
	unsigned int param, arg, pullup, strength;
	bool value, output_enabled = false;
	const struct ssc_tlmm_pingroup *g;
	int i;
	u32 val;

	g = &pctrl->data->groups[group];
	for (i = 0; i < nconfs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			pullup = SSC_TLMM_GPIO_BIAS_DISABLE;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pullup = SSC_TLMM_GPIO_PULL_DOWN;
			break;
		case PIN_CONFIG_BIAS_BUS_HOLD:
			pullup = SSC_TLMM_GPIO_KEEPER;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			pullup = SSC_TLMM_GPIO_PULL_UP;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			output_enabled = false;
			break;
		case PIN_CONFIG_OUTPUT:
			output_enabled = true;
			value = arg;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			strength = arg;
			break;
		default:
			return -EINVAL;
		}
	}

	val = ssc_tlmm_pin_read_cfg(pctrl, group);

	u32p_replace_bits(&val, pullup, SSC_TLMM_GPIO_PULL_MASK);
	u32p_replace_bits(&val, SSC_TLMM_GPIO_DS_TO_VAL(strength),
			  SSC_TLMM_GPIO_OUT_STRENGTH_MASK);
	u32p_replace_bits(&val, output_enabled, SSC_TLMM_GPIO_OE_MASK);

	ssc_tlmm_pin_write_config(pctrl, group, val);

	if (output_enabled)
		ssc_tlmm_pin_write_gpio_value(pctrl, group, value);

	return 0;
}

static const struct pinconf_ops ssc_tlmm_gpio_pinconf_ops = {
	.is_generic			= true,
	.pin_config_group_get		= ssc_tlmm_config_get,
	.pin_config_group_set		= ssc_tlmm_config_set,
};

static int ssc_tlmm_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	struct ssc_tlmm_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);

	return ssc_tlmm_config_set(state->ctrl, pin, &config, 1);
}

static int ssc_tlmm_gpio_direction_output(struct gpio_chip *chip,
					  unsigned int pin, int val)
{
	struct ssc_tlmm_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, val);

	return ssc_tlmm_config_set(state->ctrl, pin, &config, 1);
}

static int ssc_tlmm_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ssc_tlmm_pinctrl *state = gpiochip_get_data(chip);

	return ssc_tlmm_pin_read_gpio_value(state, pin);
}

static void ssc_tlmm_gpio_set(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct ssc_tlmm_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);

	ssc_tlmm_config_set(state->ctrl, pin, &config, 1);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static unsigned int ssc_tlmm_regval_to_drive(u32 val)
{
	return (val + 1) * 2;
}

static void ssc_tlmm_gpio_dbg_show_one(struct seq_file *s,
				       struct pinctrl_dev *pctldev,
				       struct gpio_chip *chip,
				       unsigned int pin,
				       unsigned int gpio)
{
	struct ssc_tlmm_pinctrl *state = gpiochip_get_data(chip);
	struct pinctrl_pin_desc pindesc;
	unsigned int func;
	int is_out;
	int drive;
	int pull;
	int value;
	u32 ctl_reg;

	static const char * const pulls[] = {
		"no pull",
		"pull down",
		"keeper",
		"pull up"
	};

	pctldev = pctldev ? : state->ctrl;
	pindesc = pctldev->desc->pins[pin];
	ctl_reg = ssc_tlmm_pin_read_cfg(state, pin);
	is_out = ctl_reg & SSC_TLMM_GPIO_OE_MASK;
	value = ssc_tlmm_pin_read_gpio_value(state, pin);

	func = FIELD_GET(SSC_TLMM_GPIO_FUNCTION_MASK, ctl_reg);
	drive = FIELD_GET(SSC_TLMM_GPIO_OUT_STRENGTH_MASK, ctl_reg);
	pull = FIELD_GET(SSC_TLMM_GPIO_PULL_MASK, ctl_reg);

	seq_printf(s, " %-8s: %-3s %-4s %d", pindesc.name, is_out ? "out" : "in", value ? "high" : "low", func);
	seq_printf(s, " %dmA", ssc_tlmm_regval_to_drive(drive));
	seq_printf(s, " %s", pulls[pull]);
}

static void ssc_tlmm_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned int gpio = chip->base;
	unsigned int i;

	for (i = 0; i < chip->ngpio; i++, gpio++) {
		ssc_tlmm_gpio_dbg_show_one(s, NULL, chip, i, gpio);
		seq_puts(s, "\n");
	}
}

#else
#define ssc_tlmm_gpio_dbg_show NULL
#endif

static const struct gpio_chip ssc_tlmm_gpio_template = {
	.direction_input	= ssc_tlmm_gpio_direction_input,
	.direction_output	= ssc_tlmm_gpio_direction_output,
	.get			= ssc_tlmm_gpio_get,
	.set			= ssc_tlmm_gpio_set,
	.request		= gpiochip_generic_request,
	.free			= gpiochip_generic_free,
	.dbg_show		= ssc_tlmm_gpio_dbg_show,
};

static int ssc_tlmm_pinctrl_probe(struct platform_device *pdev)
{
	const struct ssc_tlmm_pinctrl_variant_data *data;
	struct device *dev = &pdev->dev;
	struct ssc_tlmm_pinctrl *pctrl;
	int ret;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pctrl);

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	pctrl->data = data;
	pctrl->dev = &pdev->dev;

	pctrl->tlmm_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctrl->tlmm_base))
		return dev_err_probe(dev, PTR_ERR(pctrl->tlmm_base),
				     "TLMM resource not provided\n");

	pctrl->desc.pctlops = &ssc_tlmm_gpio_pinctrl_ops;
	pctrl->desc.pmxops = &ssc_tlmm_pinmux_ops;
	pctrl->desc.confops = &ssc_tlmm_gpio_pinconf_ops;
	pctrl->desc.owner = THIS_MODULE;
	pctrl->desc.name = dev_name(dev);
	pctrl->desc.pins = data->pins;
	pctrl->desc.npins = data->npins;
	pctrl->chip = ssc_tlmm_gpio_template;
	pctrl->chip.parent = dev;
	pctrl->chip.base = -1;
	pctrl->chip.ngpio = data->npins;
	pctrl->chip.label = dev_name(dev);
	pctrl->chip.of_gpio_n_cells = 2;
	pctrl->chip.can_sleep = false;

	pctrl->ctrl = devm_pinctrl_register(dev, &pctrl->desc, pctrl);
	if (IS_ERR(pctrl->ctrl)) {
		ret = PTR_ERR(pctrl->ctrl);
		dev_err(dev, "failed to add pin controller\n");
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &pctrl->chip, pctrl);
	if (ret) {
		dev_err(pctrl->dev, "can't add gpio chip\n");
		return ret;
	}

	return 0;
}

static int ssc_tlmm_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ssc_tlmm_pinctrl_of_match[] = {
	{
	       .compatible = "qcom,msm8998-ssc-tlmm-pinctrl",
	       .data = &msm8998_ssc_tlmm_data,
	},
	{
	       .compatible = "qcom,sdm845-ssc-tlmm-pinctrl",
	       .data = &sdm845_ssc_tlmm_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ssc_tlmm_pinctrl_of_match);

static struct platform_driver ssc_tlmm_pinctrl_driver = {
	.driver = {
		   .name = "qcom-ssc-tlmm-pinctrl",
		   .of_match_table = ssc_tlmm_pinctrl_of_match,
	},
	.probe = ssc_tlmm_pinctrl_probe,
	.remove = ssc_tlmm_pinctrl_remove,
};

module_platform_driver(ssc_tlmm_pinctrl_driver);
MODULE_AUTHOR("Michael Srba");
MODULE_DESCRIPTION("GPIO pin control driver for the SSC-specific TLMM on some qcom SoCs");
MODULE_LICENSE("GPL");
