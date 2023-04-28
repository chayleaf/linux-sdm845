# SPDX-License-Identifier: GPL-2.0
#
ccflags-y += -DDEBUG
obj-$(CONFIG_TYPEC_QCOM_PMIC)		+= qcom_pmic_tcpm.o
qcom_pmic_tcpm-y			+= qcom_pmic_virt_tcpm.o \
					   qcom_pmic_typec.o \
					   qcom_pmic_pdphy.o \
					   qcom_pmic_typec_reg_pm8150b.o \
					   qcom_pmic_typec_reg_pmi8998.o
