/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 * Copyright (C) 2018-2022 Linaro Ltd.
 */
#ifndef _IPA_INTERRUPT_H_
#define _IPA_INTERRUPT_H_

#include <linux/types.h>
#include <linux/bits.h>

struct ipa;

/**
 * ipa_interrupt_clear_irq - Clear an IRQ by ID
 * @ipa:		IPA Structure
 * @irq_id:		The ID of the IRQ to clear
 * 
 * This function is called by interrupt handlers. The microcontroller
 * interrupts need to be cleared before processing, but the TX SUSPEND
 * interrupt must be clear after processing.
 */
void ipa_interrupt_clear_irq(struct ipa *ipa, enum ipa_irq_id irq_id);

/**
 * ipa_interrupt_suspend_enable - Enable TX_SUSPEND for an endpoint
 * @interrupt:		IPA interrupt structure
 * @endpoint_id:	Endpoint whose interrupt should be enabled
 *
 * Note:  The "TX" in the name is from the perspective of the IPA hardware.
 * A TX_SUSPEND interrupt arrives on an AP RX enpoint when packet data can't
 * be delivered to the endpoint because it is suspended (or its underlying
 * channel is stopped).
 */
void ipa_interrupt_suspend_enable(struct ipa *ipa,
				  u32 endpoint_id);

/**
 * ipa_interrupt_suspend_disable - Disable TX_SUSPEND for an endpoint
 * @interrupt:		IPA interrupt structure
 * @endpoint_id:	Endpoint whose interrupt should be disabled
 */
void ipa_interrupt_suspend_disable(struct ipa *ipa,
				   u32 endpoint_id);

/**
 * ipa_interrupt_suspend_clear_all - clear all suspend interrupts
 * @interrupt:	IPA interrupt structure
 *
 * Clear the TX_SUSPEND interrupt for all endpoints that signaled it.
 */
void ipa_interrupt_suspend_clear_all(struct ipa *ipa);

/**
 * ipa_interrupt_simulate_suspend() - Simulate TX_SUSPEND IPA interrupt
 * @interrupt:	IPA interrupt structure
 *
 * This calls the TX_SUSPEND interrupt handler, as if such an interrupt
 * had been signaled.  This is needed to work around a hardware quirk
 * that occurs if aggregation is active on an endpoint when its underlying
 * channel is suspended.
 */
void ipa_interrupt_simulate_suspend(struct ipa *ipa);

/**
 * ipa_interrupt_config() - Configure the IPA interrupt framework
 * @ipa:	IPA pointer
 *
 * Return:	Pointer to IPA SMP2P info, or a pointer-coded error
 */
int ipa_interrupt_config(struct ipa *ipa);

/**
 * ipa_interrupt_deconfig() - Inverse of ipa_interrupt_config()
 * @interrupt:	IPA interrupt structure
 */
void ipa_interrupt_deconfig(struct ipa *ipa);

#endif /* _IPA_INTERRUPT_H_ */
