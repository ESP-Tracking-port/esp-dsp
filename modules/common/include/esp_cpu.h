/*
 * SPDX-FileCopyrightText: 2010-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ESP_CPU_H
#define _ESP_CPU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hal/cpu_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_WATCHPOINT_LOAD 0x40000000
#define ESP_WATCHPOINT_STORE 0x80000000
#define ESP_WATCHPOINT_ACCESS 0xC0000000

typedef uint32_t esp_cpu_ccount_t;

/** @brief Read current stack pointer address
 *
 */
static inline void *esp_cpu_get_sp(void)
{
    return NULL;
}

/**
 * @brief Stall CPU using RTC controller
 * @param cpu_id ID of the CPU to stall (0 = PRO, 1 = APP)
 */
#define esp_cpu_stall(...)

/**
 * @brief Un-stall CPU using RTC controller
 * @param cpu_id ID of the CPU to un-stall (0 = PRO, 1 = APP)
 */
#define esp_cpu_unstall(...)

/**
 * @brief Reset CPU using RTC controller
 * @param cpu_id ID of the CPU to reset (0 = PRO, 1 = APP)
 */
#define esp_cpu_reset(...)

/**
 * @brief Returns true if a JTAG debugger is attached to CPU
 * OCD (on chip debug) port.
 *
 * @note If "Make exception and panic handlers JTAG/OCD aware"
 * is disabled, this function always returns false.
 */
#define esp_cpu_in_ocd_debug_mode(...) 0

static inline esp_cpu_ccount_t esp_cpu_get_ccount(void)
{
    return 1;
}

static inline void esp_cpu_set_ccount(esp_cpu_ccount_t val)
{
}

/**
 * @brief Set a watchpoint to break/panic when a certain memory range is accessed.
 *
 * @param no Watchpoint number. On the ESP32, this can be 0 or 1.
 * @param adr Base address to watch
 * @param size Size of the region, starting at the base address, to watch. Must
 *             be one of 2^n, with n in [0..6].
 * @param flags One of ESP_WATCHPOINT_* flags
 *
 * @return ESP_ERR_INVALID_ARG on invalid arg, ESP_OK otherwise
 *
 * @warning The ESP32 watchpoint hardware watches a region of bytes by effectively
 *          masking away the lower n bits for a region with size 2^n. If adr does
 *          not have zero for these lower n bits, you may not be watching the
 *          region you intended.
 */
#define esp_cpu_set_watchpoint(...)

/**
 * @brief Clear a watchpoint
 *
 * @param no Watchpoint to clear
 *
 */
#define esp_cpu_clear_watchpoint(...)

#ifdef __cplusplus
}
#endif

#endif // _ESP_CPU_H
