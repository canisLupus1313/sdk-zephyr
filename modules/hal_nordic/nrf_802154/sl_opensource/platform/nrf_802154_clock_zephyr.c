/*
 * Copyright (c) 2020 - 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <platform/nrf_802154_clock.h>
#include "nrf_802154_config.h"

#include <stddef.h>

#include <compiler_abstraction.h>
#if !defined(NRF54H_SERIES)
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#endif
#include <zephyr/drivers/clock_control.h>

#define NRF_LRCCONF_RADIO_PD NRF_LRCCONF010

static bool hfclk_is_running;
static bool lfclk_is_running;
#if !defined(NRF54H_SERIES)
static struct onoff_client hfclk_cli;
static struct onoff_client lfclk_cli;
#endif

void nrf_802154_clock_init(void)
{
	/* Intentionally empty. */
}

void nrf_802154_clock_deinit(void)
{
	/* Intentionally empty. */
}

#if !defined(NRF54H_SERIES)
static void hfclk_on_callback(struct onoff_manager *mgr,
			      struct onoff_client  *cli,
			      uint32_t state,
			      int res)
{
	hfclk_is_running = true;
	nrf_802154_clock_hfclk_ready();
}
#endif

void nrf_802154_clock_hfclk_start(void)
{
#if defined(NRF54H_SERIES)
	/* Use register directly, there is no support for that task in nrf_lrcconf_task_trigger */
	NRF_LRCCONF_RADIO_PD->EVENTS_HFXOSTARTED = 0x0;
	NRF_LRCCONF_RADIO_PD->TASKS_REQHFXO = 0x1;
#else /* defined(NRF54H_SERIES) */
	int ret;
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

	__ASSERT_NO_MSG(mgr != NULL);

	sys_notify_init_callback(&hfclk_cli.notify, hfclk_on_callback);

	ret = onoff_request(mgr, &hfclk_cli);
	__ASSERT_NO_MSG(ret >= 0);
#endif /* defined(NRF54H_SERIES) */
}

void nrf_802154_clock_hfclk_stop(void)
{
#if defined(NRF54H_SERIES)
	/* Use register directly, there is no support for that task in nrf_lrcconf_task_trigger */
	NRF_LRCCONF_RADIO_PD->TASKS_STOPREQHFXO=0x1;
	NRF_LRCCONF_RADIO_PD->EVENTS_HFXOSTARTED = 0x0;
#else /* defined(NRF54H_SERIES) */
	int ret;
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

	__ASSERT_NO_MSG(mgr != NULL);

	ret = onoff_cancel_or_release(mgr, &hfclk_cli);
	__ASSERT_NO_MSG(ret >= 0);
	hfclk_is_running = false;
#endif /* defined(NRF54H_SERIES) */
}

bool nrf_802154_clock_hfclk_is_running(void)
{
	return hfclk_is_running;
}

#if !defined(NRF54H_SERIES)
static void lfclk_on_callback(struct onoff_manager *mgr,
			      struct onoff_client  *cli,
			      uint32_t state,
			      int res)
{
	lfclk_is_running = true;
	nrf_802154_clock_lfclk_ready();
}
#endif

void nrf_802154_clock_lfclk_start(void)
{
#if !defined(NRF54H_SERIES)
	int ret;
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);

	__ASSERT_NO_MSG(mgr != NULL);

	sys_notify_init_callback(&lfclk_cli.notify, lfclk_on_callback);

	ret = onoff_request(mgr, &lfclk_cli);
	__ASSERT_NO_MSG(ret >= 0);
#endif
}

void nrf_802154_clock_lfclk_stop(void)
{
#if !defined(NRF54H_SERIES)
	int ret;
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);

	__ASSERT_NO_MSG(mgr != NULL);

	ret = onoff_cancel_or_release(mgr, &lfclk_cli);
	__ASSERT_NO_MSG(ret >= 0);
	lfclk_is_running = false;
#endif
}

bool nrf_802154_clock_lfclk_is_running(void)
{
	return lfclk_is_running;
}

__WEAK void nrf_802154_clock_lfclk_ready(void)
{
	/* Intentionally empty. */
}
