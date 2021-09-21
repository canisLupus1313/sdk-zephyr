/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sys/reboot.h>
#include <openthread/logging.h>
#include <openthread/instance.h>
#include <openthread/platform/misc.h>

#include "platform-zephyr.h"

void otPlatReset(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	/* This function does nothing on the Posix platform. */
	sys_reboot(SYS_REBOOT_WARM);
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	return OT_PLAT_RESET_REASON_POWER_ON;
}

void otPlatWakeHost(void)
{
	/* TODO */
}

void otPlatAssertFail(const char *aFilename, int aLineNumber)
{
	otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_MAC, "%s:%d\n", aFilename, aLineNumber);
	__ASSERT(false, "OpenThread ASSERT @ %s:%d", aFilename, aLineNumber);
}
