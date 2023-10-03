/* main.c - OpenThread NCP */

/*
 * Copyright (c) 2020 Tridonic GmbH & Co KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ot_br, LOG_LEVEL_DBG);

#define APP_BANNER "***** OpenThread NCP on Zephyr %s *****"

void bt_ctlr_assert_handle(char *file, uint32_t line)
{

}

int main(void)
{
	LOG_INF(APP_BANNER, CONFIG_NET_SAMPLE_APPLICATION_VERSION);
	return 0;
}
