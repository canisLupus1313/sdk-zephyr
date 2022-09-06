/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_OPENTHREAD_SOCKETS_H_
#define ZEPHYR_DRIVERS_OPENTHREAD_SOCKETS_H_

#ifdef __cplusplus
extern "C" {
#endif

extern const struct socket_dns_offload openthread_dns_ops;
extern void openthread_sockets_init(void);
extern int openthread_socket_create(int family, int type, int proto);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_OPENTHREAD_SOCKETS_H_ */
