/*
 *  Copyright (c) 2022, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// C standard libraries
#include <stddef.h>
#include <string.h>
#include <errno.h>

// Zephir Libraries
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// Zephyr OpenThread integration Library
#include <zephyr/net/openthread.h>

// OpenThread BLE driver API
#include <openthread/platform/ble.h>

// --------------
// Zephyr Logging
// --------------

#define LOG_MODULE_NAME net_openthread_ble
#define LOG_LEVEL       CONFIG_OPENTHREAD_LOG_LEVEL

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// ----------------
// Settings
// ----------------

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Temporarily use the UUIDs of the Nordic UART service to allow using existing tools for testing
// This will be replaced with a Threadgroup UUID

// TO DO align uuid
#define MY_SERVICE_UUID 0xfffb
#define RX_CHARACTERISTIC_UUID                                                                     \
	BT_UUID_128_ENCODE(0x7fddf61f, 0x280a, 0x4773, 0xb448, 0xba1b8fe0dd69)
#define TX_CHARACTERISTIC_UUID                                                                     \
	BT_UUID_128_ENCODE(0x6bd10d8b, 0x85a7, 0x4e5a, 0xba2d, 0xc83558a5f220)

#define BT_UUID_TCAT_SERVICE    BT_UUID_DECLARE_16(MY_SERVICE_UUID)
#define BT_UUID_TCAT_SERVICE_RX BT_UUID_DECLARE_128(RX_CHARACTERISTIC_UUID)
#define BT_UUID_TCAT_SERVICE_TX BT_UUID_DECLARE_128(TX_CHARACTERISTIC_UUID)

#define PLAT_BLE_RING_BUF_SIZE     500
#define PLAT_BLE_THREAD_STACK_SIZE 4200
#define PLAT_BLE_THREAD_DEALY      500
#define PLAT_BLE_MSG_DATA_MAX      CONFIG_BT_L2CAP_TX_MTU // must match the maximum MTU size used

#define PLAT_BLE_MSG_CONNECT    0xFE
#define PLAT_BLE_MSG_DISCONNECT 0xFF

// ---------------------------------
// Zephyr Kernel Objects
// ---------------------------------

static void otPlatBleThread(void *, void *, void *);
uint8_t otPlatBleMsgBuf[PLAT_BLE_MSG_DATA_MAX];

K_SEM_DEFINE(otPlatBleInitSemaphor, 0, 1);
K_SEM_DEFINE(otPlatBleEventSemaphor, 0, 10000);
RING_BUF_DECLARE(otPlatBleRingBuf, PLAT_BLE_RING_BUF_SIZE);
K_THREAD_DEFINE(otPlatBleTid, PLAT_BLE_THREAD_STACK_SIZE, otPlatBleThread, NULL, NULL, NULL, 5, 0,
		PLAT_BLE_THREAD_DEALY);

// ---------------------------------
// OpenThread Objects
// ---------------------------------

otInstance *otPlatBleOpenThreadInstance = NULL;

// ---------------------------------
// BLE service Objects
// ---------------------------------

// forward declaration for callback functions
static ssize_t on_receive(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags);
void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Service Declaration and Registration
BT_GATT_SERVICE_DEFINE(my_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_TCAT_SERVICE),
		       BT_GATT_CHARACTERISTIC(BT_UUID_TCAT_SERVICE_RX,
					      BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
					      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL,
					      on_receive, NULL),
		       BT_GATT_CHARACTERISTIC(BT_UUID_TCAT_SERVICE_TX, BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_READ, NULL, NULL, NULL),
		       BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

// ---------------------------------
// Zephyr BLE Objects
// ---------------------------------

// forward declaration for callback functions
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param);
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout);

static struct bt_conn *otPlatBleConnection = NULL;

static struct bt_conn_cb conn_callbacks = {.connected = connected,
					   .disconnected = disconnected,
					   .le_param_req = le_param_req,
					   .le_param_updated = le_param_updated};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(MY_SERVICE_UUID)),
};

// ------------------------------------------------------------------
// Zephyr BLE Message Queue and Thread
// ------------------------------------------------------------------

// SDK integration: Should be includeded in the OpenThread Thread to avoid the need for another
// Thread with large stack

static bool otPlatBleQueueMsg(const uint8_t *aData, uint8_t aLen, int8_t aRssi)
{
	otError error = OT_ERROR_NONE;

	if (aLen <= PLAT_BLE_MSG_DATA_MAX && aData == NULL) {
		return OT_ERROR_INVALID_ARGS;
	}

	k_sched_lock();

	if (ring_buf_space_get(&otPlatBleRingBuf) >=
			    sizeof(aLen) + sizeof(aRssi) + (aLen <= PLAT_BLE_MSG_DATA_MAX)
		    ? aLen
		    : 0) {
		ring_buf_put(&otPlatBleRingBuf, &aLen, sizeof(aLen));
		ring_buf_put(&otPlatBleRingBuf, &aRssi, sizeof(aRssi));
		if (aLen <= PLAT_BLE_MSG_DATA_MAX) {
			ring_buf_put(&otPlatBleRingBuf, aData, aLen);
		}
		k_sem_give(&otPlatBleEventSemaphor);
	} else {
		error = OT_ERROR_NO_BUFS;
	}

	k_sched_unlock();

	return error;
}

static void otPlatBleThread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	uint8_t len;
	int8_t rssi;
	otBleRadioPacket myPacket;

	LOG_INF("otPlatBleThread started");

	while (1) {
		k_sem_take(&otPlatBleEventSemaphor, K_FOREVER);
		ring_buf_get(&otPlatBleRingBuf, &len, sizeof(len));
		ring_buf_get(&otPlatBleRingBuf, &rssi, sizeof(rssi));
		if (len <= PLAT_BLE_MSG_DATA_MAX) {
			ring_buf_get(&otPlatBleRingBuf, otPlatBleMsgBuf, len);
		}

		openthread_api_mutex_lock(openthread_get_default_context());

		if (len <= PLAT_BLE_MSG_DATA_MAX) {
			// The packet parameter in otPlatBleGattServerOnWriteRequest is not const.
			// Re-write all members.
			myPacket.mValue = otPlatBleMsgBuf;
			myPacket.mPower = rssi;
			myPacket.mLength = len;
			otPlatBleGattServerOnWriteRequest(otPlatBleOpenThreadInstance, 0,
							  &myPacket);
		} else if (len == PLAT_BLE_MSG_CONNECT) {
			otPlatBleGapOnConnected(otPlatBleOpenThreadInstance, 0);
		} else if (len == PLAT_BLE_MSG_DISCONNECT) {
			otPlatBleGapOnDisconnected(otPlatBleOpenThreadInstance, 0);
		}
		openthread_api_mutex_unlock(openthread_get_default_context());
	}
}

// ------------------------------------------------------------------
// Zephyr BLE service callbacks
// ------------------------------------------------------------------

/* This function is called whenever the RX Characteristic has been written to by a Client */
static ssize_t on_receive(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Received data, handle %"PRIu16", len %"PRIu16, attr->handle, len);

	otError error = otPlatBleQueueMsg(buf, len, 0 /* TBD */);
	if (error != OT_ERROR_NONE) {
		LOG_WRN("Error queuing message: %s", otThreadErrorToString(error));
	}

	return len;
}

/* This function is called whenever a Notification has been sent by the TX Characteristic */
static void on_sent(struct bt_conn *conn, void *user_data)
{
	// TODO verify if needed
	ARG_UNUSED(user_data);

	LOG_DBG("Data sent");
}

/* This function is called whenever the CCCD register has been changed by the client*/
void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	switch (value) {
	case BT_GATT_CCC_NOTIFY:
		// How should the device driver inform about CCCD change?
		// Quick fix: Delay otPlatBleGapOnConnected(...) until notifications can be sent.

		otError error = otPlatBleQueueMsg(NULL, PLAT_BLE_MSG_CONNECT, 0);
		if (error != OT_ERROR_NONE) {
			LOG_WRN("Error queuing message: %s", otThreadErrorToString(error));
		}

		uint16_t mtu;
		error = otPlatBleGattMtuGet(otPlatBleOpenThreadInstance, &mtu);
		if (error != OT_ERROR_NONE) {
			LOG_WRN("Error retrieving mtu: %s", otThreadErrorToString(error));
		}

		LOG_INF("CCCD update (mtu=%"PRIu16")!", mtu);

		break;

	case BT_GATT_CCC_INDICATE:
		break;

	case 0:
		break;
	}
}

otError otPlatBleGattServerIndicate(otInstance *aInstance, uint16_t aHandle,
				    const otBleRadioPacket *aPacket)
{
	ARG_UNUSED(aInstance);

	const struct bt_gatt_attr *attr = &my_service.attrs[3];

	struct bt_gatt_notify_params params = {.uuid = BT_UUID_TCAT_SERVICE_TX,
					       .attr = attr,
					       .data = aPacket->mValue,
					       .len = aPacket->mLength,
					       .func = on_sent};

	LOG_INF("Send data, handle %d, len %d", attr->handle, aPacket->mLength);

	// Only one connection supported
	if (aHandle != 0) {
		return OT_ERROR_INVALID_ARGS;
	}

	if (otPlatBleConnection == NULL) {
		return OT_ERROR_INVALID_STATE;
	}

	// Check whether notifications are enabled or not
	if (bt_gatt_is_subscribed(otPlatBleConnection, attr, BT_GATT_CCC_NOTIFY)) {
		// Send the notification
		if (bt_gatt_notify_cb(otPlatBleConnection, &params)) {
			LOG_INF("Error, unable to send notification");
			return OT_ERROR_INVALID_ARGS;
		}
	} else {
		LOG_INF("Warning, notification not enabled on the selected attribute");
		return OT_ERROR_INVALID_STATE;
	}

	return OT_ERROR_NONE;
}

otError otPlatBleGattMtuGet(otInstance *aInstance, uint16_t *aMtu)
{
	ARG_UNUSED(aInstance);

	if (otPlatBleConnection == NULL) {
		return OT_ERROR_FAILED;
	}

	if (aMtu != NULL) {
		*aMtu = bt_gatt_get_mtu(otPlatBleConnection);
	}

	return OT_ERROR_NONE;
}

otError otPlatBleGapDisconnect(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	if (otPlatBleConnection == NULL) {
		return OT_ERROR_INVALID_STATE;
	}

	if (bt_conn_disconnect(otPlatBleConnection, BT_HCI_ERR_REMOTE_USER_TERM_CONN)) {
		return OT_ERROR_INVALID_STATE;
	}

	return OT_ERROR_NONE;
}

// ------------------------------------------------------------------
// Zephyr BLE callbacks
// ------------------------------------------------------------------

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	otPlatBleConnection = bt_conn_ref(conn);

	if (err) {
		LOG_INF("Connection failed (err %u)", err);
		return;
	} else if (bt_conn_get_info(conn, &info)) {
		LOG_INF("Could not parse connection info");
	} else {
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		uint16_t mtu;
		otError error = otPlatBleGattMtuGet(otPlatBleOpenThreadInstance, &mtu);
		if (error != OT_ERROR_NONE) {
			LOG_WRN("Error retrieving mtu: %s", otThreadErrorToString(error));
		}

		LOG_INF("Connection established (mtu=%"PRIu16")!", mtu);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %"PRIu8")", reason);

	if (otPlatBleConnection) {
		bt_conn_unref(otPlatBleConnection);
		otPlatBleConnection = NULL;

		otError error = otPlatBleQueueMsg(NULL, PLAT_BLE_MSG_DISCONNECT, 0);
		if (error != OT_ERROR_NONE) {
			LOG_WRN("Error queuing message: %s", otThreadErrorToString(error));
		}
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	if (bt_conn_get_info(conn, &info)) {
		LOG_INF("Could not parse connection info");
	} else {
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		uint16_t mtu;
		otError error = otPlatBleGattMtuGet(otPlatBleOpenThreadInstance, &mtu);
		if (error != OT_ERROR_NONE) {
			LOG_WRN("Error retrieving mtu: %s", otThreadErrorToString(error));
		}

		LOG_INF("Connection parameters updated (mtu=%"PRIu16")!", mtu);
	}
}

static void bt_ready(int err)
{
	if (err) {
		LOG_INF("BLE init failed with error code %d", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	k_sem_give(&otPlatBleInitSemaphor); // BLE stack up an running
}

otError otPlatBleGapAdvStart(otInstance *aInstance, uint16_t aInterval)
{
	ARG_UNUSED(aInstance);
	ARG_UNUSED(aInterval); // To be decided how to derive the min max range from this value

	// TO DO advertisement format change
	int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err != 0 && err != -EALREADY) {
		LOG_INF("Advertising failed to start (err %d)", err);
		return OT_ERROR_INVALID_STATE;
	}

	LOG_INF("Advertising successfully started");

	return OT_ERROR_NONE;
}

otError otPlatBleGapAdvStop(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	int err = bt_le_adv_stop();
	if (err != 0 && err != -EALREADY) {
		LOG_WRN("Advertisement failed to stop (err %d)", err);
		return OT_ERROR_FAILED;
	}
	return OT_ERROR_NONE;
}

// ------------------------------------------------------------------
// Zephyr BLE initialization
// ------------------------------------------------------------------

otError otPlatBleEnable(otInstance *aInstance)
{
	int err;

	otPlatBleOpenThreadInstance = aInstance;
	err = bt_enable(bt_ready);

	if (err != 0 && err != -EALREADY) {
		LOG_INF("BLE enable failed with error code %d", err);
		return OT_ERROR_FAILED;
	} else if(err == -EALREADY) {
		err = k_sem_take(&otPlatBleInitSemaphor, K_MSEC(500)); // ignore
		return OT_ERROR_NONE;
	}

	err = k_sem_take(&otPlatBleInitSemaphor, K_MSEC(500));

	if (!err) {
		LOG_INF("Bluetooth initialized");
	} else {
		LOG_INF("BLE initialization did not complete in time");
		return OT_ERROR_FAILED;
	}

	return OT_ERROR_NONE;
}

otError otPlatBleDisable(otInstance *aInstance)
{
	ARG_UNUSED(aInstance);

	/*int err = bt_disable();

	printk("%d\n" , err);

	if (err != 0 && err != -EALREADY) {
		LOG_WRN("Error disabling bluetooth (err %d)", err);
		return OT_ERROR_FAILED;
	}*/

	return OT_ERROR_NONE;
}
