/*
 * Copyright (c) 2024
 */
 
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "ble_nus_cmd.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_nus_cmd, CONFIG_BLE_NUS_CMD_LOG_LEVEL);

#define DEVICE_NAME        CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN    (sizeof(DEVICE_NAME) - 1)


#define INTERVAL_MIN  15 
#define INTERVAL_MAX  15 



static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param);
static void conn_params_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout);
static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
static void auth_cancel(struct bt_conn *conn);
static void pairing_complete(struct bt_conn *conn, bool bonded);
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
static void __attribute__((unused))
security_changed(struct bt_conn *conn, bt_security_t level,
		 enum bt_security_err err);

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated= conn_params_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated,
	COND_CODE_1(CONFIG_BT_SMP, (.security_changed = security_changed), ())
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

static struct k_work on_connect_work;
static struct k_work on_disconnect_work;
static struct bt_conn *current_conn;
static struct nus_entry *nus_commands;

static uint8_t payload_length = 0;
static bool nus_cccd_enabled = false;
static bool conn_param_updated = false;


static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static int update_connection_parameters(void)
{	
	int err;

	err = bt_conn_le_param_update(current_conn, conn_param);
	if (err) {
		LOG_ERR("Cannot update conneciton parameter (err: %d)", err);
		return err;
	}

	LOG_INF("Connection parameters update requested");
	return 0;
}


static void conn_params_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	uint32_t interval_int = interval*1.25;
	LOG_INF("Conn params updated: interval %d ms, latency %d, timeout: %d0 ms",interval_int, latency, timeout);

	if (interval== INTERVAL_MIN) 
	{
		conn_param_updated = true;
	}
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)\n", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
	

}


static void MTU_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done. "); 
		payload_length = bt_gatt_get_mtu(current_conn)-3; //3 bytes ATT header
		LOG_WRN("MTU payload allowed %d", payload_length);
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}


static void request_mtu_exchange(void)
{	
	int err;
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = MTU_exchange_cb;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	} else {
		LOG_INF("MTU exchange pending");
	}	

}

static void request_data_len_update(void)
{
	int err;
	err = bt_conn_le_data_len_update(current_conn, BT_LE_DATA_LEN_PARAM_MAX);
	if (err) {
		LOG_ERR("LE data length update request failed: %d",  err);
	}
}


static void request_phy_update(void)
{
	int err;

	err = bt_conn_le_phy_update(current_conn, BT_CONN_LE_PHY_PARAM_2M);
		if (err) {
			LOG_ERR("Phy update request failed: %d",  err);
		}
}


static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s\n",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	LOG_INF("Connected");
	current_conn = bt_conn_ref(conn);
	
	// Delays added to avoid collision (in case the central also send request)
	// should be better with a state machine. 
	conn_param_updated = false;
	update_connection_parameters();
	k_sleep(K_MSEC(500));
	request_mtu_exchange();
	k_sleep(K_MSEC(500));
	request_data_len_update();
	k_sleep(K_MSEC(500));
	request_phy_update();

	k_work_submit(&on_connect_work);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		
		k_work_submit(&on_disconnect_work);
	}
	
	conn_param_updated = false;
}

static char *ble_addr(struct bt_conn *conn)
{
	static char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	return addr;
}

static void __attribute__((unused))
security_changed(struct bt_conn *conn, bt_security_t level,
		 enum bt_security_err err)
{
	char *addr = ble_addr(conn);

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_INF("Security failed: %s level %u err %d", addr,
			level, err);
	}
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char *addr = ble_addr(conn);

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char *addr = ble_addr(conn);

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char *addr = ble_addr(conn);

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char *addr = ble_addr(conn);

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static int ble_utils_init(struct bt_nus_cb *nus_clbs,
			  nus_connection_cb_t on_connect,
			  nus_disconnection_cb_t on_disconnect)
{
	int ret;
	
	struct bt_le_adv_param *adv_param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE |
				BT_LE_ADV_OPT_USE_IDENTITY,
				BT_GAP_ADV_FAST_INT_MIN_2,
				BT_GAP_ADV_FAST_INT_MAX_2,
				NULL);
	
	k_work_init(&on_connect_work, on_connect);
	k_work_init(&on_disconnect_work, on_disconnect);

	bt_conn_cb_register(&conn_callbacks);

	if (IS_ENABLED(CONFIG_BT_SMP)) {
		ret = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (ret) {
			LOG_ERR("Failed to register authorization callbacks.");
			goto end;
		}

		ret = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (ret) {
			LOG_ERR("Failed to register authorization info callbacks.");
			goto end;
		}
	}

	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("Bluetooth initialization failed (error: %d)", ret);
		goto end;
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	ret = bt_nus_init(nus_clbs);
	if (ret) {
		LOG_ERR("Failed to initialize UART service (error: %d)", ret);
		goto end;
	}

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (ret) {
		LOG_ERR("Advertising failed to start (error: %d)", ret);
		goto end;
	}

end:
	return ret;
}

static void nus_command_handler(struct bt_conn *conn,
				const uint8_t *const data, uint16_t length)
{
	LOG_HEXDUMP_DBG(data, length, "NUS recv:");

	for (struct nus_entry *p = nus_commands; p->cmd != NULL; p++) {
		if ((length >= strlen(p->cmd)) &&
		    (!strncmp(p->cmd, (const char *)data, strlen(p->cmd)))) {
			LOG_INF("Matched cmd : %s", p->cmd);
			strcpy(p->cmd_data, (const char *)&data[strlen(p->cmd)]);
			LOG_INF("command data: %s", p->cmd_data);
			k_work_submit(&p->work);
			return;
		}
	}

	LOG_HEXDUMP_WRN(data, length, "Command unrecognized");
}

static void bt_send_enabled_cb(enum bt_nus_send_status status)
{
	if (status == BT_NUS_SEND_STATUS_ENABLED) 
	{
		nus_cccd_enabled = true;
		LOG_INF("CCCD enabled");
	}
	else{
		nus_cccd_enabled = false;
		LOG_INF("CCCD Disabled");
	}

}

int nus_cmd_init(nus_connection_cb_t on_connect,
		  nus_disconnection_cb_t on_disconnect,
		  struct nus_entry *command_set)
{
	int ret;
	
	struct bt_nus_cb nus_clbs = {
		.received = nus_command_handler,
		.send_enabled = bt_send_enabled_cb,
		.sent = NULL,
	};

	nus_commands = command_set;
	ret = ble_utils_init(&nus_clbs, on_connect, on_disconnect);;
	return ret;
}

int ble_send(uint8_t *buf, uint16_t len)
{
	int ret = 0;
	
	if((buf == NULL)  || (len == 0)) {
		LOG_ERR("Invalid buffer or buffer length");
		ret = -EINVAL;
		goto end;
	}
	
	if(nus_cccd_enabled) {
		
		if(conn_param_updated) {
			
			if(len > payload_length) {
				LOG_WRN("BLE data length %d exceeds allowed mtu length %d", len, payload_length);
				goto end;
			} else {
				LOG_WRN("BLE data length %d", len);
			}
			
			ret = bt_nus_send(NULL, buf, len);
			if (ret) {
				LOG_ERR("Failed to send data over BLE connection(error: %d)", ret);
				goto end;
			}
		} else {
			LOG_WRN("NUS Connection parameter not updated");
		}
		
	} else {
		LOG_WRN("NUS CCCD disabled");
	}
end:
	return ret;
}