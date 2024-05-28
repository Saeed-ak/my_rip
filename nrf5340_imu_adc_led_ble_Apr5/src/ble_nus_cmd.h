/*
 * Copyright (c) 2024
 */

/**
 * @file ble_nus_cmd.h
 *
 * @brief APIs for BLE NUS Command Service.
 */

#ifndef __BLE_NUS_CMD_H__
#define __BLE_NUS_CMD_H__

/** @brief Type indicates function called when Bluetooth LE connection
 *         is established.
 *
 * @param[in] item pointer to work item.
 */
typedef void (*nus_connection_cb_t)(struct k_work *item);

/** @brief Type indicates function called when Bluetooth LE connection is ended.
 *
 * @param[in] item pointer to work item.
 */
typedef void (*nus_disconnection_cb_t)(struct k_work *item);


/** @brief BLE NUS command entry.
 */
struct nus_entry {
	const char *cmd;        /**< Command string. */
	char *cmd_data;         /**< Command data string. */
	struct k_work work;     /**< Command work item. */
};

/**@brief Macro to initialise (bind) command string to work handler.
 */
#define NUS_COMMAND(_cmd, _cmd_data, _handler)		      \
	{					                                  \
		.work = Z_WORK_INITIALIZER(_handler),             \
		.cmd = _cmd,			                          \
		.cmd_data = _cmd_data,			                  \
	}					                                  \

/**@brief Function to initialise NUS Command service.
 *
 * @param[in] on_connect Callback that will be called when central connects to
 *			 the NUS service.
 * @param[in] on_disconnect Callback that will be called when BLE central
 *			    disconnects from the NUS service. 
 * @param[in] command_set   A pointer to an array with NUS commands.
 * @param[out] 0 if success, error code otherwise.
 */
int nus_cmd_init(nus_connection_cb_t on_connect,
		  nus_disconnection_cb_t on_disconnect,
		  struct nus_entry *command_set);


/**@brief Function to send data over BLR NUS service.
 *
 * @param[in] buf data to be transmitted over BLE.
 * @param[in] len length of BLE data.
 * @param[out] 0 if success, error code otherwise.
 */		  
int ble_send(uint8_t *buf, uint16_t len);		  

#endif
