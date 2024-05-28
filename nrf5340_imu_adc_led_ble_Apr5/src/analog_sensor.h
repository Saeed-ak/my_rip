/*
 * Copyright (c) 2024
 */

/**
 * @file analog_sensor.h
 *
 * @brief APIs for Analog Sensor.
 */

#ifndef __ANALOG_SENSOR_H__
#define __ANALOG_SENSOR_H__



/**@brief Function to initialise ADC.
 *
 * @param[out] 0 if success, error code otherwise.
 */
int analog_init(void);

/**@brief Function to read Analog signals.
 *
 * @param[out] 0 if success, error code otherwise.
 */
int analog_read(void);

/**@brief Function to read Analog signals.
 *
 * @param[in] channel_idx data to be transmitted over BLE.
 * @param[in] data_raw length of BLE data.
 * @param[out] 0 if success, error code otherwise.
 */

int analog_get_raw(uint16_t channel_idx, uint16_t *data_raw);

#endif
