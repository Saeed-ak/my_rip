/*
 * Copyright (c) 2024
 */

/**
 * @file imu_sensor.h
 *
 * @brief APIs for IMU Sensor.
 */

#ifndef __IMU_SENSOR_H__
#define __IMU_SENSOR_H__

#include "lsm6dsv16x_reg.h"
#include "lis2mdl_reg.h"



/**@brief Function to initialise IMU sensor.
 *
 * @param[out] 0 if success, error code otherwise.
 */
int imu_init(void);

/**@brief Function to initialise IMU sensor.
 * 
 * @param[in] tap      tap detection.
 * @param[in] tap_ths  tap threshold.
 * @param[in] tap_win  tap window.
 * @param[in] tap_mode tap mode.
 * @param[out] 0 if success, error code otherwise.
 *
 */
int imu_tap_configure(lsm6dsv16x_tap_detection_t tap, 
					  lsm6dsv16x_tap_thresholds_t tap_ths,
					  lsm6dsv16x_tap_time_windows_t tap_win,
					  lsm6dsv16x_tap_mode_t tap_mode);


/**@brief Function to put IMU sensor in low power mode.
 *
 * @param[in] active    true = low power mode, false otherwise.
 * @param[out] 0 if success, error code otherwise.
 */

int imu_set_low_power(bool active);

int imu_enable_fifo_mode(void);

extern stmdev_ctx_t lsm6dsv16x_ctx;

#endif
