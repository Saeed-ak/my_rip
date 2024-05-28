/*
 * Copyright (c) 2024
 */
 
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>


#include "imu_sensor.h"


#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME imu_sensor
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME            10 //ms
#define FIFO_WATERMARK       16

#define IMU_DEV_RETRY        50
#define I2C_INST  DT_NODELABEL(i2c1)


/* Private variables ---------------------------------------------------------*/

const struct device *const i2c_sensor = DEVICE_DT_GET(I2C_INST);

stmdev_ctx_t lsm6dsv16x_ctx;
static stmdev_ctx_t lis2mdl_ctx;

// SA0 = 0, ((0xD5 >> 1) & 0x7F) = 0x6A
static uint16_t lsm_i2c_addr = ((LSM6DSV16X_I2C_ADD_L >> 1) & 0x7F);


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	int ret;
	
	ARG_UNUSED(handle);		
	
	uint8_t tx_buf[8];
	
	tx_buf[0] = reg;
	
	for(uint8_t i = 0; i < len; i++) {
		tx_buf[i+1] = bufp[i];
	}

	ret = i2c_write(i2c_sensor, tx_buf, (len + 1), lsm_i2c_addr);
	if (ret != 0) {
		LOG_ERR("i2c_write error : %d", ret);
	} 
	
	return ret;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */							  
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int ret;
	
	ARG_UNUSED(handle);		 
	
	ret = i2c_write_read(i2c_sensor, lsm_i2c_addr, &reg, sizeof(reg), bufp, len);
	if (ret != 0) {
		LOG_ERR("i2c_write_read error : %d", ret);
	} 
	
	return ret;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static int32_t platform_init(void)
{
	int ret = 0;
	
	if (!device_is_ready(i2c_sensor)) {
		LOG_ERR("i2c is not ready!");
		ret = -ENODEV;
	} 
	
	return ret;
}
	

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */	
static void platform_delay(uint32_t ms)
{
	k_sleep(K_MSEC(ms));
}


static int32_t lsm6dsv16x_write_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                          const uint8_t *data, uint16_t len)
{
  int16_t raw_xl[3];
  int32_t ret;
  uint8_t retry_count = 0;
  lsm6dsv16x_data_ready_t drdy;
  lsm6dsv16x_status_master_t master_status;
  lsm6dsv16x_sh_cfg_write_t sh_cfg_write;

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  
  ret = lsm6dsv16x_sh_cfg_write(&lsm6dsv16x_ctx, &sh_cfg_write);
  if (ret) {
	LOG_ERR("Cannot write sh cfg(err: %d)", ret);
	goto error;
  }
	
  /* Disable accelerometer. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  if (ret) {
	LOG_ERR("Cannot disable accelerometer(err: %d)", ret);
	goto error;
  }
  
    
  /* Enable I2C Master. */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  if (ret) {
	LOG_ERR("Cannot enable sh master i2c(err: %d)", ret);
	goto error;
  }
    
  
  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_480Hz);
  if (ret) {
	LOG_ERR("Cannot set xl data rate(err: %d)", ret);
	goto error;
  }
  
  /* Wait Sensor Hub operation flag set. */
  ret = lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);
  if (ret) {
	LOG_ERR("Cannot get raw accelerometer(err: %d)", ret);
	goto error;
  }
  
  
  do {
		platform_delay(20);
    
		ret = lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
		if (ret) {
			LOG_ERR("Cannot get drdy flag(err: %d)", ret);
			goto error;
		} else {
				
			if(!drdy.drdy_xl) {
				LOG_INF("xl drdy flag not set yet");
				retry_count++;
				
				if(retry_count ==  IMU_DEV_RETRY) {
					LOG_ERR("IMU retry count reached");
					ret = -ETIMEDOUT;
					goto error;
				}
			} else {
				LOG_INF("xl drdy flag set");
			}
			
		}	
	
  } while (!drdy.drdy_xl);

  do {
		platform_delay(20);
		
		ret = lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
		if (ret) {
			LOG_ERR("Cannot get sh status(err: %d)", ret);
			goto error;
		} else {
				
			if(!master_status.sens_hub_endop) {
				LOG_INF("sens_hub_endop not set yet");
				retry_count++;
				
				if(retry_count ==  IMU_DEV_RETRY) {
					LOG_ERR("IMU retry count reached");
					ret = -ETIMEDOUT;
					goto error;
				}
			} else {
				LOG_INF("sens_hub_endop set");
			}
		}	
		
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  if (ret) {
	LOG_ERR("Cannot disable sh master i2c(err: %d)", ret);
	goto error;
  }
  
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  if (ret) {
	LOG_ERR("Cannot disable accelerometer(err: %d)", ret);
	goto error;
  }

  
error:	
	return ret;
  
}

/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dsv16x_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len)
{
  return lsm6dsv16x_write_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}


static int32_t lsm6dsv16x_read_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                         uint8_t *data, uint16_t len)
{
  lsm6dsv16x_sh_cfg_read_t sh_cfg_read;
  int16_t raw_xl[3];
  int32_t ret;
  uint8_t retry_count = 0;
  lsm6dsv16x_data_ready_t drdy;
  lsm6dsv16x_status_master_t master_status;

  /* Disable accelerometer. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  if (ret) {
	LOG_ERR("Cannot disable accelerometer(err: %d)", ret);
	goto error;
  }
  
  
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
  if (ret) {
	LOG_ERR("Cannot read sh cfg(err: %d)", ret);
	goto error;
  }
  
  ret = lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);
  if (ret) {
		LOG_ERR("Cannot set sh to read slave (err: %d)", ret);
		goto error;
  }
  
  /* Enable I2C Master. */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  if (ret) {
	LOG_ERR("Cannot enable sh master i2c(err: %d)", ret);
	goto error;
  }
  
  
  /* Enable accelerometer to trigger Sensor Hub operation. */
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_480Hz);
  if (ret) {
	LOG_ERR("Cannot set xl data rate(err: %d)", ret);
	goto error;
  }
  
  
  /* Wait Sensor Hub operation flag set. */
  ret = lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, raw_xl);
  if (ret) {
	LOG_ERR("Cannot get raw accelerometer(err: %d)", ret);
	goto error;
  }
  
  
  
  do {
		platform_delay(20);
    
		ret = lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);
		if (ret) {
			LOG_ERR("Cannot get drdy flag(err: %d)", ret);
			goto error;
		} else {
				
			if(!drdy.drdy_xl) {
				LOG_INF("xl drdy flag not set yet");
				retry_count++;
				
				if(retry_count ==  IMU_DEV_RETRY) {
					LOG_ERR("IMU retry count reached");
					ret = -ETIMEDOUT;
					goto error;
				}
			} else {
				LOG_INF("xl drdy flag set");
			}
			
		}	
	
  } while (!drdy.drdy_xl);

  do {
				
		ret = lsm6dsv16x_sh_status_get(&lsm6dsv16x_ctx, &master_status);
		if (ret) {
			LOG_ERR("Cannot get sh status(err: %d)", ret);
			goto error;
		} else {
				
			if(!master_status.sens_hub_endop) {
				LOG_INF("sens_hub_endop not set yet");
				retry_count++;
				
				if(retry_count ==  IMU_DEV_RETRY) {
					LOG_ERR("IMU retry count reached");
					ret = -ETIMEDOUT;
					goto error;
				}
			} else {
				LOG_INF("sens_hub_endop set");
			}
		}	
		
  } while (!master_status.sens_hub_endop);


  /* Disable I2C master and XL (trigger). */
  ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
  if (ret) {
	LOG_ERR("Cannot disable sh master i2c(err: %d)", ret);
	goto error;
  }
  
  ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
  if (ret) {
	LOG_ERR("Cannot disable accelerometer(err: %d)", ret);
	goto error;
  }
    
  /* Read SensorHub registers. */
  ret = lsm6dsv16x_sh_read_data_raw_get(&lsm6dsv16x_ctx, data, len);
  if (ret) {
	LOG_ERR("Cannot get sh raw data(err: %d)", ret);
	goto error;
  }

error:	
	return ret;

}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dsv16x_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len)
{
  return lsm6dsv16x_read_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

static int imu_sensorhub_configure()
{
	int ret;
	uint8_t whoamI;
	lsm6dsv16x_pin_int_route_t pin_int1;
	lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
	lsm6dsv16x_sh_cfg_read_t sh_cfg_read;  
	uint8_t lis2mdl_rst;
	uint8_t retry_count = 0;
	
	/* Initialize lis2mdl driver interface */
    lis2mdl_ctx.read_reg = lsm6dsv16x_read_lis2mdl_cx;
    lis2mdl_ctx.write_reg = lsm6dsv16x_write_lis2mdl_cx;
    
	/*
     * Configure LIS2MDL target.
    */

    /* Check if LIS2MDL connected to Sensor Hub. */
    ret = lis2mdl_device_id_get(&lis2mdl_ctx, &whoamI);
	if (ret != 0) {
		LOG_ERR("Failed to get lis2mdl device ID");
		goto error;
	} else {

		LOG_INF("LIS2MDL Dev ID : %d", whoamI);
		
		if (whoamI != LIS2MDL_ID) {
			LOG_ERR("LIS2MDL_ID mismatch!");
			ret = -EINVAL;
			goto error;
		}
	}
	
    
    /* Reset */
    ret = lis2mdl_reset_set(&lis2mdl_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Failed to restore device (err: %d)", ret);
		goto error;
	} else {	
	
		k_sleep(K_USEC(5));
		
		/* Wait till the device is ready */
		do {
			ret = lis2mdl_reset_get(&lis2mdl_ctx, &lis2mdl_rst);
			if (ret) {
				LOG_ERR("Failed to get device status(err: %d)", ret);
				goto error;
			} else {
				
				if(lis2mdl_rst) {
					LOG_INF("LIS2MDL device is not ready yet");
					retry_count++;
					if(retry_count ==  IMU_DEV_RETRY) {
						LOG_ERR("IMU retry count reached");
						ret = -ETIMEDOUT;
						goto error;
					}
				} else {
					LOG_INF("LIS2MDL device is ready");
				}
			}
			
		} while (lis2mdl_rst);
	}
	
	

	ret = lis2mdl_block_data_update_set(&lis2mdl_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot update BDU(err: %d)", ret);
		goto error;
	}
	
	ret = lis2mdl_offset_temp_comp_set(&lis2mdl_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set temp comp (err: %d)", ret);
		goto error;
	}


	ret = lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_CONTINUOUS_MODE);
	if (ret) {
		LOG_ERR("Cannot set mag operating mode (err: %d)", ret);
		goto error;
	}
	
	
	ret = lis2mdl_data_rate_set(&lis2mdl_ctx, LIS2MDL_ODR_100Hz);
	if (ret) {
		LOG_ERR("Cannot set MG data rate(err: %d)", ret);
		goto error;
	}
		

	/*
     * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to FIFO_WATERMARK samples
    */
	ret = lsm6dsv16x_fifo_watermark_set(&lsm6dsv16x_ctx, FIFO_WATERMARK);
	if (ret) {
		LOG_ERR("Cannot set FIFO watermark (err: %d)", ret);
		goto error;
	}
	
	
	ret = lsm6dsv16x_fifo_stop_on_wtm_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set stop on FIFO watermark (err: %d)", ret);
		goto error;
	}

	/* Set FIFO batch XL/Gyro ODR to 240Hz */
	ret = lsm6dsv16x_fifo_xl_batch_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_BATCHED_AT_480Hz);
	if (ret) {
		LOG_ERR("Cannot set fifo xl bacth (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_fifo_gy_batch_set(&lsm6dsv16x_ctx, LSM6DSV16X_GY_BATCHED_AT_480Hz);
	if (ret) {
		LOG_ERR("Cannot set fifo gy bacth (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_fifo_timestamp_batch_set(&lsm6dsv16x_ctx, LSM6DSV16X_TMSTMP_DEC_1);
	if (ret) {
		LOG_ERR("Cannot set fifo timestamp batch (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_timestamp_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set timestamp (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_BYPASS_MODE);
	if (ret) {
		LOG_ERR("Cannot set fifo mode (err: %d)", ret);
		goto error;
	}


	#if 1
	/* Configure filtering chain */
	filt_settling_mask.drdy = PROPERTY_ENABLE;
	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	filt_settling_mask.irq_g = PROPERTY_ENABLE;
	ret = lsm6dsv16x_filt_settling_mask_set(&lsm6dsv16x_ctx, filt_settling_mask);
	if (ret) {
		LOG_ERR("Cannot set filter settings mask (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_filt_xl_lp2_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set filter xl lp2 (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_filt_xl_lp2_bandwidth_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_STRONG);
	if (ret) {
		LOG_ERR("Cannot set filter xl lp2 bandwidth(err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_filt_gy_lp1_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set filter gy lp1 (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_filt_gy_lp1_bandwidth_set(&lsm6dsv16x_ctx, LSM6DSV16X_GY_STRONG);
	if (ret) {
		LOG_ERR("Cannot set filter gy lp1 bandwidth(err: %d)", ret);
		goto error;
	}
	#endif

	ret = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
	if (ret) {
		LOG_ERR("Cannot turn off XL data rate (err: %d)", ret);
		goto error;
	}
	
	/*
	* Prepare sensor hub to read data from external slave0 (lis2mdl) 
	* in order to store data in FIFO.
	*/
	sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
	sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
	sh_cfg_read.slv_len = 6;
	ret = lsm6dsv16x_sh_slv_cfg_read(&lsm6dsv16x_ctx, 0, &sh_cfg_read);
	if (ret) {
		LOG_ERR("Cannot read sh slv cfg (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_fifo_sh_batch_slave_set(&lsm6dsv16x_ctx, 0, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot set sh batch slave (err: %d)", ret);
		goto error;
	}

	
	/* Configure Sensor Hub data rate */
	ret = lsm6dsv16x_sh_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_SH_480Hz);
	if (ret) {
		LOG_ERR("Cannot set sh data rate (err: %d)", ret);
		goto error;
	}

	/* Configure Sensor Hub to read one slave. */
	ret = lsm6dsv16x_sh_slave_connected_set(&lsm6dsv16x_ctx, LSM6DSV16X_SLV_0);
	if (ret) {
		LOG_ERR("Cannot set sh to read slave (err: %d)", ret);
		goto error;
	}

	/* Enable I2C Master. */
	ret = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot enable sh i2c master (err: %d)", ret);
		goto error;
	}
	
	pin_int1.fifo_full = PROPERTY_ENABLE;   		
	ret = lsm6dsv16x_pin_int1_route_set(&lsm6dsv16x_ctx, &pin_int1);
	if (ret) {
		LOG_ERR("Cannot route Fifo Full on interrupt1 (err: %d)", ret);
		goto error;
	}
	
error:	
	return ret;
	
}
	
int imu_tap_configure(lsm6dsv16x_tap_detection_t tap, 
					  lsm6dsv16x_tap_thresholds_t tap_ths,
					  lsm6dsv16x_tap_time_windows_t tap_win,
					  lsm6dsv16x_tap_mode_t tap_mode)
{
	int ret;
	lsm6dsv16x_pin_int_route_t pin_int2;
				
    ret = lsm6dsv16x_tap_detection_set(&lsm6dsv16x_ctx, tap);
	if (ret) {
		LOG_ERR("Cannot set tap detection (err: %d)", ret);
		goto error;
	}

	ret = lsm6dsv16x_tap_thresholds_set(&lsm6dsv16x_ctx, tap_ths);
	if (ret) {
		LOG_ERR("Cannot set tap threshold (err: %d)", ret);
		goto error;
	}

	ret = lsm6dsv16x_tap_time_windows_set(&lsm6dsv16x_ctx, tap_win);
	if (ret) {
		LOG_ERR("Cannot set tap time windows (err: %d)", ret);
		goto error;
	}

    ret = lsm6dsv16x_tap_mode_set(&lsm6dsv16x_ctx, tap_mode);
	if (ret) {
		LOG_ERR("Cannot set tap mode(err: %d)", ret);
		goto error;
	}
	
	if(tap_mode == LSM6DSV16X_BOTH_SINGLE_DOUBLE) {
		pin_int2.double_tap = PROPERTY_ENABLE;
	}
	
	pin_int2.single_tap = PROPERTY_ENABLE;
	ret = lsm6dsv16x_pin_int2_route_set(&lsm6dsv16x_ctx, &pin_int2);
	if (ret) {
		LOG_ERR("Cannot route tap on interrupt2 (err: %d)", ret);
		goto error;
	}
	
	
	

error:	
	return ret;
}
	
int imu_set_low_power(bool active)
{
	int err = 0;
#if CONFIG_PM_DEVICE
	
	lsm6dsv16x_interrupt_mode_t irq;	
	lsm6dsv16x_tap_detection_t tap;
	enum pm_device_action action;

	action = !active ? PM_DEVICE_ACTION_RESUME : PM_DEVICE_ACTION_SUSPEND;

	if(active) {
		
		irq.enable = 0;
		irq.lir = 0;
		err = lsm6dsv16x_interrupt_enable_set(&lsm6dsv16x_ctx, irq);
		if (err) {
			LOG_ERR("Cannot disable IMU interrupts (err: %d)", err);
			goto error;
		}

		/* Set FIFO mode */
		err = lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_BYPASS_MODE);
		if (err) {
			LOG_ERR("Cannot set fifo bypass mode (err: %d)", err);
			goto error;
		}

		tap.tap_z_en = 0;    
		err = lsm6dsv16x_tap_detection_set(&lsm6dsv16x_ctx, tap);
		if (err) {
			LOG_ERR("Cannot disable tap detection (err: %d)", err);
			goto error;
		}

		// err = lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_POWER_DOWN);
		// if (err) {
		// 	LOG_ERR("Cannot power down mag (err: %d)", err);
		// 	goto error;
		// }
		
    	/* Disable I2C master and XL (trigger). */
		err = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_DISABLE);
		if (err) {
		   LOG_ERR("Cannot disable sh master i2c(err: %d)", err);
		   goto error;
		}
		
		// As per App note 7.2.1, MASTER_CONFIG (14h)
		// The following procedure must be implemented:
        // 1. Turn off I²C master by setting MASTER_ON = 0.
        // 2. Wait 300 μs.
        // 3. set the accelerometer/gyroscope in powerdown mode
		k_sleep(K_USEC(300));

		err = lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
		if (err) {
			LOG_ERR("Cannot power down gy(err: %d)", err);
			goto error;
		}

		/* Set Output Data Rate.*/
		err = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_OFF);
		if (err) {
			LOG_ERR("Cannot power down xl(err: %d)", err);
			goto error;
		}
		
		
	}

	err = pm_device_action_run(i2c_sensor, action);
	if ((err < 0) && (err != -EALREADY)) {
		LOG_ERR("Sensor: pm_device_action_run failed: %d", err);
		goto error;
	}

	if(!active) {
		
			
		/* Set Output Data Rate.*/
		err = lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_480Hz);
		if (err) {
			LOG_ERR("Cannot set xl data rate (err: %d)", err);
			goto error;
		}
		
		err = lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_480Hz);
		if (err) {
			LOG_ERR("Cannot set gy data rate (err: %d)", err);
			goto error;
		}
		
		// As per App note section 3.10 Gyroscope turn-on/off time
		// Refer Tables 23, 24 and 25
		k_sleep(K_MSEC(80));

		/* Enable I2C Master. */
		err = lsm6dsv16x_sh_master_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
		if (err) {
			LOG_ERR("Cannot enable sh i2c master (err: %d)", err);
			goto error;
		}

		//err = lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_CONTINUOUS_MODE);
		//if (err) {
		//	LOG_ERR("Cannot set mag operating mode (err: %d)", err);
		//	goto error;
		//}
		
		/* Set FIFO mode */
		err = lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_FIFO_MODE);
		if (err) {
			LOG_ERR("Cannot enable fifo mode (err: %d)", err);
			goto error;
		}

		tap.tap_z_en = 1;    
		err = lsm6dsv16x_tap_detection_set(&lsm6dsv16x_ctx, tap);
		if (err) {
			LOG_ERR("Cannot enable tap detection (err: %d)", err);
			goto error;
		}
		
		irq.enable = 1;
		irq.lir = 0;
		err = lsm6dsv16x_interrupt_enable_set(&lsm6dsv16x_ctx, irq);
		if (err) {
			LOG_ERR("Cannot enable IMU interrupts (err: %d)", err);
			goto error;
		}
	
	}
	
	
#endif

error:
	return err;
}
	

int imu_init(void)
{
	int ret;
	uint8_t whoamI;
	uint8_t retry_count = 0;
	lsm6dsv16x_all_sources_t status;
	lsm6dsv16x_tap_detection_t tap;
    lsm6dsv16x_tap_thresholds_t tap_ths;
    lsm6dsv16x_tap_time_windows_t tap_win;
	lsm6dsv16x_tap_mode_t tap_mode;
	lsm6dsv16x_reset_t rst;

    /* Initialize mems driver interface */
    lsm6dsv16x_ctx.write_reg = platform_write;
    lsm6dsv16x_ctx.read_reg = platform_read;
    lsm6dsv16x_ctx.mdelay = platform_delay;
    
    /* Init platform */
	ret = platform_init();
	if (ret) {
		goto error;
	}
	
    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
	
	/* All registers except 0x01 are different between banks, including the WHO_AM_I
	 * register and the register used for a SW reset.  If the lsm6dsv16x wasn't on the user
	 * bank when it reset, then both the chip id check and the sw reset will fail unless we
	 * set the bank now.
	 */
	if (lsm6dsv16x_mem_bank_set(&lsm6dsv16x_ctx, LSM6DSV16X_MAIN_MEM_BANK) < 0) {
		LOG_ERR("Failed to set user bank");
		return -EIO;
	}

    
	/* Check device ID */
    ret = lsm6dsv16x_device_id_get(&lsm6dsv16x_ctx, &whoamI);	
	if (ret != 0) {
		LOG_ERR("Failed to get lsm6dsv16x device ID");
		goto error;
	} else {

		LOG_INF("LSM6DSV16X Dev ID : %d", whoamI);
		
		if (whoamI != LSM6DSV16X_ID) {
			LOG_ERR("LSM6DSV16X_ID mismatch!");
			ret = -EINVAL;
			goto error;
		}
	}

    
    /* Restore default configuration */
    ret = lsm6dsv16x_reset_set(&lsm6dsv16x_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
	if (ret) {
		LOG_ERR("Failed to restore device (err: %d)", ret);
		goto error;
	} else {	
		/* Wait till the device is ready */
		do {
			ret = lsm6dsv16x_reset_get(&lsm6dsv16x_ctx, &rst);
			if (ret) {
				LOG_ERR("Failed to get device status(err: %d)", ret);
				goto error;
			} else {
				
				if(rst != LSM6DSV16X_READY) {
					LOG_INF("LSM6DSV16X device is not ready yet");
					retry_count++;
					if(retry_count ==  IMU_DEV_RETRY) {
						LOG_ERR("IMU retry count reached");
						ret = -ETIMEDOUT;
						goto error;
					}
				} else {
					LOG_INF("LSM6DSV16X device is ready");
				}
			}
			
		} while (rst != LSM6DSV16X_READY);
	}
	

	/* Enable Block Data Update */
	ret = lsm6dsv16x_block_data_update_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("Cannot update BDU(err: %d)", ret);
		goto error;
	}
	
	
	ret = imu_sensorhub_configure();
	if (ret) {
		LOG_ERR("Cannot configure IMU sensor hub (err: %d)", ret);
		goto error;
	}
	
	tap.tap_x_en = 0;
	tap_ths.x = 0;    
	tap.tap_y_en = 0;
	tap_ths.y = 0;    
	tap.tap_z_en = 0;
	tap_ths.z = 3;    
    tap_win.tap_gap = 7;
    tap_win.shock = 3;
    tap_win.quiet = 3;
	tap_mode = LSM6DSV16X_BOTH_SINGLE_DOUBLE;
    ret = imu_tap_configure(tap, tap_ths, tap_win, tap_mode);
	if (ret) {
		LOG_ERR("Cannot configure IMU tap settings (err: %d)", ret);
		goto error;
	}
	
			
    /* Set full scale */
    ret = lsm6dsv16x_xl_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_2g);
	if (ret) {
		LOG_ERR("Cannot set xl full scale (err: %d)", ret);
		goto error;
	}
	
	ret = lsm6dsv16x_gy_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_2000dps);
	if (ret) {
		LOG_ERR("Cannot set gy full scale (err: %d)", ret);
		goto error;
	}
	
	lsm6dsv16x_all_sources_get(&lsm6dsv16x_ctx, &status);
	
error:	
	return ret;
}

int imu_enable_fifo_mode(void)
{
	int err;
	
	/* Set FIFO bypass mode */
	err = lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_BYPASS_MODE);
	if (err) {
		LOG_ERR("Cannot set fifo mode (err: %d)", err);
		goto error;
	}
		
	/* Set FIFO mode */
	err = lsm6dsv16x_fifo_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_FIFO_MODE);
	if (err) {
		LOG_ERR("Cannot set fifo mode (err: %d)", err);
		goto error;
	}
		
error:	
	return err;
}
		