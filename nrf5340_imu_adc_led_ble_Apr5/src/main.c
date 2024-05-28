/*
 * Copyright (c) 2024
 */

/** @file
 *  @brief Application does the following
 *         - detect IMU sensor events, 
 *         - sample and process analog signals
 *         - control leds on board via BLE commands
 *         - configure imu on board via BLE commands
 *         - Send imu sensor events with raw data, analog signals raw data, led status 
 */


#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <dk_buttons_and_leds.h>

#include "ble_nus_cmd.h"
#include "imu_sensor.h"
#include "analog_sensor.h"


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_led_ble, CONFIG_IMU_LED_BLE_LOG_LEVEL);

// Set it to 1 to test without IMU sensor
#define IMU_SIMULATION             0

#if IMU_SIMULATION
#include <zephyr/random/rand32.h>
#endif


/* Private macro -------------------------------------------------------------*/
#if IMU_SIMULATION
/* IMU sensor interrupts mask */
#define IMU_SENSOR_INT1            DK_BTN1_MSK
#define IMU_SENSOR_INT2            DK_BTN2_MSK
#else
	
/*
 * Get interrupt configuration from the devicetree alias. This is mandatory.
 */
#define FIFO_INT	DT_ALIAS(fifo_int)
#if !DT_NODE_HAS_STATUS(FIFO_INT, okay)
#error "Unsupported board: fifo-int devicetree alias is not defined"
#endif

#define TAP_INT	DT_ALIAS(tap_int)
#if !DT_NODE_HAS_STATUS(TAP_INT, okay)
#error "Unsupported board: tap-int devicetree alias is not defined"
#endif

static const struct gpio_dt_spec fifo_int = GPIO_DT_SPEC_GET_OR(FIFO_INT, gpios, {0});
static struct gpio_callback fifo_int_cb_data;

static const struct gpio_dt_spec tap_int = GPIO_DT_SPEC_GET_OR(TAP_INT, gpios, {0});
static struct gpio_callback tap_int_cb_data;

#endif


// Analog signals are sampled at 100 Hz, which would be 10 milli seconds
#define ANALOG_SAMPLE_INTERVAL         10 // change it to 10 after IMU tests

/* command that will turn on/off led. 
 * command syntax --> led:1111 will turn on all leds
 * command syntax --> led:0000 will turn off all leds 
 * command syntax --> led:0101 will turn on leds 1 and 3 and turn off leds 2 and 4
 * command syntax --> led:1010 will turn on leds 2 and 4 and turn off leds 1 and 3
*/
#define COMMAND_LED               "led:"
#define LED_CMD_LEN                4

/* command that will configure tap. */
#define COMMAND_TAP               "tap:"
#define TAP_CMD_LEN                16

/* Private variables ---------------------------------------------------------*/
static bool led_state[LED_CMD_LEN];

static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;

static uint16_t adc_raw[4];


static uint8_t tx_ble_buffer[CONFIG_BT_L2CAP_TX_MTU];

static uint8_t led_cmd_data[LED_CMD_LEN];
static uint8_t tap_cmd_data[TAP_CMD_LEN];

static void analog_sample_work_handler(struct k_work *work);
static void imu_tap_detected_work_handler(struct k_work *work);
static void imu_data_ready_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(analog_sample_work, analog_sample_work_handler);
static K_WORK_DEFINE(imu_tap_detected_work, imu_tap_detected_work_handler);
static K_WORK_DEFINE(imu_data_ready_work, imu_data_ready_work_handler);

void error(void)
{
	__ASSERT_NO_MSG(NULL);
}



static void led_cmd(struct k_work *item)
{
	int ret;
	struct nus_entry *nus_cmd = CONTAINER_OF(item, struct nus_entry, work);
	
	memset(tx_ble_buffer, 0, sizeof(tx_ble_buffer));  
	
	if(strlen(nus_cmd->cmd_data) != LED_CMD_LEN) {
		
		sprintf((char *)tx_ble_buffer, "Invalid command length\r\n");
		goto tx_ble;	
		
	} else {
		
		if(((nus_cmd->cmd_data[0] != '0') && (nus_cmd->cmd_data[0] != '1')) ||
		   ((nus_cmd->cmd_data[1] != '0') && (nus_cmd->cmd_data[1] != '1')) ||
		   ((nus_cmd->cmd_data[2] != '0') && (nus_cmd->cmd_data[2] != '1')) ||
		   ((nus_cmd->cmd_data[3] != '0') && (nus_cmd->cmd_data[3] != '1'))) {
			   
		    sprintf((char *)tx_ble_buffer, "Invalid command argument\r\n");
		    goto tx_ble;
			
		} else {
			
			for(uint8_t idx = 0; idx < LED_CMD_LEN; idx++) {
				
				if((nus_cmd->cmd_data[idx] == '0')) {
					ret = dk_set_led_off(idx);
					if(ret) {
						LOG_ERR("Cannot turn off led%d (err: %d)", idx, ret);
						break;
					} else {
						led_state[idx] = false;	
					}
				} else {
					ret = dk_set_led_on(idx);
					if(ret) {
						LOG_ERR("Cannot turn on led%d (err: %d)", idx, ret);
						break;
					} else {
						led_state[idx] = true;	
					}
				}
				
			}
			
			if(ret == 0) {
				sprintf((char *)tx_ble_buffer, "LED command success\r\n");
			} else {
				sprintf((char *)tx_ble_buffer, "LED command failure\r\n");
			}
		    		
		}
	}
				
	
tx_ble:
	LOG_INF("BLE Tx data : %s", tx_ble_buffer);	
		
	ret = ble_send(tx_ble_buffer, strlen(tx_ble_buffer));
	if (ret) {
		LOG_ERR("Cannot send led command status via BLE");	
	}		
		
}



static void tap_cmd(struct k_work *item)
{
	ARG_UNUSED(item);
	
}

static struct nus_entry commands[] = {
	NUS_COMMAND(COMMAND_LED, led_cmd_data, led_cmd),	
	NUS_COMMAND(COMMAND_TAP, tap_cmd_data, tap_cmd),
	NUS_COMMAND(NULL, NULL, NULL),
};


static void analog_sample_work_handler(struct k_work *work)
{
	int ret = 0;
	
	
	/* Start ADC */
	ret = analog_read();
	if (ret) {
		LOG_ERR("Cannot read analog signals(err: %d)", ret);		
	} else {
		for(uint8_t i = 0; i < 4; i++) {
			ret = analog_get_raw(i, &adc_raw[i]);
			if (ret) {
				LOG_ERR("Cannot get raw analog values(err: %d)", ret);
				break;
			}
		}
	}
	
	
		
	if(!ret) {		
			
		memset(tx_ble_buffer, 0, sizeof(tx_ble_buffer));  
		sprintf((char *)tx_ble_buffer, "ADC:A1=%d,A2=%d,A3=%d,A4=%d\r\nLED:L1=%d,L2=%d,L3=%d,L4=%d\r\n",
      	adc_raw[0],adc_raw[1],adc_raw[2],adc_raw[3],led_state[0],led_state[1],led_state[2],led_state[3]);
				
		LOG_INF("BLE Tx data : %s", tx_ble_buffer);	
			
		ret = ble_send(tx_ble_buffer, strlen(tx_ble_buffer));	
		if (ret) {
			LOG_ERR("Cannot send ADC/LED data via BLE");	
		}		
	}

	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(ANALOG_SAMPLE_INTERVAL));
}


static void on_nus_connect(struct k_work *item)
{
	ARG_UNUSED(item);	

	
	
#if !(IMU_SIMULATION)

	gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_EDGE_TO_ACTIVE);
	

	/* IMU out of low power mode */
	int ret = imu_set_low_power(false);
	if (ret) {
		LOG_ERR("Cannot get IMU out of low power(err: %d)", ret);		
	}
#endif	
	
	
	/* Schedule analog signal processing */
	k_work_schedule(&analog_sample_work, K_MSEC(ANALOG_SAMPLE_INTERVAL));
		
}

static void on_nus_disconnect(struct k_work *item)
{
	ARG_UNUSED(item);

	
#if !(IMU_SIMULATION)
	/* set IMU in low power mode */
	int ret = imu_set_low_power(true);
	if (ret) {
		LOG_ERR("Cannot put IMU in low power(err: %d)", ret);		
	}
	
	gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_DISABLE);
	
#endif	

	/* cancel analog signal processing */
	k_work_cancel_delayable(&analog_sample_work);

		
}

static void imu_tap_detected_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);
  int ret = 0;
  lsm6dsv16x_all_sources_t status;

  /* cancel analog signal processing */
  k_work_cancel_delayable(&analog_sample_work);

  #if !(IMU_SIMULATION)
  /* Read output only if new xl value is available */
  ret = lsm6dsv16x_all_sources_get(&lsm6dsv16x_ctx, &status);
  if (ret) {
	LOG_ERR("Cannot get interrupt source(err: %d)", ret);	
  } else 
  #else
  static bool test_tap = true;	  
  if(test_tap) {
	status.single_tap = 1;
	status.double_tap = 0;
	test_tap = false;
  } else {
	status.single_tap = 0;  
	status.double_tap = 1;
	test_tap = true;
  }
  #endif
  
  {
	  
	memset(tx_ble_buffer, 0, sizeof(tx_ble_buffer));

	if (status.single_tap) {
		sprintf((char *)tx_ble_buffer, "TAP:Single\r\n");
	}
	if (status.double_tap) {
		sprintf((char *)tx_ble_buffer, "TAP:Double\r\n");
	}
	 	  
	LOG_INF("BLE Tx data : %s", tx_ble_buffer);	
	 	  
	ret = ble_send(tx_ble_buffer, strlen(tx_ble_buffer));
	if (ret) {
		LOG_ERR("Cannot send TAP data via BLE(err: %d)", ret);	
	}
	  
  }
  	  
  #if !(IMU_SIMULATION)  
  gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_EDGE_TO_ACTIVE);
  #endif
  
  /* Schedule analog signal processing */
  k_work_schedule(&analog_sample_work, K_MSEC(0));
  
  
}

static void imu_data_ready_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);
  int ret = 0;
  uint16_t num = 0;
  lsm6dsv16x_fifo_status_t fifo_status;
  bool tx_data = false;
  
  static bool what_the_tap = true;
  lsm6dsv16x_all_sources_t status;
  if(what_the_tap) {
	  what_the_tap = false;
	  lsm6dsv16x_all_sources_get(&lsm6dsv16x_ctx, &status);
  }
  
  
  /* cancel analog signal processing */
  k_work_cancel_delayable(&analog_sample_work);

#if !(IMU_SIMULATION)
  /* Read watermark flag */
  ret = lsm6dsv16x_fifo_status_get(&lsm6dsv16x_ctx, &fifo_status);
  if (ret) {
	LOG_ERR("Cannot get FIFO status(err: %d)", ret);	
  } else 
#else
  fifo_status.fifo_level = 16;
#endif	

  {
	num = fifo_status.fifo_level;
	LOG_INF("FIFO num: %d", num);
	    
	memset(tx_ble_buffer, 0, sizeof(tx_ble_buffer));

	while (num--) {
		lsm6dsv16x_fifo_out_raw_t f_data;

		#if !(IMU_SIMULATION)
		/* Read FIFO sensor value */
		ret = lsm6dsv16x_fifo_out_raw_get(&lsm6dsv16x_ctx, &f_data);
		if (ret) {
			LOG_ERR("Cannot get FIFO raw data for the fifo entry %d (err: %d)", (num + 1), ret);		
		}
		#else
		size_t data_length = 6;
		uint8_t idx = 0;
		
		idx = ((num + 1) % 4);
		if(idx== 0) {
			f_data.tag = LSM6DSV16X_TIMESTAMP_TAG;	
			data_length = 4;
		} else if(idx== 3) {
			f_data.tag = LSM6DSV16X_XL_NC_TAG;
		} else if(idx== 2) {
			f_data.tag = LSM6DSV16X_GY_NC_TAG;
		} else {
			f_data.tag = LSM6DSV16X_SENSORHUB_SLAVE0_TAG;
		}
		
		sys_rand_get(&f_data.data[0], data_length);	
		#endif
			
		datax = (int16_t *)&f_data.data[0];
		datay = (int16_t *)&f_data.data[2];
		dataz = (int16_t *)&f_data.data[4];
		ts = (int32_t *)&f_data.data[0];
		
		LOG_INF("TAG %02x", f_data.tag);  

		switch (f_data.tag) {
		case LSM6DSV16X_XL_NC_TAG:
		  #if 1
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "ACC[mg]:x:%4.2f,y:%4.2f,z:%4.2f,",
				  lsm6dsv16x_from_fs2_to_mg(*datax),
				  lsm6dsv16x_from_fs2_to_mg(*datay),
				  lsm6dsv16x_from_fs2_to_mg(*dataz));    
		  #else
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "ACC:x:%d,y:%d,z:%d,",
				  *datax,*datay,*dataz);  
		  #endif
		  break;
		case LSM6DSV16X_GY_NC_TAG:
		  #if 1
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "GYRO[mdps]:x:%4.2f,y:%4.2f,z:%4.2f,",
				  lsm6dsv16x_from_fs2000_to_mdps(*datax),
				  lsm6dsv16x_from_fs2000_to_mdps(*datay),
				  lsm6dsv16x_from_fs2000_to_mdps(*dataz));	
		  #else
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "GYRO:x:%d,y:%d,z:%d,",
				  *datax,*datay,*dataz); 	  
		  #endif			  
		  break;  
		case LSM6DSV16X_TIMESTAMP_TAG:	  
		  sprintf((char *)tx_ble_buffer, "T[ms]:%d,", *ts);
		  break;
		case LSM6DSV16X_SENSORHUB_SLAVE0_TAG:
		  #if 1
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "LIS2MDL[mGa]:x:%4.2f,y:%4.2f,z:%4.2f\r\n",
				  lis2mdl_from_lsb_to_mgauss(*datax),
				  lis2mdl_from_lsb_to_mgauss(*datay),
				  lis2mdl_from_lsb_to_mgauss(*dataz));	
		  #else
		  sprintf((char *)&tx_ble_buffer[strlen(tx_ble_buffer)], "MAG:x:%d,y:%d,z:%d\r\n",
				  *datax,*datay,*dataz);  		  
		  #endif	

		  tx_data = true;	  
		  break;
	  
		default:
		  LOG_ERR("Invalid TAG %02x", f_data.tag);  
		  break;
		}
		
		if(tx_data) {	
		
			tx_data = false;		
			LOG_INF("BLE Tx data : %s", tx_ble_buffer);				  
		  
			ret = ble_send(tx_ble_buffer, strlen((char const *)tx_ble_buffer));
			if (ret) {
				LOG_ERR("Cannot send IMU data via BLE");	            
			}
						
			memset(tx_ble_buffer, 0, sizeof(tx_ble_buffer));				
		}
			
	}
  }

#if !(IMU_SIMULATION)  
  imu_enable_fifo_mode();  
  gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_EDGE_TO_ACTIVE);
#endif
  
  /* Schedule analog signal processing */
  k_work_schedule(&analog_sample_work, K_MSEC(0));
          
}

#if (IMU_SIMULATION)
void button_interrupt_handler(uint32_t interrupt_state, uint32_t has_changed)
{
	if (IMU_SENSOR_INT1 & has_changed) {
		if (IMU_SENSOR_INT1 & interrupt_state) {
			/* Interrupt 1 low */
			/* Submit IMU data ready work */					
			k_work_submit(&imu_data_ready_work);
		} 		
	}
	
	if (IMU_SENSOR_INT2 & has_changed) {
		if (IMU_SENSOR_INT2 & interrupt_state) {
			/* Interrupt 2 low */			
			/* Submit IMU tap detection work */			
			k_work_submit(&imu_tap_detected_work);
		}
	}
	
}	
#else
static void fifo_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_DISABLE);

	/* Submit IMU data ready work */					
	k_work_submit(&imu_data_ready_work);
}

static void tap_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);


	gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_DISABLE);


	/* Submit IMU tap detection work */			
	k_work_submit(&imu_tap_detected_work);
}	



int imu_interrupts_init(void)
{
	int ret;
	
	/* setup FIFO gpio interrupt (INT1) */
	if (!gpio_is_ready_dt(&fifo_int)) {
		LOG_ERR("Cannot get pointer to fifo_int device");
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&fifo_int, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure fifo_int pin");
		return ret;
	}
	
	ret =  gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure fifo_int");
		return ret;
	}

	gpio_init_callback(&fifo_int_cb_data, fifo_int_callback,  BIT(fifo_int.pin));
	if (gpio_add_callback(fifo_int.port, &fifo_int_cb_data) < 0) {
		LOG_ERR("Could not set FIFO gpio callback");
		return -EIO;
	}
	
	/* setup TAP gpio interrupt (INT2) */
	if (!gpio_is_ready_dt(&tap_int)) {
		LOG_ERR("Cannot get pointer to tap_int device");
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&tap_int, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure tap_int pin");
		return ret;
	}
	
	ret =  gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure tap_int");
		return ret;
	}

	gpio_init_callback(&tap_int_cb_data, tap_int_callback,  BIT(tap_int.pin));
	if (gpio_add_callback(tap_int.port, &tap_int_cb_data) < 0) {
		LOG_ERR("Could not set TAP gpio callback");
		return -EIO;
	}
	
	return ret;
	
}
#endif

int main(void)
{

	int err = 0;

	/* Initialize LEDs */
	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
		error();
	}
	
	#if (IMU_SIMULATION)
	/* Reuse DK button library to handle IMU sensor interrupts */
	err = dk_buttons_init(button_interrupt_handler);	
	#else
	err = imu_interrupts_init();	
	#endif
	if (err) {
		LOG_ERR("Cannot init IMU interrupts (err: %d)", err);
		error();
	}

		

#if !(IMU_SIMULATION)
	/* Initialize IMU sensor */
	err = imu_init();
	if (err) {
		error();
	}
#endif
	
	/* Initialize ADC to sample and process analog signals */
	err = analog_init();
	if (err) {
		error();
	}

	
	
#if !(IMU_SIMULATION)
	/* Set IMU in low power mode */
	err = imu_set_low_power(true);
	if (err) {
		error();
	}

	gpio_pin_interrupt_configure_dt(&fifo_int, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure_dt(&tap_int, GPIO_INT_DISABLE);
#endif

	/* Initialize NUS command service. */
	err = nus_cmd_init(on_nus_connect, on_nus_disconnect, commands);
	if (err) {
		error();
	}

	LOG_INF("Firmware Init status OK");
	
   /* Sleep forver */
	while (1) {
		k_sleep(K_FOREVER);
	}
}

	