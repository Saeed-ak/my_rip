/*
 * Copyright (c) 2024
 */
 
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "analog_sensor.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(analog_sensor, CONFIG_ANALOG_SENSOR_LOG_LEVEL);


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static uint16_t adc_raw_channel[ARRAY_SIZE(adc_channels)];

int analog_init(void)
{
	int err;
	
	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC controller device %s not ready", adc_channels[i].dev->name);
			return -ENODEV;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)", i, err);
			return err;
		}
	}

	
	return 0;
}


int analog_read(void)
{
	int err;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};
	
	LOG_INF("ADC reading");
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		

		LOG_INF("- %s, channel %d: ",
			   adc_channels[i].dev->name,
			   adc_channels[i].channel_id);

		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		err = adc_read(adc_channels[i].dev, &sequence);
		if (err < 0) {
			LOG_ERR("Could not read (%d)", err);
			break;
		} else {		
			adc_raw_channel[i] = buf;
			LOG_INF("raw %u", adc_raw_channel[i]);
		}
	}
		
	return err;

}

int analog_get_raw(uint16_t channel_idx, uint16_t *data_raw)
{
	int err = 0; 
	
	if((channel_idx >= 0) && (channel_idx <= 3)) {		
		*data_raw =  adc_raw_channel[channel_idx];
	} else {
		LOG_ERR("Invalid channel index");
		*data_raw = 0;
		err = -EINVAL;
	}
	
	return err;
}