add_custom_target(hci_rpmsg_shared_property_target)
add_custom_target(CPUNET_shared_property_target)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY KERNEL_HEX_NAME;zephyr.hex)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY ZEPHYR_BINARY_DIR;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY KERNEL_ELF_NAME;zephyr.elf)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY BUILD_BYPRODUCTS;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/zephyr.hex;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/zephyr.elf)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_PARTITIONS;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/partitions_CPUNET.yml)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_REGIONS;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/regions_CPUNET.yml)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_HEADER_FILES;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/include/generated/pm_config.h)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_IMAGES;CPUNET:hci_rpmsg)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_HEX_FILE;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/merged_CPUNET.hex)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOTCONF_FILES;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/pm_CPUNET.config)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_APP_HEX;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/app.hex)
set_property(TARGET hci_rpmsg_shared_property_target APPEND PROPERTY BUILD_BYPRODUCTS;C:/ncs/My_Projects/nrf5340_imu_adc_led_ble_Apr5/nrf5340_imu_adc_led_ble_Apr5/build_1/hci_rpmsg/zephyr/merged_CPUNET.hex)
