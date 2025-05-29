/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <sample_usbd.h>

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <string.h> 
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define NAME_LEN 30

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
#endif

int playing = 0;


struct kalman {
    double x_value;
    double y_value;
};

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static bool rx_throttled;

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 1
#define DAC_RESOLUTION 12
#endif

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id  = DAC_CHANNEL_ID,
	.resolution  = DAC_RESOLUTION,
	.buffered = true
};


static inline void print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate %u", baudrate);
	}
}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static struct usbd_context *sample_usbd;
K_SEM_DEFINE(dtr_sem, 0, 1);

static void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			k_sem_give(&dtr_sem);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		print_baudrate(msg->dev);
	}
}

static int enable_usb_device_next(void)
{
	int err;

	sample_usbd = sample_usbd_init_device(sample_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		err = usbd_enable(sample_usbd);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");

	return 0;
}
#endif /* defined(CONFIG_USB_DEVICE_STACK_NEXT) */

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	static uint8_t byte_buffer[2];  // to store the 2 incoming bytes per sample
    static int byte_index = 0;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            int len = uart_fifo_read(dev, buf, sizeof(buf));

            for (int i = 0; i < len; i++) {
                byte_buffer[byte_index++] = buf[i];

                if (byte_index == 2) {
                    // Combine high nibble and low byte into a 12-bit sample
                    uint16_t sample = ((uint16_t)(byte_buffer[0] & 0x0F) << 8) | byte_buffer[1];
                    
                    // Output to DAC
                    int ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, sample);
                    if (ret != 0) {
                        printk("DAC write failed: %d\n", ret);
                    }

                    byte_index = 0;  // reset for next sample
                }
            }
        }
    }
}

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
			  0x4c, 0x00, /* Apple */
		      0x02, 0x15, /* iBeacon */
		      0x18, 0xee, 0x15, 0x16, /* UUID[15..12] */
		      0x01, 0x6b, /* UUID[11..10] */
		      0x4b, 0xec, /* UUID[9..8] */
		      0xad, 0x96, /* UUID[7..6] */
		      0xbc, 0xb9, 0x6d, 0x16, 0x6e, 0x97, /* UUID[5..0] */
		      0x00, 0x00, /* Major */
		      0x00, 0x00, /* Minor */
		      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};


// Function to solve 2x2 linear least squares



static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
	struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	// Iterate through AD structures
	struct bt_data data;
	struct net_buf_simple temp_buf = *ad;

	while (temp_buf.len > 1) {
		struct kalman blu_kalman_values;
		struct kalman ult_kalman_values;

		uint8_t len = net_buf_simple_pull_u8(&temp_buf);
		if (len == 0 || len > temp_buf.len) {
			break;
		}
		data.type = net_buf_simple_pull_u8(&temp_buf);
		data.data_len = len - 1;
		data.data = temp_buf.data;
		net_buf_simple_pull(&temp_buf, data.data_len);

		if (data.type == BT_DATA_MANUFACTURER_DATA) {
			
			// Check for ID of mobile node using first 4 bytes of UUID (12345678 in hex)
			if (data.data[4] == 0x12 && data.data[5] == 0x34 && data.data[6] == 0x56 && data.data[7] == 0x78) {	
				
				// UUID starts at data[4]
				char uuid_str[37];
				snprintf(uuid_str, sizeof(uuid_str),
					"%02x%02x%02x%02x-%d-%d-%d-%d-%d-%d-%d-%d",
					data.data[4], data.data[5], data.data[6], data.data[7],		// Identifier
					data.data[8], data.data[9],			// RSSIs
					data.data[10], data.data[11],
					data.data[12], data.data[13],
					data.data[14], data.data[15]);
				// printf("iBeacon UUID: %s\n", uuid_str);



				if(data.data[8] == 1){
					if(playing){
						playing = 0;
						printf("stop\n");
						k_msleep(1000);
					} else{
						playing = 1;
						k_msleep(1000);
						printf("start\n");
					}
				}

				
			} 
			
			// Check for ID of ultrasonic node using first 4 bytes of UUID (11223344 in hex)
			else if (data.data[4] == 0x11 && data.data[5] == 0x22 && data.data[6] == 0x33 && data.data[7] == 0x44) {	
				// Assuming ultrasonic sensor is at H
				// Get ultrasonic data (no need to localise) from UUID
				int x_dist = (int)data.data[8];
				//	Put it in kalman struct
				ult_kalman_values.x_value = x_dist / 10.0; 	// Make it into x_value in meters
				// Assuming that if there's reading, then y is along the sensor's line of sight (y=2)
				ult_kalman_values.y_value = 2.0;
				// printf("%d", data.data[8]);
				// printf("Ult: %.2f\n", ult_kalman_values.x_value);
				// Put into kalman queue
				char uuid_str[37];
				snprintf(uuid_str, sizeof(uuid_str),
					"%02x%02x%02x%02x-%d-%d-%d-%d-%d-%d-%d-%d",
					data.data[4], data.data[5], data.data[6], data.data[7],		// Identifier
					data.data[8], data.data[9],			// RSSIs
					data.data[10], data.data[11],
					data.data[12], data.data[13],
					data.data[14], data.data[15]);
				// printf("iBeacon UUID: %s\n", uuid_str);

				if(data.data[8] == 1){
					printf("up\n");
				} else if(data.data[8] == 2){
					printf("left\n");
				} else if(data.data[8] == 3){
					printf("right\n");
				} else if(data.data[8] == 4){
					printf("down\n");
				}
				// printf("%d\n", data.data[8]); 
			}
		}
	}
}

int observer_start(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Start scanning failed (err %d)\n", err);
		return err;
	}
	//printk("Started scanning...\n");

	return 0;
}

int main(void)
{
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	(void)observer_start();

	int ret2 = dac_channel_setup(dac_dev, &dac_ch_cfg);

	if (ret2 != 0) {
		printk("Setting up of DAC channel failed with code %d\n", ret2);
		return 0;
	}

	int ret;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = enable_usb_device_next();
#else
		ret = usb_enable(NULL);
#endif

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	k_sem_take(&dtr_sem, K_FOREVER);
#else
	while (true) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
#endif

	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

#ifndef CONFIG_USB_DEVICE_STACK_NEXT
	print_baudrate(uart_dev);
#endif
	uart_irq_callback_set(uart_dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(uart_dev);

	if (!device_is_ready(dac_dev)) {
		printk("DAC device %s is not ready\n", dac_dev->name);
		return 0;
	}

	return 0;
}
