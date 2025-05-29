#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/random/random.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define IBEACON_RSSI  0xC8

#define MY_PRIORITY 2
#define MY_STACK_SIZE 1028

struct sensor_data {
    uint8_t sensor_value1;
    uint8_t sensor_value2;
};

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

K_THREAD_STACK_DEFINE(temp_stack,  MY_STACK_SIZE); //thread
static struct k_thread temp_thread_data;

K_THREAD_STACK_DEFINE(adv_stack,  MY_STACK_SIZE); //thread
static struct k_thread adv_thread_data;

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_data), 20, 1); //queue

static const struct device *ultrasonic = DEVICE_DT_GET_ONE(hc_sr04);  //get the node

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
			      NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("iBeacon started\n");
}

void read_sensor(void) {

    struct sensor_value distance;
    struct sensor_data data;

    if (sensor_sample_fetch(ultrasonic) == 0) {
        // printk("hello");
        if (sensor_channel_get(ultrasonic,SENSOR_CHAN_DISTANCE, &distance) == 0){

			// uint32_t value = distance.val2/1000;
			// printf("%d\n", value);

            uint32_t cm = (uint32_t)distance.val1 * 100U
                    + (uint32_t)(distance.val2 / 10000U);

            if (cm > 0xFFFF) {        
                cm = 0xFFFF;
            }
            
            // data.sensor_value1 = cm >> 8;    
            // data.sensor_value2 = cm & 0xFF;

			if (cm < 20) {
				data.sensor_value1 = 1;
                printk("yes\n");
            } else {
				data.sensor_value1 = 0;
                printk("no\n");
            }



            // printk("Distance: %d.%03d m\n",
            //     distance.val1, distance.val2);
            // double range = (float)sensor_value_to_double(&distance); 
            // data.sensor_value1 = distance.val1;
            // data.sensor_value2 = distance.val2;
            k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);
        } 
    }
}


static void temp_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        // printk("hello");
        read_sensor();
        k_msleep(500);  
    }
}


static void adv_thread(void *p1, void *p2, void *p3)

{
    struct sensor_data data;

    while(1) {
        k_msgq_get(&sensor_msgq, &data, K_FOREVER);

        ad[1] = (struct bt_data)BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
            0x4c, 0x00, /* Apple */
            0x02, 0x15, /* iBeacon */
            0x12, 0x34, 0x56, 0x78, /* UUID[15..12] */
            data.sensor_value1, 0x11, /* UUID[11..10] */
            80, 50, /* UUID[9..8] */
            50, 80, /* UUID[7..6] */
            50, 50, 50, 50, 50, 50, /* UUID[5..0] */
            0x44, 0x66, /* Major */
            0x00, 0x00, /* Minor */
            IBEACON_RSSI); /* Calibrated RSSI @ 1m */

        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        // printf("Pressure:%.1f kPa\n", data.sensor_value);
        // printk("hello\n");
        k_msleep(100);
    }
}


int main() {

    k_thread_create(&temp_thread_data, temp_stack, MY_STACK_SIZE,
        temp_thread, NULL, NULL, NULL,
        MY_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&adv_thread_data, adv_stack, MY_STACK_SIZE,
        adv_thread, NULL, NULL, NULL,
        MY_PRIORITY, 0, K_NO_WAIT);

    int err;

    printk("Starting iBeacon Demo\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }
    return 0;
}
