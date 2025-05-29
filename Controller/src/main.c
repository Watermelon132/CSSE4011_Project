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

// Led Macros
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Button Macros
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#define DEBOUNCE_DELAY_MS 10

bool is_sampling = false;
bool prev_button_state = false;

int sent_gesture = 0;

double sum_x = 0, sum_y = 0, sum_z = 0;
int sample_count = 0;




struct sensor_data {
    uint8_t sensorx_value;
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

const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lis2dh);  //get the node

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

// Function to classify gesture based on x and y
const uint8_t classify_gesture(double x, double y, double z) {
    if (z<-8.31){
        return 1; //Up
    }
    if (y <= -10.56) {
        if (x <= -0.23) {
            return 2; //left
        } else { // x > -0.83
            if (y <= -11.63) {
                return 1; //up
            } else { // y > -11.63
                return 1; //up
            }
        }
    } else { // y > -10.56
        if (x <= -0.22) {
            return 2; //left
        } else { // x > -0.42
            if (y <= -6.27) {
                return 3; //right
            } else { // y > -6.27
                return 4; //down
            }
        }
    }
}

void read_sensor(void)
{
    char out_str[64];
    static struct sensor_value accel_x, accel_y, accel_z;
    struct sensor_data data;

    if (!device_is_ready(lsm6dsl_dev)) {
        printk("sensor: device not ready.\n");
        return;
    }

    bool curr_button_state = gpio_pin_get_dt(&button);
    gpio_pin_set_dt(&led0, curr_button_state ? 1 : 0);

    // Detect rising edge — button just pressed
    if (curr_button_state && !prev_button_state) {
        is_sampling = true;
        sum_x = sum_y = sum_z = 0;
        sample_count = 0;
        //printk("Started sampling...\n");
    }

    // Detect falling edge — button just released
    if (!curr_button_state && prev_button_state && is_sampling) {
        is_sampling = false;
        if (sample_count > 0) {
            double mean_x = sum_x / sample_count;
            double mean_y = sum_y / sample_count;
            double mean_z = sum_z / sample_count;
            // printk("Mean accel: x=%.2f y=%.2f z=%.2f (from %d samples)\n",
            //         mean_x, mean_y, mean_z, sample_count);

            int gesture = classify_gesture(mean_x, mean_y, mean_z);
            printf("Detected gesture: %d\n", gesture);

            sent_gesture = gesture;

            data.sensorx_value = gesture;

            // uint8_t ax_dm = (uint8_t)(mean_x * 10.0);
            // uint8_t ay_dm = (uint8_t)(mean_y * 10.0);
            // uint8_t az_dm = (uint8_t)(mean_z * 10.0);

            // data.sensorx_value  = ax_dm;
            // data.sensory_value  = ay_dm;
            // data.sensorz_value  = az_dm;

            k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);

        } else {
            printk("No samples collected.\n");
        }
    }

    if (is_sampling && curr_button_state) {
        if (sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ) == 0) {
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

            double ax = sensor_value_to_double(&accel_x);
            double ay = sensor_value_to_double(&accel_y);
            double az = sensor_value_to_double(&accel_z);

            sum_x += ax;
            sum_y += ay;
            sum_z += az;
            sample_count++;
        }
    }

    prev_button_state = curr_button_state;

}


static void temp_thread(void *p1, void *p2, void *p3)
{
    struct sensor_value odr_attr;

    odr_attr.val1 = 50;
    odr_attr.val2 = 0;

    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for accelerometer.\n");
        return 0;
    }
    while (1) {
        // printk("hello");
        read_sensor();
        k_msleep(100);  
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
        0x11, 0x22, 0x33, 0x44, /* UUID[15..12] */
        data.sensorx_value, 0x00, /* UUID[11..10] */
        0x00, 50, /* UUID[9..8] */
        50, 80, /* UUID[7..6] */
        50, 50, 50, 50, 50, 50, /* UUID[5..0] */
        0x44, 0x66, /* Major */
        0x00, 0x00, /* Minor */
        IBEACON_RSSI); /* Calibrated RSSI @ 1m */

        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);

        k_msleep(300);

        ad[1] = (struct bt_data)BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
        0x4c, 0x00, /* Apple */
        0x02, 0x15, /* iBeacon */
        0x11, 0x22, 0x33, 0x44, /* UUID[15..12] */
        0x00, 0x00, /* UUID[11..10] */
        0x00, 50, /* UUID[9..8] */
        50, 80, /* UUID[7..6] */
        50, 50, 50, 50, 50, 50, /* UUID[5..0] */
        0x44, 0x66, /* Major */
        0x00, 0x00, /* Minor */
        IBEACON_RSSI);

        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);

        k_msleep(10);
    }
}


int main() {

    int ret;

    // Configure LED
    if (!gpio_is_ready_dt(&led0)) {
        printk("LED not ready\n");
        return 0;
    }
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to configure LED\n");
        return 0;
    }

    // Configure Button
    if (!gpio_is_ready_dt(&button)) {
        printk("Button not ready\n");
        return 0;
    }
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Failed to configure button\n");
        return 0;
    }


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
