/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
#include <nrf_erratas.h>
#if NRF54L_ERRATA_20_PRESENT
#include <hal/nrf_power.h>
#endif /* NRF54L_ERRATA_20_PRESENT */
#if defined(NRF54LM20A_ENGA_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54LM20A_ENGA_XXAA) */

/* USB HID includes */
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

/* Sensor data packet structure */
typedef struct {
	uint8_t btn_state;      /* 3 bits: btn1, btn2, btn3 (1=pressed, 0=released) */
	int16_t quat_w;         /* Quaternion w component (scaled by 32767) */
	int16_t quat_x;         /* Quaternion x component (scaled by 32767) */
	int16_t quat_y;         /* Quaternion y component (scaled by 32767) */
	int16_t quat_z;         /* Quaternion z component (scaled by 32767) */
} __attribute__((packed)) sensor_data_t;

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

/* LED for nrf52840dongle */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool led_on = false;

/* USB HID Mouse definitions */
#define MOUSE_BTN_REPORT_IDX	0
#define MOUSE_X_REPORT_IDX	1
#define MOUSE_Y_REPORT_IDX	2
#define MOUSE_WHEEL_REPORT_IDX	3
#define MOUSE_REPORT_COUNT	4

/* HID Report Descriptor for a simple 3-byte mouse (buttons, X, Y) */
static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static enum usb_dc_status_code usb_status;
static const struct device *hid_dev;

/* Message queue for mouse reports */
K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 8, 4);
static K_SEM_DEFINE(ep_write_sem, 0, 1);

static void leds_update(uint8_t value)
{
	/* Toggle LED to indicate packet reception */
	led_on = !led_on;
	gpio_pin_set_dt(&led, led_on);
}

/**
 * Convert quaternion to Euler angles (roll, pitch, yaw) using ZYX sequence.
 */
static void quat_to_euler(float qw, float qx, float qy, float qz,
                          float *roll, float *pitch, float *yaw)
{
	/* Roll (rotation around X-axis) */
	*roll = atan2f(2.0f * (qy * qz + qw * qx),
	               1.0f - 2.0f * (qx * qx + qy * qy));

	/* Pitch (rotation around Y-axis) - clamp input to prevent domain errors */
	float pitch_input = 2.0f * (qw * qy - qx * qz);
	if (pitch_input > 1.0f) pitch_input = 1.0f;
	if (pitch_input < -1.0f) pitch_input = -1.0f;
	*pitch = asinf(pitch_input);

	/* Yaw (rotation around Z-axis) - calculated but not used for mouse */
	*yaw = atan2f(2.0f * (qx * qy + qw * qz),
	              1.0f - 2.0f * (qy * qy + qz * qz));
}

/**
 * Convert Euler angles to mouse displacement with deadzone and scaling.
 */
static void euler_to_mouse_displacement(float roll, float pitch,
                                        int8_t *mouse_x, int8_t *mouse_y)
{
	const float DEADZONE = 0.05f;      /* ±0.05 radians (~±3 degrees) */
	const float SENSITIVITY = 20.0f;   /* displacement = angle * 20 */

	/* Apply deadzone to roll (X-axis) */
	float roll_filtered = (fabsf(roll) < DEADZONE) ? 0.0f : roll;

	/* Apply deadzone to pitch (Y-axis) */
	float pitch_filtered = (fabsf(pitch) < DEADZONE) ? 0.0f : pitch;

	/* Scale angles to mouse displacement */
	float x_float = roll_filtered * SENSITIVITY;
	float y_float = pitch_filtered * SENSITIVITY;

	/* Clamp to int8_t range (-128 to +127) */
	if (x_float > 127.0f) x_float = 127.0f;
	if (x_float < -128.0f) x_float = -128.0f;
	if (y_float > 127.0f) y_float = 127.0f;
	if (y_float < -128.0f) y_float = -128.0f;

	/* Convert to int8_t (invert X axis) */
	*mouse_x = -(int8_t)x_float;
	*mouse_y = (int8_t)y_float;
}

/* USB HID callback functions */
static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&ep_write_sem);
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
};

/* Function to send mouse movement report */
static void send_mouse_report(int8_t x, int8_t y, uint8_t buttons)
{
	uint8_t report[MOUSE_REPORT_COUNT] = {buttons, x, y, 0};

	if (k_msgq_put(&mouse_msgq, report, K_NO_WAIT) != 0) {
		LOG_WRN("Failed to queue mouse report");
	}
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			/* Check if payload size matches sensor_data_t */
			if (rx_payload.length != sizeof(sensor_data_t)) {
				LOG_WRN("Unexpected payload length: %d (expected %d)",
					rx_payload.length, sizeof(sensor_data_t));
				return;
			}

			/* Parse packet as sensor_data_t */
			sensor_data_t *sensor_data = (sensor_data_t *)rx_payload.data;

			/* Extract button states from btn_state */
			uint8_t btn1 = sensor_data->btn_state & 0x01;  /* bit 0 */
			uint8_t btn2 = (sensor_data->btn_state >> 1) & 0x01;  /* bit 1 */
			uint8_t btn3 = (sensor_data->btn_state >> 2) & 0x01;  /* bit 2 */

			/* Convert quaternion int16_t to normalized float (-1.0 to 1.0) */
			float qw = (float)sensor_data->quat_w / 32767.0f;
			float qx = (float)sensor_data->quat_x / 32767.0f;
			float qy = (float)sensor_data->quat_y / 32767.0f;
			float qz = (float)sensor_data->quat_z / 32767.0f;

			/* Log received quaternion data */
			LOG_INF("Packet received - btn_state: 0x%02x (btn1=%d, btn2=%d, btn3=%d)",
				sensor_data->btn_state, btn1, btn2, btn3);
			LOG_DBG("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f", qw, qx, qy, qz);

			/* Convert quaternion to Euler angles */
			float roll, pitch, yaw;
			quat_to_euler(qw, qx, qy, qz, &roll, &pitch, &yaw);
			LOG_DBG("Euler angles: roll=%.3f rad, pitch=%.3f rad, yaw=%.3f rad",
				roll, pitch, yaw);

			/* Convert Euler angles to mouse displacement */
			int8_t mouse_x, mouse_y;
			euler_to_mouse_displacement(roll, pitch, &mouse_x, &mouse_y);
			LOG_DBG("Mouse displacement: X=%d, Y=%d", mouse_x, mouse_y);

			leds_update(sensor_data->btn_state);

			/* Build HID mouse button byte:
			 * bit 0 = left button (btn1)
			 * bit 1 = right button (btn2)
			 */
			uint8_t mouse_buttons = 0;
			if (btn1) {
				mouse_buttons |= 0x01;  /* Left button pressed */
				LOG_INF("Left mouse button: PRESSED");
			} else {
				LOG_INF("Left mouse button: RELEASED");
			}

			if (btn2) {
				mouse_buttons |= 0x02;  /* Right button pressed */
				LOG_INF("Right mouse button: PRESSED");
			} else {
				LOG_INF("Right mouse button: RELEASED");
			}

			/* Send mouse report with quaternion-based movement and button state */
			send_mouse_report(mouse_x, mouse_y, mouse_buttons);
			LOG_INF("Sending mouse report: X=%d, Y=%d, buttons=0x%02x",
				mouse_x, mouse_y, mouse_buttons);
		} else {
			LOG_ERR("Error while reading rx packet");
		}
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if NRF54L_ERRATA_20_PRESENT
	if (nrf54l_errata_20()) {
		nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
	}
#endif /* NRF54L_ERRATA_20_PRESENT */

#if defined(NRF54LM20A_ENGA_XXAA)
	/* MLTPAN-39 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

	LOG_DBG("HF clock started");
	return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void)
{
	int err;
	int res;
	const struct device *radio_clk_dev =
		DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
	struct onoff_client radio_cli;

	/** Keep radio domain powered all the time to reduce latency. */
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

	sys_notify_init_spinwait(&radio_cli.notify);

	err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

	do {
		err = sys_notify_fetch_result(&radio_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err == -EAGAIN);

	nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
	nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

	LOG_DBG("HF clock started");

	return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_1MBPS;  /* 1Mbps for maximum range */
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;
	config.tx_output_power = ESB_TX_POWER_8DBM;  /* Maximum +8dBm for nRF52840 */
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

int main(void)
{
	int err;

	LOG_INF("Nordic AirPad Sample");

	err = clocks_start();
	if (err) {
		return 0;
	}

	/* Initialize LED for dongle */
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED GPIO device not ready");
		return 0;
	}
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("LED GPIO configuration failed, err %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	LOG_INF("Initialization complete");

	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Write payload, err %d", err);
		return 0;
	}

	LOG_INF("Setting up for packet reception");

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	/* Initialize USB HID */
	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				&ops);

	err = usb_hid_init(hid_dev);
	if (err) {
		LOG_ERR("Failed to initialize USB HID, err %d", err);
		return 0;
	}

	err = usb_enable(status_cb);
	if (err != 0) {
		LOG_ERR("Failed to enable USB, err %d", err);
		return 0;
	}

	LOG_INF("USB HID Mouse initialized - ESB packets will left click mouse");

	/* Main loop: process USB HID mouse reports */
	while (true) {
		uint8_t report[MOUSE_REPORT_COUNT];

		/* Wait for mouse report from ESB event handler */
		err = k_msgq_get(&mouse_msgq, &report, K_FOREVER);
		if (err) {
			LOG_ERR("Failed to get report from queue: %d", err);
			continue;
		}

		LOG_INF("Sending HID report: btn=%d, x=%d, y=%d, wheel=%d",
			report[MOUSE_BTN_REPORT_IDX],
			(int8_t)report[MOUSE_X_REPORT_IDX],
			(int8_t)report[MOUSE_Y_REPORT_IDX],
			(int8_t)report[MOUSE_WHEEL_REPORT_IDX]);

		/* Send HID report over USB */
		err = hid_int_ep_write(hid_dev, report, MOUSE_REPORT_COUNT, NULL);
		if (err) {
			LOG_ERR("HID write error, %d", err);
		} else {
			LOG_DBG("HID write successful, waiting for endpoint ready");
			/* Wait for endpoint to be ready */
			k_sem_take(&ep_write_sem, K_FOREVER);
			LOG_DBG("Endpoint ready");
		}
	}

	return 0;
}
