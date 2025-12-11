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
	uint8_t mgc_state;      /* MGC touch + airwheel data */
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
#define MOUSE_X_LOW_IDX		1	/* X position low byte */
#define MOUSE_X_HIGH_IDX	2	/* X position high byte */
#define MOUSE_Y_LOW_IDX		3	/* Y position low byte */
#define MOUSE_Y_HIGH_IDX	4	/* Y position high byte */
#define MOUSE_WHEEL_IDX		5	/* Scroll wheel */
#define MOUSE_REPORT_COUNT	6

/* HID Report Descriptor for absolute positioning digitizer */
static const uint8_t hid_report_desc[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop)
	0x09, 0x02,        // Usage (Mouse)
	0xA1, 0x01,        // Collection (Application)
	0x09, 0x01,        //   Usage (Pointer)
	0xA1, 0x00,        //   Collection (Physical)

	// Button bits (3 buttons)
	0x05, 0x09,        //     Usage Page (Buttons)
	0x19, 0x01,        //     Usage Minimum (Button 1)
	0x29, 0x03,        //     Usage Maximum (Button 3)
	0x15, 0x00,        //     Logical Minimum (0)
	0x25, 0x01,        //     Logical Maximum (1)
	0x95, 0x03,        //     Report Count (3)
	0x75, 0x01,        //     Report Size (1 bit)
	0x81, 0x02,        //     Input (Data, Variable, Absolute)

	// Padding to byte boundary
	0x95, 0x01,        //     Report Count (1)
	0x75, 0x05,        //     Report Size (5 bits)
	0x81, 0x01,        //     Input (Constant) - padding

	// X Position (Absolute, 16-bit, 0-32767)
	0x05, 0x01,        //     Usage Page (Generic Desktop)
	0x09, 0x30,        //     Usage (X)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
	0x75, 0x10,        //     Report Size (16 bits)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data, Variable, Absolute)

	// Y Position (Absolute, 16-bit, 0-32767)
	0x09, 0x31,        //     Usage (Y)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
	0x75, 0x10,        //     Report Size (16 bits)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data, Variable, Absolute)

	// Scroll Wheel (Relative, 8-bit, -127 to +127)
	0x09, 0x38,        //     Usage (Wheel)
	0x15, 0x81,        //     Logical Minimum (-127)
	0x25, 0x7F,        //     Logical Maximum (127)
	0x75, 0x08,        //     Report Size (8 bits)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x06,        //     Input (Data, Variable, Relative)

	0xC0,              //   End Collection (Physical)
	0xC0               // End Collection (Application)
};

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
 * Convert Euler angles to absolute screen position with smoothing.
 *
 * @param roll Roll angle in radians (maps to X position, inverted)
 * @param pitch Pitch angle in radians (maps to Y position)
 * @param abs_x Output: absolute X coordinate (0-32767, screen center = 16383)
 * @param abs_y Output: absolute Y coordinate (0-32767, screen center = 16383)
 */
static void euler_to_absolute_position(float roll, float pitch,
                                       uint16_t *abs_x, uint16_t *abs_y)
{
	const float CENTER_DEADZONE = 0.0f;    /* ±5° in radians (increased for stability) */
	const float MAX_ANGLE = 1.0472f;          /* ±60° in radians */
	const uint16_t SCREEN_CENTER = 16383;     /* Center of 0-32767 range */
	const uint16_t SCREEN_MAX = 32767;
	const float SMOOTHING_FACTOR = 0.5f;      /* 0.0 = no smoothing, 1.0 = max smoothing */

	/* Static variables to store previous smoothed values */
	static float roll_smoothed = 0.0f;
	static float pitch_smoothed = 0.0f;

	/* Exponential smoothing filter to reduce jitter */
	roll_smoothed = roll_smoothed * SMOOTHING_FACTOR + roll * (1.0f - SMOOTHING_FACTOR);
	pitch_smoothed = pitch_smoothed * SMOOTHING_FACTOR + pitch * (1.0f - SMOOTHING_FACTOR);

	/* Apply center deadzone with smooth transition */
	float roll_active, pitch_active;

	if (fabsf(roll_smoothed) < CENTER_DEADZONE) {
		roll_active = 0.0f;
	} else {
		/* Remove deadzone offset for smooth transition */
		roll_active = roll_smoothed - (roll_smoothed > 0 ? CENTER_DEADZONE : -CENTER_DEADZONE);
	}

	if (fabsf(pitch_smoothed) < CENTER_DEADZONE) {
		pitch_active = 0.0f;
	} else {
		/* Remove deadzone offset for smooth transition */
		pitch_active = pitch_smoothed - (pitch_smoothed > 0 ? CENTER_DEADZONE : -CENTER_DEADZONE);
	}

	/* Adjust max angle to account for deadzone offset */
	const float EFFECTIVE_MAX_ANGLE = MAX_ANGLE - CENTER_DEADZONE;

	/* Clamp angles to effective range */
	if (roll_active > EFFECTIVE_MAX_ANGLE) roll_active = EFFECTIVE_MAX_ANGLE;
	if (roll_active < -EFFECTIVE_MAX_ANGLE) roll_active = -EFFECTIVE_MAX_ANGLE;
	if (pitch_active > EFFECTIVE_MAX_ANGLE) pitch_active = EFFECTIVE_MAX_ANGLE;
	if (pitch_active < -EFFECTIVE_MAX_ANGLE) pitch_active = -EFFECTIVE_MAX_ANGLE;

	/* Map angle to screen coordinate
	 * -55° (60° - 5° deadzone) → 0 (left/top edge)
	 *   0° → 16383 (center)
	 * +55° → 32767 (right/bottom edge)
	 */

	/* X position (INVERTED: positive roll → left, negative roll → right) */
	float x_normalized = -roll_active / EFFECTIVE_MAX_ANGLE;  /* -1.0 to +1.0 */
	float x_float = SCREEN_CENTER + (x_normalized * SCREEN_CENTER);

	/* Y position (positive pitch → down, negative pitch → up) */
	float y_normalized = pitch_active / EFFECTIVE_MAX_ANGLE;  /* -1.0 to +1.0 */
	float y_float = SCREEN_CENTER + (y_normalized * SCREEN_CENTER);

	/* Clamp to valid range and convert to uint16_t */
	if (x_float < 0.0f) x_float = 0.0f;
	if (x_float > SCREEN_MAX) x_float = SCREEN_MAX;
	if (y_float < 0.0f) y_float = 0.0f;
	if (y_float > SCREEN_MAX) y_float = SCREEN_MAX;

	*abs_x = (uint16_t)x_float;
	*abs_y = (uint16_t)y_float;
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
static void send_mouse_report(uint16_t abs_x, uint16_t abs_y, uint8_t buttons, int8_t wheel)
{
	uint8_t report[MOUSE_REPORT_COUNT];

	report[MOUSE_BTN_REPORT_IDX] = buttons;
	report[MOUSE_X_LOW_IDX] = abs_x & 0xFF;           /* Low byte */
	report[MOUSE_X_HIGH_IDX] = (abs_x >> 8) & 0x7F;   /* High byte (15-bit) */
	report[MOUSE_Y_LOW_IDX] = abs_y & 0xFF;           /* Low byte */
	report[MOUSE_Y_HIGH_IDX] = (abs_y >> 8) & 0x7F;   /* High byte (15-bit) */
	report[MOUSE_WHEEL_IDX] = (uint8_t)wheel;         /* Wheel (signed) */

	if (k_msgq_put(&mouse_msgq, report, K_NO_WAIT) != 0) {
		LOG_WRN("Failed to queue mouse report");
	}
}

/* MGC state bit field definitions */
#define MGC_TOUCH_N         (1 << 0)  /* North electrode */
#define MGC_TOUCH_S         (1 << 1)  /* South electrode */
#define MGC_TOUCH_E         (1 << 2)  /* East electrode */
#define MGC_TOUCH_W         (1 << 3)  /* West electrode */
#define MGC_AIRWHEEL_ACTIVE (1 << 4)  /* Airwheel gesture active */
#define MGC_AIRWHEEL_DIR    (1 << 5)  /* Direction: 1=CW, 0=CCW */
#define MGC_AIRWHEEL_VEL_MASK 0xC0    /* Bits 6-7: velocity level */
#define MGC_AIRWHEEL_VEL_SHIFT 6

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

			/* Convert Euler angles to absolute screen position */
			uint16_t abs_x, abs_y;
			euler_to_absolute_position(roll, pitch, &abs_x, &abs_y);
			LOG_DBG("Absolute position: X=%u, Y=%u", abs_x, abs_y);

			leds_update(sensor_data->btn_state);

			/* Parse MGC state */
			uint8_t mgc = sensor_data->mgc_state;
			uint8_t airwheel_active = mgc & MGC_AIRWHEEL_ACTIVE;
			uint8_t airwheel_cw = mgc & MGC_AIRWHEEL_DIR;
			uint8_t airwheel_vel = (mgc & MGC_AIRWHEEL_VEL_MASK) >> MGC_AIRWHEEL_VEL_SHIFT;

			LOG_DBG("MGC state: 0x%02x (N=%d S=%d E=%d W=%d AW_active=%d dir=%s vel=%d)",
				mgc,
				(mgc & MGC_TOUCH_N)?1:0,
				(mgc & MGC_TOUCH_S)?1:0,
				(mgc & MGC_TOUCH_E)?1:0,
				(mgc & MGC_TOUCH_W)?1:0,
				airwheel_active?1:0,
				airwheel_cw?"CW":"CCW",
				airwheel_vel);

			/* Calculate scroll wheel value */
			int8_t wheel = 0;

			/* Touch gesture scrolling - edge-triggered */
			static uint8_t prev_touch_state = 0;
			uint8_t touch_state = mgc & 0x0F;  /* Lower 4 bits = touch electrodes */
			uint8_t touch_edge = touch_state & ~prev_touch_state;  /* Rising edge */
			uint8_t touch_n_edge = touch_edge & MGC_TOUCH_N;
			uint8_t touch_s_edge = touch_edge & MGC_TOUCH_S;

			if (touch_edge & MGC_TOUCH_E) {
				wheel = 10;  /* Scroll up */
				LOG_INF("Touch East → Scroll up (wheel=%d)", wheel);
			}
			if (touch_edge & MGC_TOUCH_W) {
				wheel = -10;  /* Scroll down */
				LOG_INF("Touch West → Scroll down (wheel=%d)", wheel);
			}

			prev_touch_state = touch_state;

			/* Airwheel scrolling - continuous */
			if (airwheel_active) {
				const int8_t vel_scale[] = {0, 5, 10, 20};
				int8_t scroll_amount = vel_scale[airwheel_vel];

				if (airwheel_cw) {
					wheel += scroll_amount;  /* CW = scroll up */
					LOG_INF("Airwheel CW: vel=%d → scroll up (wheel=%d)", airwheel_vel, wheel);
				} else {
					wheel -= scroll_amount;  /* CCW = scroll down */
					LOG_INF("Airwheel CCW: vel=%d → scroll down (wheel=%d)", airwheel_vel, wheel);
				}
			}

			/* Clamp wheel to valid range */
			if (wheel > 127) wheel = 127;
			if (wheel < -127) wheel = -127;

			/* Build HID mouse button byte with OR logic (touch OR physical buttons):
			 * bit 0 = left button (btn1 OR touch N)
			 * bit 1 = right button (btn2 OR touch S)
			 * bit 2 = middle button (btn3)
			 */
			uint8_t mouse_buttons = 0;

			/* Physical button 1 OR Touch North → Left click */
			if (btn1 || touch_n_edge) {
				mouse_buttons |= 0x01;
				LOG_INF("Left mouse button: PRESSED");
			} else {
				LOG_INF("Left mouse button: RELEASED");
			}

			/* Physical button 2 OR Touch South → Right click */
			if (btn2 || touch_s_edge) {
				mouse_buttons |= 0x02;
				LOG_INF("Right mouse button: PRESSED");
			} else {
				LOG_INF("Right mouse button: RELEASED");
			}

			/* Physical button 3 → Middle click (optional) */
			if (btn3) {
				mouse_buttons |= 0x04;
			}

			/* Send mouse report with absolute positioning and button state */
			send_mouse_report(abs_x, abs_y, mouse_buttons, wheel);
			LOG_INF("Sending mouse report: X=%u, Y=%u, buttons=0x%02x, wheel=%d",
				abs_x, abs_y, mouse_buttons, wheel);
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

		/* Reconstruct 16-bit absolute coordinates for logging */
		uint16_t abs_x = report[MOUSE_X_LOW_IDX] | (report[MOUSE_X_HIGH_IDX] << 8);
		uint16_t abs_y = report[MOUSE_Y_LOW_IDX] | (report[MOUSE_Y_HIGH_IDX] << 8);

		LOG_INF("Sending HID report: btn=%d, abs_x=%u, abs_y=%u",
			report[MOUSE_BTN_REPORT_IDX],
			abs_x,
			abs_y);

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
