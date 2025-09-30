/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_EI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <modem/lte_lc.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// Nordic modem includes for GPS
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>

// GNSS data (EXACT from working LoRa project)
static struct nrf_modem_gnss_pvt_data_frame current_pvt;
static volatile bool new_pvt_data = false;

// Semaphores and message queue (from working LoRa project)
K_SEM_DEFINE(gnss_data_sem, 0, 1);
K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame*), 10, 4);

// Poll events for GNSS data
static struct k_poll_event events[2] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                     K_POLL_MODE_NOTIFY_ONLY,
                                     &gnss_data_sem, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                     K_POLL_MODE_NOTIFY_ONLY,
                                     &nmea_queue, 0),
};

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

// Forward declaration for Edge Impulse result type
typedef struct ei_impulse_result_t ei_impulse_result_t;
#include "ei_device_nrf91.h"
#include "ei_at_handlers.h"
#include "ei_ws_client.h"
#include "inference/ei_run_impulse.h"
#include "sensors/ei_accelerometer.h"
#include "led_states.h"

const struct device *uart;
static const struct device *uart_dev;  // Use same name as working lora project
static const struct device *gpio_dev;

// LED pins (Thingy:91 X) - corrected mapping: P29=RED, P31=GREEN, P30=BLUE
#define LED_RED_PIN   29  // P0.29 controls RED LED
#define LED_GREEN_PIN 31  // P0.31 controls GREEN LED
#define LED_BLUE_PIN  30  // P0.30 controls BLUE LED

// UART buffer for LoRa-E5
#define UART_BUF_SIZE 512
static uint8_t uart_rx_buf[UART_BUF_SIZE];
static char recv_buf[UART_BUF_SIZE];
static volatile bool at_response_received = false;
static size_t rx_data_len = 0;

// Mutex for UART operations
K_MUTEX_DEFINE(uart_mutex);

// GPS and LoRaWAN variables
static volatile bool gnss_has_fix = false;
static double gnss_latitude = 0.0;
static double gnss_longitude = 0.0;
static float gnss_altitude = 0.0;
static float gnss_accuracy = 0.0;
static bool emergency_mode = false;
static uint64_t last_emergency_time = 0;
static uint64_t last_routine_time __attribute__((unused)) = 0;

// NMEA data handling is now done directly in ISR (no queue needed)

// LoRaWAN credentials - Replace with your actual values from TTN
#define LORAWAN_APPEUI    "0000000000000000"  // 16 hex chars - App EUI from TTN
#define LORAWAN_DEVEUI    "70B3D57ED0071E94"  // 16 hex chars - Device EUI from TTN
#define LORAWAN_APPKEY    "2B7E151628AED2A6ABF7158809CF4F3C"  // 32 hex chars - App Key from TTN

// System capability flags
static bool gnss_available = false;
static bool lora_available = false;
static bool lora_initialized = false;
static bool lora_joined = false;

// Timing constants
#define EMERGENCY_COOLDOWN_MS  30000   // 30 seconds
#define ROUTINE_INTERVAL_MS    300000  // 5 minutes

// Message types
#define MSG_TYPE_ROUTINE_UPDATE   0x01
#define MSG_TYPE_ALERT_RUNNING    0x02

void ei_putchar(char c)
{
    // it is ~30% faster than printf
    uart_poll_out(uart, c);
}

#ifdef CONFIG_REMOTE_INGESTION
void pdn_event_handler(uint8_t cid, enum pdn_event event, int reason)
{
	switch (event) {
	case PDN_EVENT_CNEC_ESM:
		LOG_DBG("PDP context %d error, %s", cid, pdn_esm_strerror(reason));
		break;
	case PDN_EVENT_ACTIVATED:
		LOG_DBG("PDP context %d activated", cid);
		break;
	case PDN_EVENT_DEACTIVATED:
		LOG_DBG("PDP context %d deactivated", cid);
		break;
	case PDN_EVENT_NETWORK_DETACH:
		LOG_DBG("PDP context %d network detached", cid);
		break;
	case PDN_EVENT_IPV6_UP:
		LOG_DBG("PDP context %d IPv6 up", cid);
		break;
	case PDN_EVENT_IPV6_DOWN:
		LOG_DBG("PDP context %d IPv6 down", cid);
		break;
	default:
		LOG_DBG("PDP context %d, unknown event %d", cid, event);
		break;
	}
}

bool modem_connect(void)
{
	int ret;

	LOG_DBG("Modem Lib init...");
	ret = nrf_modem_lib_init();
	if (ret < 0) {
		LOG_ERR("Unable to init modem library (%d)", ret);
		return false;
	}
	LOG_DBG("Modem Lib init... OK");

	/* Setup a callback for the default PDP context (zero).
	 * Do this before switching to function mode 1 (CFUN=1)
	 * to receive the first activation event.
	 */
	LOG_DBG("PDN context init...");
	ret = pdn_default_ctx_cb_reg(pdn_event_handler);
	if (ret) {
		LOG_ERR("pdn_default_ctx_cb_reg() failed, err %d", ret);
		return false;
	}
	LOG_DBG("PDN context init... OK");

	LOG_DBG("Waiting for network...");
	ret = lte_lc_init_and_connect();
	if (ret) {
		LOG_ERR("Failed to connect to the LTE network, err %d", ret);
		return false;
	}
	LOG_DBG("Waiting for network... OK");

	return true;
}
#endif

// LED control functions
void set_led(int pin, bool state)
{
    if (gpio_dev) {
        gpio_pin_set(gpio_dev, pin, state ? 1 : 0);
    }
}

void blink_led(int pin, int count)
{
    for (int i = 0; i < count; i++) {
        set_led(pin, true);
        k_msleep(100);
        set_led(pin, false);
        k_msleep(100);
    }
}

// UART callback for LoRa-E5
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        rx_data_len = evt->data.rx.len;

        if (rx_data_len < UART_BUF_SIZE) {
            memcpy(recv_buf, uart_rx_buf, rx_data_len);
            recv_buf[rx_data_len] = '\0';
        }

        LOG_INF("UART RX (%d bytes): %s", rx_data_len, recv_buf);
        at_response_received = true;
        break;

    case UART_RX_DISABLED:
        uart_rx_enable(dev, uart_rx_buf, sizeof(uart_rx_buf) - 1, 5000);
        break;

    case UART_RX_BUF_RELEASED:
        // Buffer was released, restart reception
        uart_rx_enable(dev, uart_rx_buf, sizeof(uart_rx_buf) - 1, 5000);
        break;

    default:
        break;
    }
}

// AT command send with response check
int at_send_check_response(const char *p_ack, int timeout_ms, const char *p_cmd, ...)
{
    char cmd[128];
    va_list args;
    int ret;
    int timeout_count = 0;
    int max_timeout = timeout_ms / 10;

    k_mutex_lock(&uart_mutex, K_FOREVER);

    memset(recv_buf, 0, sizeof(recv_buf));
    at_response_received = false;

    va_start(args, p_cmd);
    vsnprintf(cmd, sizeof(cmd), p_cmd, args);
    va_end(args);

    LOG_INF("Sending AT command: %s", cmd);

    ret = uart_tx(uart_dev, (uint8_t*)cmd, strlen(cmd), 2000);
    if (ret) {
        LOG_ERR("UART TX failed: %d", ret);
        k_mutex_unlock(&uart_mutex);
        return 0;  // Return 0 for failure like working lora project
    }

    k_msleep(200);  // Add delay like working lora project

    while (timeout_count < max_timeout) {
        if (at_response_received) {
            if (strstr(recv_buf, p_ack)) {
                LOG_INF("ACK received: %s", p_ack);
                k_mutex_unlock(&uart_mutex);
                return 1;  // Return 1 for success like working lora project
            }
            at_response_received = false;
        }
        k_msleep(10);
        timeout_count++;
    }

    LOG_WRN("No ACK received for command: %s", cmd);
    k_mutex_unlock(&uart_mutex);
    return 0;  // Return 0 for failure like working lora project
}

// Initialize LoRa-E5 module (copied from working lora project)
bool init_lora_e5(void)
{
    LOG_INF("Initializing LoRa-E5 module...");
    led_set_state(LED_STATE_LORA_INITIALIZING);

    // Basic AT test (use working lora project pattern)
    if (!at_send_check_response("+AT: OK", 1000, "AT\r\n")) {
        LOG_ERR("LoRa-E5 AT test failed");
        led_set_state(LED_STATE_LORA_INIT_FAILED);
        return false;
    }

    // Configure module for OTAA (use working lora project commands)
    LOG_INF("Configuring LoRa-E5 for OTAA...");

    // Set credentials using defined constants like working project
    at_send_check_response("+ID: AppEui", 1000, "AT+ID=AppEui,%s\r\n", LORAWAN_APPEUI);
    at_send_check_response("+ID: DevEui", 1000, "AT+ID=DevEui,%s\r\n", LORAWAN_DEVEUI);
    at_send_check_response("+KEY: APPKEY", 1000, "AT+KEY=APPKEY,%s\r\n", LORAWAN_APPKEY);

    at_send_check_response("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");
    at_send_check_response("+DR: IN865", 1000, "AT+DR=IN865\r\n");
    at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-2\r\n");
    at_send_check_response("+CLASS: A", 1000, "AT+CLASS=A\r\n");
    at_send_check_response("+PORT: 8", 1000, "AT+PORT=8\r\n");

    LOG_INF("LoRa-E5 initialized successfully");
    led_set_state(LED_STATE_LORA_INIT_SUCCESS);
    lora_initialized = true;
    return true;
}

// LoRa network join function
bool join_lorawan_network(void)
{
    if (!lora_initialized) {
        return false;
    }

    LOG_INF("Attempting to join LoRaWAN network...");
    printk("Attempting to join LoRaWAN network...\n");
    led_set_state(LED_STATE_LORA_JOINING);

    if (at_send_check_response("+JOIN: Network joined", 12000, "AT+JOIN\r\n")) {
        lora_joined = true;
        LOG_INF("Successfully joined LoRaWAN network");
        printk("Successfully joined LoRaWAN network!\n");
        led_set_state(LED_STATE_LORA_JOINED);
        return true;
    } else {
        LOG_WRN("Join failed, will retry...");
        printk("Join failed, will retry...\n");
        led_set_state(LED_STATE_LORA_JOIN_FAILED);
        return false;
    }
}

// GNSS event handler - runs in ISR context, cannot use mutexes!
static void gnss_event_handler(int event)
{
    int retval;

    switch (event) {
    case NRF_MODEM_GNSS_EVT_PVT:
        retval = nrf_modem_gnss_read(&current_pvt, sizeof(current_pvt), NRF_MODEM_GNSS_DATA_PVT);
        if (retval == 0) {
            new_pvt_data = true;
            k_sem_give(&gnss_data_sem);
        }
        break;

    case NRF_MODEM_GNSS_EVT_FIX:
        gnss_has_fix = true;
        LOG_INF("GNSS fix acquired");
        printk("GNSS fix acquired!\n");
        // NOTE: Cannot call led_set_state() here - we're in ISR context!
        // LED will be updated in main loop when fix is detected
        break;

    default:
        break;
    }
}

// Initialize GNSS (EXACT copy from working LoRa project)
bool init_gnss(void)
{
    LOG_INF("Initializing GNSS...");
    printk("Initializing GNSS...\n");
    led_set_state(LED_STATE_GPS_INITIALIZING);

    // Initialize modem library (EXACT from working LoRa project)
    int err = nrf_modem_lib_init();
    if (err && err != -EALREADY) {  // Allow already initialized
        LOG_ERR("Modem library initialization failed, error: %d", err);
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }
    if (err == -EALREADY) {
        LOG_INF("Modem library already initialized");
    }

    // Enable GNSS functional mode (EXACT from working LoRa project)
    if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
        LOG_ERR("Failed to activate GNSS functional mode");
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }

    // Configure GNSS event handler (EXACT from working LoRa project)
    if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
        LOG_ERR("Failed to set GNSS event handler");
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }

    // Set GNSS configuration (EXACT from working LoRa project)
    uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;
    if (nrf_modem_gnss_use_case_set(use_case) != 0) {
        LOG_WRN("Failed to set GNSS use case");
    }

    // Set fix retry and interval (EXACT from working LoRa project)
    if (nrf_modem_gnss_fix_retry_set(0) != 0) {
        LOG_ERR("Failed to set GNSS fix retry");
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }

    if (nrf_modem_gnss_fix_interval_set(1) != 0) {
        LOG_ERR("Failed to set GNSS fix interval");
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }

    LOG_INF("GNSS initialized successfully");
    printk("GNSS initialized successfully\n");
    led_set_state(LED_STATE_GPS_INIT_SUCCESS);
    return true;
}

// Start GNSS (EXACT copy from working LoRa project)
bool start_gnss(void)
{
    LOG_INF("Starting GNSS...");
    printk("Starting GNSS search...\n");
    led_set_state(LED_STATE_GPS_SEARCHING);

    gnss_has_fix = false;

    if (nrf_modem_gnss_start() != 0) {
        LOG_ERR("Failed to start GNSS");
        printk("Failed to start GNSS\n");
        led_set_state(LED_STATE_GPS_INIT_FAILED);
        return false;
    }

    return true;
}

// Stop GNSS (from working LoRa project)
void stop_gnss(void)
{
    nrf_modem_gnss_stop();
    LOG_INF("GNSS stopped");
}

// Send location data via LoRaWAN
bool send_location_data(uint8_t msg_type)
{
    if (!gnss_has_fix) {
        LOG_WRN("No GNSS fix available, cannot send location");
        return false;
    }

    led_set_state(LED_STATE_LORA_SENDING);

    // Prepare payload: 1 byte msg type + 4 bytes lat + 4 bytes lon + 2 bytes alt + 2 bytes accuracy
    uint8_t payload[13];
    uint32_t lat_encoded = (uint32_t)((gnss_latitude + 90.0) * 1000000.0);
    uint32_t lon_encoded = (uint32_t)((gnss_longitude + 180.0) * 1000000.0);
    uint16_t alt_encoded = (uint16_t)(gnss_altitude + 1000.0f);
    uint16_t acc_encoded = (uint16_t)(gnss_accuracy * 10.0f);

    payload[0] = msg_type;

    // Pack latitude (4 bytes, big endian)
    payload[1] = (lat_encoded >> 24) & 0xFF;
    payload[2] = (lat_encoded >> 16) & 0xFF;
    payload[3] = (lat_encoded >> 8) & 0xFF;
    payload[4] = lat_encoded & 0xFF;

    // Pack longitude (4 bytes, big endian)
    payload[5] = (lon_encoded >> 24) & 0xFF;
    payload[6] = (lon_encoded >> 16) & 0xFF;
    payload[7] = (lon_encoded >> 8) & 0xFF;
    payload[8] = lon_encoded & 0xFF;

    // Pack altitude (2 bytes, big endian)
    payload[9] = (alt_encoded >> 8) & 0xFF;
    payload[10] = alt_encoded & 0xFF;

    // Pack accuracy (2 bytes, big endian)
    payload[11] = (acc_encoded >> 8) & 0xFF;
    payload[12] = acc_encoded & 0xFF;

    // Convert to hex string
    char hex_payload[32];
    for (int i = 0; i < 13; i++) {
        sprintf(&hex_payload[i * 2], "%02X", payload[i]);
    }
    hex_payload[26] = '\0';

    // Send via LoRaWAN on port 2
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMSGHEX=\"%s\"\r\n", hex_payload);

    if (at_send_check_response("Done", 30000, cmd)) {
        LOG_INF("Location data sent successfully (type: %s)",
               msg_type == MSG_TYPE_ALERT_RUNNING ? "RUNNING_ALERT" : "ROUTINE");

        if (msg_type == MSG_TYPE_ALERT_RUNNING) {
            led_set_state(LED_STATE_LOCATION_EMERGENCY_SENT);
        } else {
            led_set_state(LED_STATE_LOCATION_ROUTINE_SENT);
        }
        return true;
    } else {
        LOG_ERR("Failed to send location data");
        led_set_state(LED_STATE_LOCATION_SEND_FAILED);
        return false;
    }
}

// Running activity detection callback
void activity_detected_handler(ei_impulse_result_t* result)
{
    uint64_t current_time = k_uptime_get();

    // Check alert cooldown to prevent spam
    if (current_time - last_emergency_time < EMERGENCY_COOLDOWN_MS) {
        LOG_WRN("Alert cooldown active, ignoring running detection");
        led_set_state(LED_STATE_EMERGENCY_COOLDOWN);
        return;
    }

    emergency_mode = true;
    last_emergency_time = current_time;

    LOG_ERR("=== RUNNING DETECTED - ALERT ===");
    ei_printf("ALERT: Running activity detected!\r\n");
    led_set_state(LED_STATE_FALL_DETECTED);

    // Try to send alert with current location
    bool alert_sent = false;

    // Check if we have both GPS and LoRa available
    // (using the static variables defined at file scope)

    if (gnss_has_fix && lora_available) {
        if (send_location_data(MSG_TYPE_ALERT_RUNNING)) {
            ei_printf("Running alert with location sent via LoRaWAN!\r\n");
            led_set_state(LED_STATE_EMERGENCY_ALERT_SENT);
            alert_sent = true;
        } else {
            LOG_ERR("Failed to send LoRaWAN alert");
            led_set_state(LED_STATE_EMERGENCY_ALERT_FAILED);
        }
    } else {
        led_set_state(LED_STATE_EMERGENCY_ALERT_FAILED);
    }

    // Provide warnings for missing capabilities
    if (!alert_sent) {
        if (!gnss_has_fix) {
            ei_printf("Warning: No GPS fix for alert location\r\n");
        }
        if (!lora_available) {
            ei_printf("Warning: LoRaWAN not available for alert\r\n");
        }
        if (!gnss_has_fix && !lora_available) {
            ei_printf("Alert: Running detected but no communication available!\r\n");
        }
    }
}

int main(void)
{
    /* This is needed so that output of printf
       is output immediately without buffering
    */
    setvbuf(stdout, NULL, _IONBF, 0);

    // Initialize LED system first for visual feedback
    led_init();
    led_set_state(LED_STATE_SYSTEM_INIT);

    uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    if (!device_is_ready(uart)) {
		LOG_ERR("%s: device not ready\n", uart->name);
        led_set_state(LED_STATE_SYSTEM_ERROR);
        return -1;
	}

    // Initialize UART for LoRa-E5 (like working lora project)
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        led_set_state(LED_STATE_SYSTEM_ERROR);
        return -1;
    }

    int ret = uart_callback_set(uart_dev, uart_cb, NULL);
    if (ret) {
        LOG_ERR("UART callback set failed: %d", ret);
        led_set_state(LED_STATE_SYSTEM_ERROR);
        return -1;
    }

    ret = uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf) - 1, 5000);
    if (ret) {
        LOG_ERR("UART RX enable failed: %d", ret);
        led_set_state(LED_STATE_SYSTEM_ERROR);
        return -1;
    }

    // Wait for LoRa-E5 to boot (like working lora project)
    LOG_INF("Waiting for LoRa-E5 to boot...");
    k_msleep(2000);

#ifdef CONFIG_REMOTE_INGESTION
	EiDeviceNRF91* dev = static_cast<EiDeviceNRF91*>(EiDeviceInfo::get_device());
	if(!modem_connect()) {
		LOG_ERR("Modem connection failed!");
	}
	else {
    	ei_ws_client_start(dev, at_sample_start);
	}
#else
	// Modem library will be initialized later in init_gnss()
	printk("Modem library initialization deferred to GPS init\n");
#endif

    // Print startup message
    ei_printf("*************************\r\n");
    ei_printf("* Activity Monitoring + GPS Alert System *\r\n");
    ei_printf("*************************\r\n");
    ei_printf("Starting activity monitoring with GPS tracking...\r\n");

    // Initialize LoRa-E5 first and require it to work
    ei_printf("Initializing LoRa-E5 module (required)...\r\n");
    int lora_retries = 3;

    while (!lora_initialized && lora_retries > 0) {
        if (init_lora_e5()) {
            ei_printf("LoRa-E5 module initialized successfully\r\n");
            lora_available = true;
            break;  // Exit the loop since lora_initialized is set in init_lora_e5()
        } else {
            ei_printf("Failed to initialize LoRa-E5, retrying...\r\n");
            k_msleep(2000);
            lora_retries--;
        }
    }

    // Now try to join the network if initialized
    if (lora_initialized) {
        int join_retries = 3;
        while (!lora_joined && join_retries > 0) {
            if (join_lorawan_network()) {
                ei_printf("LoRaWAN network joined successfully\r\n");
                break;
            } else {
                ei_printf("Failed to join LoRaWAN network, retrying...\r\n");
                k_msleep(5000);
                join_retries--;
            }
        }
    }

    if (!lora_initialized) {
        ei_printf("ERROR: Could not initialize LoRa-E5. Activity monitoring will not start.\r\n");
        lora_available = false;
        // Don't start activity monitoring without LoRa
        while(1) {
            k_msleep(1000);
            ei_printf("Waiting for LoRa-E5 to be available...\r\n");
        }
    }

    // Initialize GPS and wait for fix
    ei_printf("Initializing GPS system (required)...\r\n");

    bool gps_ready = false;

    // Initialize GNSS once (don't retry modem init to avoid error -1)
    if (!init_gnss()) {
        ei_printf("ERROR: Failed to initialize GPS system!\r\n");
        gnss_available = false;
        while(1) {
            k_msleep(1000);
            ei_printf("GPS initialization failed - cannot continue\r\n");
        }
    }

    ei_printf("GPS system initialized\r\n");
    gnss_available = true;

    // Try starting GNSS search with retries (only restart search, not modem)
    int search_retries = 3;
    while (!gps_ready && search_retries > 0) {
        if (start_gnss()) {
            ei_printf("GPS search started, waiting for fix...\r\n");

            // Wait for GPS fix (up to 10 minutes)
            int fix_timeout = 600; // 10 minutes
            while (fix_timeout > 0 && !gnss_has_fix) {
                k_msleep(1000);
                fix_timeout--;

                if (fix_timeout % 10 == 0) {
                    // Read current PVT data to get satellite count and health
                    struct nrf_modem_gnss_pvt_data_frame temp_pvt;
                    if (nrf_modem_gnss_read(&temp_pvt, sizeof(temp_pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {
                        uint8_t tracked = 0;
                        uint8_t healthy = 0;
                        for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
                            if (temp_pvt.sv[i].sv > 0) {
                                tracked++;
                                // Satellite is healthy if signal strength is good (cn0 > 0)
                                if (temp_pvt.sv[i].cn0 > 0) {
                                    healthy++;
                                }
                            }
                        }
                        printk("Searching for GPS fix... (%d satellites tracked, %d healthy, %d seconds remaining)\n",
                               tracked, healthy, fix_timeout);
                    }
                    led_set_state(LED_STATE_GPS_SEARCHING);
                }
            }

            if (gnss_has_fix) {
                ei_printf("GPS fix obtained! Location: lat=%.6f, lon=%.6f\r\n",
                         gnss_latitude, gnss_longitude);
                gps_ready = true;
            } else {
                ei_printf("GPS fix timeout, stopping and retrying search...\r\n");
                led_set_state(LED_STATE_GPS_TIMEOUT);
                stop_gnss();
                k_msleep(5000); // Wait 5 seconds before retry
                search_retries--;
            }
        } else {
            ei_printf("Failed to start GPS search, retrying...\r\n");
            k_msleep(2000);
            search_retries--;
        }
    }

    if (!gps_ready) {
        ei_printf("ERROR: Could not obtain GPS fix. Activity monitoring will not start.\r\n");
        ei_printf("Please move device outdoors with clear sky view.\r\n");
        gnss_available = false;
        // Don't start activity monitoring without GPS fix
        while(1) {
            k_msleep(1000);
            ei_printf("Waiting for GPS fix...\r\n");
        }
    }

    // Both systems are ready - start activity monitoring
    ei_printf("=== ALL SYSTEMS READY ===\r\n");
    ei_printf("LoRa-E5: Connected to LoRaWAN network\r\n");
    ei_printf("GPS: Fix obtained at lat=%.6f, lon=%.6f\r\n", gnss_latitude, gnss_longitude);

    // NOW initialize Edge Impulse ML system AFTER LoRa and GPS are ready
    ei_printf("Initializing Edge Impulse ML system...\r\n");
    led_set_state(LED_STATE_ML_INITIALIZING);

    EiDeviceNRF91* dev = static_cast<EiDeviceNRF91*>(EiDeviceInfo::get_device());

    if (!ei_accelerometer_init()) {
        ei_printf("Failed to initialize accelerometer!\r\n");
        led_set_state(LED_STATE_ML_ERROR);
        return -1;
    }

    dev->set_state(eiStateFinished);
    led_set_state(LED_STATE_ML_READY);
    ei_printf("Edge Impulse ML system initialized successfully\r\n");

    ei_printf("Starting activity monitoring system...\r\n");
    led_set_state(LED_STATE_SYSTEM_READY);

    // Start continuous inference
    ei_start_impulse(false, false, false);

    // Main loop - run single-shot inference, restart immediately when done
    uint64_t last_routine_check = 0;
    uint64_t last_inference_led = 0;

    // Nordic-style polling for PVT and NMEA data
    // Simplified polling for GNSS data (working LoRa approach)

    while(true) {
        uint64_t current_time = k_uptime_get();

        // Run Edge Impulse inference continuously
        if(is_inference_running() == true) {
            // Show inference LED indicator occasionally (every 5 seconds)
            if (current_time - last_inference_led > 5000) {
                led_set_state(LED_STATE_ML_INFERENCING);
                last_inference_led = current_time;
            }
            ei_run_impulse();
        } else {
            // Restart immediately without delay
            ei_start_impulse(false, false, false);
        }

        // Check for routine location updates (every 5 minutes)
        if (!emergency_mode &&
            (current_time - last_routine_check) >= ROUTINE_INTERVAL_MS) {

            if (gnss_has_fix && lora_available) {
                LOG_INF("Sending routine location update");
                if (!send_location_data(MSG_TYPE_ROUTINE_UPDATE)) {
                    LOG_WRN("Failed to send routine location update");
                }
            } else if (!gnss_has_fix) {
                LOG_DBG("Skipping routine update - no GPS fix");
            } else if (!lora_available) {
                LOG_DBG("Skipping routine update - LoRa not available");
            }
            last_routine_check = current_time;
        }

        // Reset emergency mode after cooldown
        if (emergency_mode &&
            (current_time - last_emergency_time) >= EMERGENCY_COOLDOWN_MS) {
            emergency_mode = false;
            LOG_INF("Emergency mode deactivated");
        }

        // Nordic-style GNSS data processing with non-blocking poll
        int ret = k_poll(events, 2, K_MSEC(10));  // Short timeout to not block main loop

        if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
            // Process PVT data
            static bool first_fix = true;

            if (current_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
                bool was_no_fix = !gnss_has_fix;

                gnss_latitude = current_pvt.latitude;
                gnss_longitude = current_pvt.longitude;
                gnss_altitude = current_pvt.altitude;
                gnss_accuracy = current_pvt.accuracy;
                gnss_has_fix = true;  // Also set by ISR NRF_MODEM_GNSS_EVT_FIX event

                // Count satellites used in fix and health
                uint8_t tracked_sats = 0;
                uint8_t used_in_fix = 0;
                uint8_t healthy = 0;
                for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
                    if (current_pvt.sv[i].sv > 0) {
                        tracked_sats++;
                        if (current_pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
                            used_in_fix++;
                        }
                        if (current_pvt.sv[i].cn0 > 0) {
                            healthy++;
                        }
                    }
                }

                printk("GPS FIX: lat=%.6f, lon=%.6f, alt=%.1fm, acc=%.1fm, sats=%d/%d (healthy=%d)\n",
                       gnss_latitude, gnss_longitude, (double)gnss_altitude, (double)gnss_accuracy,
                       used_in_fix, tracked_sats, healthy);

                if (was_no_fix || first_fix) {
                    led_set_state(LED_STATE_GPS_FIX_ACQUIRED);
                    first_fix = false;
                }
            } else {
                bool had_fix = gnss_has_fix;
                gnss_has_fix = false;

                // Count tracked satellites from PVT data (like LoRa project)
                uint8_t tracked_sats = 0;
                uint8_t healthy = 0;
                for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
                    if (current_pvt.sv[i].sv > 0) {  // Non-zero satellite ID = tracked
                        tracked_sats++;
                        if (current_pvt.sv[i].cn0 > 0) {
                            healthy++;
                        }
                    }
                }

                static int search_count = 0;
                search_count++;
                if (search_count % 10 == 0) {  // Print every 10 seconds
                    printk("Searching for GPS fix... (%d satellites tracked, %d healthy)\n",
                           tracked_sats, healthy);
                }

                if (had_fix) {
                    printk("GPS FIX LOST! Searching...\n");
                    led_set_state(LED_STATE_GPS_FIX_LOST);
                }
            }
        }

        if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
            // Process NMEA data - just consume and free
            struct nrf_modem_gnss_nmea_data_frame *nmea_data;
            ret = k_msgq_get(&nmea_queue, &nmea_data, K_NO_WAIT);
            if (ret == 0) {
                k_free(nmea_data);
            }
        }

        // Reset event states
        events[0].state = K_POLL_STATE_NOT_READY;
        events[1].state = K_POLL_STATE_NOT_READY;

        k_sleep(K_MSEC(100));  // Slightly longer delay to allow GPS processing
    }

    return 0;
}
