#include "led_states.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_states, CONFIG_LOG_DEFAULT_LEVEL);

// Global variables
static const struct device *gpio_dev = NULL;
static bool led_initialized = false;
static bool background_task_running __attribute__((unused)) = false;
static led_state_t current_state = LED_STATE_ALL_OFF;

// Background thread for LED patterns (unused for now)
static struct k_thread led_thread __attribute__((unused));
static K_THREAD_STACK_DEFINE(led_thread_stack, 1024) __attribute__((unused));

// Mutex for LED operations
K_MUTEX_DEFINE(led_mutex);

// Initialize LED subsystem
void led_init(void)
{
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready for LEDs");
        return;
    }

    // Configure LED pins as outputs (active low)
    gpio_pin_configure(gpio_dev, LED_RED_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, LED_GREEN_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, LED_BLUE_PIN, GPIO_OUTPUT_INACTIVE);

    led_initialized = true;
    LOG_INF("LED system initialized");
}

// Set individual LED color (Thingy:91 X LEDs are active HIGH)
void led_set_color(bool red, bool green, bool blue)
{
    if (!led_initialized) return;

    k_mutex_lock(&led_mutex, K_FOREVER);

    gpio_pin_set(gpio_dev, LED_RED_PIN, red ? 1 : 0);    // Active HIGH: 1=ON, 0=OFF
    gpio_pin_set(gpio_dev, LED_GREEN_PIN, green ? 1 : 0); // Active HIGH: 1=ON, 0=OFF
    gpio_pin_set(gpio_dev, LED_BLUE_PIN, blue ? 1 : 0);   // Active HIGH: 1=ON, 0=OFF

    k_mutex_unlock(&led_mutex);
}

// Turn off all LEDs
void led_all_off(void)
{
    led_set_color(false, false, false);
}

// Simple blink functions for specific colors
void led_blink_red(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(true, false, false);
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_green(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(false, true, false);
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_blue(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(false, false, true);
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_yellow(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(true, true, false);  // Red + Green = Yellow
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_cyan(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(false, true, true);  // Green + Blue = Cyan
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_magenta(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(true, false, true);  // Red + Blue = Magenta
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

void led_blink_white(uint32_t count, uint32_t duration_ms, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        led_set_color(true, true, true);  // All colors = White
        k_msleep(duration_ms);
        led_all_off();
        if (i < count - 1) k_msleep(delay_ms);
    }
}

// Solid color functions
void led_solid_green(uint32_t duration_ms)
{
    led_set_color(false, true, false);
    k_msleep(duration_ms);
    led_all_off();
}

void led_solid_red(uint32_t duration_ms)
{
    led_set_color(true, false, false);
    k_msleep(duration_ms);
    led_all_off();
}

// Emergency flash pattern
void led_emergency_flash(void)
{
    // Urgent red flashing pattern
    for (int i = 0; i < 10; i++) {
        led_set_color(true, false, false);
        k_msleep(100);
        led_all_off();
        k_msleep(100);
    }
    k_msleep(500);
    for (int i = 0; i < 10; i++) {
        led_set_color(true, false, false);
        k_msleep(100);
        led_all_off();
        k_msleep(100);
    }
}

// Breathing effect for operational state
void led_breathing_effect(void)
{
    // Gentle green breathing effect
    for (int brightness = 0; brightness < 100; brightness += 5) {
        led_set_color(false, true, false);
        k_msleep(10);
        led_all_off();
        k_msleep(50);
    }
    for (int brightness = 100; brightness > 0; brightness -= 5) {
        led_set_color(false, true, false);
        k_msleep(10);
        led_all_off();
        k_msleep(50);
    }
}

// Main LED state function
void led_set_state(led_state_t state)
{
    if (!led_initialized) return;

    current_state = state;

    switch (state) {
        case LED_STATE_SYSTEM_INIT:
            // White blink - system initializing
            led_blink_white(3, 200, 200);
            break;

        case LED_STATE_SYSTEM_READY:
            // Solid green for 2 seconds - all systems ready
            led_solid_green(2000);
            break;

        case LED_STATE_SYSTEM_ERROR:
            // Solid red - critical error
            led_set_color(true, false, false);
            break;

        // LoRa states
        case LED_STATE_LORA_INITIALIZING:
            // Yellow slow blink
            led_blink_yellow(5, 300, 500);
            break;

        case LED_STATE_LORA_INIT_SUCCESS:
            // Green blink
            led_blink_green(2, 200, 200);
            break;

        case LED_STATE_LORA_INIT_FAILED:
            // Red fast blink
            led_blink_red(5, 100, 100);
            break;

        case LED_STATE_LORA_JOINING:
            // Orange slow blink (red + green with timing)
            for (int i = 0; i < 5; i++) {
                led_set_color(true, true, false);
                k_msleep(400);
                led_all_off();
                k_msleep(600);
            }
            break;

        case LED_STATE_LORA_JOINED:
            // Green double blink
            led_blink_green(2, 150, 150);
            k_msleep(300);
            led_blink_green(2, 150, 150);
            break;

        case LED_STATE_LORA_JOIN_FAILED:
            // Red double blink
            led_blink_red(2, 150, 150);
            k_msleep(300);
            led_blink_red(2, 150, 150);
            break;

        case LED_STATE_LORA_SENDING:
            // Blue fast blink
            led_blink_blue(8, 50, 50);
            break;

        case LED_STATE_LORA_SEND_SUCCESS:
            // Green triple blink
            led_blink_green(3, 100, 100);
            break;

        case LED_STATE_LORA_SEND_FAILED:
            // Red triple blink
            led_blink_red(3, 100, 100);
            break;

        // GPS states
        case LED_STATE_GPS_INITIALIZING:
            // Cyan slow blink
            led_blink_cyan(5, 300, 500);
            break;

        case LED_STATE_GPS_INIT_SUCCESS:
            // Green blink
            led_blink_green(1, 300, 0);
            break;

        case LED_STATE_GPS_INIT_FAILED:
            // Red blink
            led_blink_red(1, 300, 0);
            break;

        case LED_STATE_GPS_SEARCHING:
            // Purple slow blink
            led_blink_magenta(1, 500, 1000);
            break;

        case LED_STATE_GPS_FIX_ACQUIRED:
            // Green solid for 2 seconds
            led_solid_green(2000);
            break;

        case LED_STATE_GPS_FIX_LOST:
            // Orange blink
            for (int i = 0; i < 3; i++) {
                led_set_color(true, true, false);
                k_msleep(200);
                led_all_off();
                k_msleep(200);
            }
            break;

        case LED_STATE_GPS_TIMEOUT:
            // Red slow blink
            led_blink_red(3, 500, 500);
            break;

        // ML states
        case LED_STATE_ML_INITIALIZING:
            // Magenta slow blink
            led_blink_magenta(5, 300, 300);
            break;

        case LED_STATE_ML_READY:
            // Green single blink
            led_blink_green(1, 200, 0);
            break;

        case LED_STATE_ML_INFERENCING:
            // Blue very fast blink
            led_blink_blue(1, 50, 0);
            break;

        case LED_STATE_ML_IDLE_DETECTED:
            // Blue dim (short pulse)
            led_blink_blue(1, 10, 0);
            break;

        case LED_STATE_ML_ACTIVE_DETECTED:
            // Blue bright (longer pulse)
            led_blink_blue(1, 100, 0);
            break;

        case LED_STATE_ML_ERROR:
            // Red blink
            led_blink_red(1, 200, 0);
            break;

        // Emergency states
        case LED_STATE_FALL_DETECTED:
            // Urgent red flashing
            led_emergency_flash();
            break;

        case LED_STATE_EMERGENCY_ALERT_SENT:
            // Red + Green alternating
            for (int i = 0; i < 6; i++) {
                led_set_color(true, false, false);
                k_msleep(150);
                led_set_color(false, true, false);
                k_msleep(150);
            }
            led_all_off();
            break;

        case LED_STATE_EMERGENCY_ALERT_FAILED:
            // Red continuous for 3 seconds
            led_set_color(true, false, false);
            k_msleep(3000);
            led_all_off();
            break;

        case LED_STATE_EMERGENCY_COOLDOWN:
            // Orange dim
            led_set_color(true, true, false);
            k_msleep(100);
            led_all_off();
            break;

        // Location transmission states
        case LED_STATE_LOCATION_ROUTINE_SENT:
            // Blue triple blink
            led_blink_blue(3, 100, 100);
            break;

        case LED_STATE_LOCATION_EMERGENCY_SENT:
            // Red + Blue alternating
            for (int i = 0; i < 4; i++) {
                led_set_color(true, false, false);
                k_msleep(100);
                led_set_color(false, false, true);
                k_msleep(100);
            }
            led_all_off();
            break;

        case LED_STATE_LOCATION_SEND_FAILED:
            // Red + Orange alternating
            for (int i = 0; i < 3; i++) {
                led_set_color(true, false, false);
                k_msleep(150);
                led_set_color(true, true, false);
                k_msleep(150);
            }
            led_all_off();
            break;

        case LED_STATE_ALL_OFF:
        default:
            led_all_off();
            break;
    }
}