#ifndef LED_STATES_H
#define LED_STATES_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// LED State Management System for Fall Detection + GPS Emergency Alert System

// LED pin definitions (Thingy:91 X) - corrected mapping: P29=RED, P31=GREEN, P30=BLUE
#define LED_RED_PIN   29  // P0.29 controls RED LED
#define LED_GREEN_PIN 31  // P0.31 controls GREEN LED
#define LED_BLUE_PIN  30  // P0.30 controls BLUE LED

// System state definitions
typedef enum {
    // System states
    LED_STATE_SYSTEM_INIT,              // White (all colors) - System initializing
    LED_STATE_SYSTEM_READY,             // Green solid - All systems operational
    LED_STATE_SYSTEM_ERROR,             // Red solid - Critical system error

    // LoRa states
    LED_STATE_LORA_INITIALIZING,        // Yellow slow blink - LoRa module initializing
    LED_STATE_LORA_INIT_SUCCESS,        // Green blink - LoRa initialized successfully
    LED_STATE_LORA_INIT_FAILED,         // Red fast blink - LoRa initialization failed
    LED_STATE_LORA_JOINING,             // Orange slow blink - Joining LoRaWAN network
    LED_STATE_LORA_JOINED,              // Green double blink - Successfully joined network
    LED_STATE_LORA_JOIN_FAILED,         // Red double blink - Failed to join network
    LED_STATE_LORA_SENDING,             // Blue fast blink - Sending data
    LED_STATE_LORA_SEND_SUCCESS,        // Green triple blink - Data sent successfully
    LED_STATE_LORA_SEND_FAILED,         // Red triple blink - Data send failed

    // GPS states
    LED_STATE_GPS_INITIALIZING,         // Cyan slow blink - GPS initializing
    LED_STATE_GPS_INIT_SUCCESS,         // Green blink - GPS initialized
    LED_STATE_GPS_INIT_FAILED,          // Red blink - GPS initialization failed
    LED_STATE_GPS_SEARCHING,            // Purple slow blink - Searching for GPS fix
    LED_STATE_GPS_FIX_ACQUIRED,         // Green solid 2s - GPS fix acquired
    LED_STATE_GPS_FIX_LOST,             // Orange blink - GPS fix lost
    LED_STATE_GPS_TIMEOUT,              // Red slow blink - GPS fix timeout

    // Edge Impulse / ML states
    LED_STATE_ML_INITIALIZING,          // Magenta slow blink - ML system initializing
    LED_STATE_ML_READY,                 // Green blink - ML system ready
    LED_STATE_ML_INFERENCING,           // Blue very fast blink - Running inference
    LED_STATE_ML_IDLE_DETECTED,         // Blue dim - Idle state detected
    LED_STATE_ML_ACTIVE_DETECTED,       // Blue bright - Active state detected
    LED_STATE_ML_ERROR,                 // Red blink - ML system error

    // Emergency states
    LED_STATE_FALL_DETECTED,            // Red urgent flash - Fall detected!
    LED_STATE_EMERGENCY_ALERT_SENT,     // Red + Green alternate - Emergency alert sent
    LED_STATE_EMERGENCY_ALERT_FAILED,   // Red continuous - Emergency alert failed
    LED_STATE_EMERGENCY_COOLDOWN,       // Orange dim - Emergency cooldown active

    // Location transmission states
    LED_STATE_LOCATION_ROUTINE_SENT,    // Blue triple blink - Routine location sent
    LED_STATE_LOCATION_EMERGENCY_SENT,  // Red + Blue alternate - Emergency location sent
    LED_STATE_LOCATION_SEND_FAILED,     // Red + Orange alternate - Location send failed

    // Special states
    LED_STATE_ALL_OFF,                  // All LEDs off
    LED_STATE_BREATHING,                // Gentle breathing effect (operational)

} led_state_t;

// LED control structure
typedef struct {
    bool red_on;
    bool green_on;
    bool blue_on;
    uint32_t duration_ms;
    uint32_t repeat_count;
    uint32_t delay_between_ms;
} led_pattern_t;

// Function declarations
void led_init(void);
void led_set_state(led_state_t state);
void led_set_color(bool red, bool green, bool blue);
void led_blink_pattern(led_pattern_t *pattern);
void led_emergency_flash(void);
void led_breathing_effect(void);
void led_all_off(void);

// Utility functions for common patterns
void led_blink_red(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_green(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_blue(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_yellow(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_cyan(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_magenta(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_blink_white(uint32_t count, uint32_t duration_ms, uint32_t delay_ms);
void led_solid_green(uint32_t duration_ms);
void led_solid_red(uint32_t duration_ms);

// Background LED state management
void led_start_background_task(void);
void led_stop_background_task(void);

#ifdef __cplusplus
}
#endif

#endif // LED_STATES_H