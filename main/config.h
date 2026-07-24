#pragma once

// === Feature toggles ===
#define ENABLE_AUTO_RUN 1        // 1 = compile auto-run feature, 0 = disable entirely
#define AUTO_RUN_DEFAULT_ON 1    // 1 = active on power-up, 0 = start paused
#define AUTO_RUN_DEFAULT_HARD 0  // 1 = hard swing (instant to extrema + hold), 0 = sin² smooth
#define HARD_SWING_SPEED_X 4.0f  // Hard-swing period multiplier (>1 = faster, 4x = continuous)

// === Power management ===
#define POWER_CTRL_GPIO GPIO_NUM_7   // Latch HIGH = power on, LOW = power off
#define POWER_OUT_GPIO GPIO_NUM_6    // ADC read, <1V = button pressed
#define POWER_ADC_THRESHOLD 1241     // 1.0V threshold for pressed/released
#define POWER_LONG_PRESS_MS 1500
#define POWER_DEBOUNCE_MS 50
#define POWER_POLL_MS 20

// Battery ADC: IO3 = ADC1_CH2
// Voltage divider: R_upper=2k, R_lower=4.7k
// Vpin = Vbat * 4.7 / (2 + 4.7) → Vbat = Vpin * 1.426
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define BATTERY_DIVIDER_RATIO 2
#define BATTERY_EMPTY_VOLTAGE_MV 3200
#define BATTERY_FULL_VOLTAGE_MV 4200
#define BATTERY_READ_TICKS 250  // Every 5s (250 * 20ms)

// === Servo pins (3x 180° servos) ===
#define SERVO_POWER_GPIO GPIO_NUM_4
#define SERVO_HEAD_GPIO GPIO_NUM_15
#define SERVO_LH_GPIO GPIO_NUM_16
#define SERVO_RH_GPIO GPIO_NUM_17
#define SERVO_MAX_ANGLE 180

// === LEDC PWM 50Hz ===
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)
#define SERVO_PERIOD_US 20000

// === WiFi AP ===
#define WIFI_SSID "paxing_hx"
#define WIFI_MAX_CONN 4
