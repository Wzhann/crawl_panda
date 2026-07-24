#include "servo.h"
#include "config.h"

#include <algorithm>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

static const char *TAG = "action_cat";

static constexpr gpio_num_t kServoPins[] = {
    SERVO_HEAD_GPIO, SERVO_LH_GPIO, SERVO_RH_GPIO};
static constexpr ledc_channel_t kServoCh[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

const int kServoCount = sizeof(kServoPins) / sizeof(kServoPins[0]);

static int servo_angles_[3] = {90, 90, 90};

static uint32_t AngleToDuty(int angle)
{
  angle = std::clamp(angle, 0, SERVO_MAX_ANGLE);
  int32_t pulse_us = 500 + angle * 2000 / SERVO_MAX_ANGLE;
  if (pulse_us < 500) pulse_us = 500;
  if (pulse_us > 2500) pulse_us = 2500;
  return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

void SetServoAngle(int idx, int angle)
{
  if (idx < 0 || idx >= kServoCount) return;
  angle = std::clamp(angle, 0, SERVO_MAX_ANGLE);
  servo_angles_[idx] = angle;
  uint32_t duty = AngleToDuty(angle);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, kServoCh[idx], duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, kServoCh[idx]);
}

void InitServos()
{
  gpio_config_t pwr_cfg = {
      .pin_bit_mask = 1ULL << SERVO_POWER_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&pwr_cfg);
  gpio_set_level(SERVO_POWER_GPIO, 1);

  ledc_timer_config_t tcfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = SERVO_DUTY_RES,
      .timer_num = SERVO_TIMER,
      .freq_hz = SERVO_FREQ_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&tcfg);

  for (int i = 0; i < kServoCount; i++)
  {
    ledc_channel_config_t ch = {
        .gpio_num = static_cast<int>(kServoPins[i]),
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = kServoCh[i],
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = AngleToDuty(servo_angles_[i]),
        .hpoint = 0,
        .flags = {.output_invert = 0},
    };
    ledc_channel_config(&ch);
  }
  ESP_LOGI(TAG, "Servos initialized");
}
