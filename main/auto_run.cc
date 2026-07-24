#include "auto_run.h"
#include "config.h"
#include "servo.h"

#if ENABLE_AUTO_RUN

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdio>
#include <cstring>

static const char *TAG = "action_cat";

static volatile bool auto_run_running_ = AUTO_RUN_DEFAULT_ON;
static volatile bool auto_run_hard_swing_ = AUTO_RUN_DEFAULT_HARD;

bool IsAutoRunRunning()          { return auto_run_running_; }
void SetAutoRunRunning(bool v)   { auto_run_running_ = v; }
bool IsAutoRunHardSwing()        { return auto_run_hard_swing_; }
void SetAutoRunHardSwing(bool v) { auto_run_hard_swing_ = v; }

// === Auto-run continuous motion task ===
static void AutoRunTask(void *arg)
{
  constexpr int kTickMs = 20;
  constexpr int kHeadCenter = 90;
  constexpr int kHeadAmplitude = 20;
  constexpr float kHeadPeriodS = 1.5f;

  constexpr int kLhStart = 90;
  constexpr int kLhEnd = 0;
  constexpr int kRhStart = 90;
  constexpr int kRhEnd = 180;
  constexpr float kCrawlPeriodS = 0.8f;
  constexpr int kModeSwitchS = 8;
  constexpr float kBlendDurationS = 1.0f;

  uint32_t tick = 0;
  uint32_t next_switch_tick = (uint32_t)(kModeSwitchS * 1000 / kTickMs);
  bool alt_mode = true;

  bool blend_active = false;
  bool blend_from_alt = false;
  uint32_t blend_start_tick = 0;

  auto calcCrawlSmooth = [](float phase, bool is_alt, int &lh, int &rh) {
    float val_lh = sinf(phase * M_PI);
    val_lh = val_lh * val_lh;
    float phase_rh = is_alt ? (phase + 0.5f) : phase;
    phase_rh = phase_rh - floorf(phase_rh);
    float val_rh = sinf(phase_rh * M_PI);
    val_rh = val_rh * val_rh;
    lh = kLhStart + (int)((kLhEnd - kLhStart) * val_lh);
    rh = kRhStart + (int)((kRhEnd - kRhStart) * val_rh);
  };

  auto calcCrawlHard = [](float phase, bool is_alt, int &lh, int &rh) {
    bool at_end = (phase >= 0.5f);
    int lh_val = at_end ? 1 : 0;
    int rh_val = (is_alt ? !at_end : at_end) ? 1 : 0;
    lh = kLhStart + (kLhEnd - kLhStart) * lh_val;
    rh = kRhStart + (kRhEnd - kRhStart) * rh_val;
  };

  auto calcCrawl = [&](float phase, bool is_alt, int &lh, int &rh) {
    if (auto_run_hard_swing_)
      calcCrawlHard(phase, is_alt, lh, rh);
    else
      calcCrawlSmooth(phase, is_alt, lh, rh);
  };

  ESP_LOGI(TAG, "Auto-run started: head nod ±%d°, crawl alt/both %ds each, blend %.1fs",
           kHeadAmplitude, kModeSwitchS, kBlendDurationS);

  while (true)
  {
    if (!auto_run_running_)
    {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    float t_s = tick * kTickMs / 1000.0f;

    // --- Head nodding ---
    float head_phase = t_s / kHeadPeriodS;
    int head_angle = kHeadCenter + (int)(kHeadAmplitude * sinf(head_phase * 2.0f * M_PI));
    SetServoAngle(0, head_angle);

    // --- Mode switching ---
    if (tick >= next_switch_tick)
    {
      blend_from_alt = alt_mode;
      alt_mode = !alt_mode;
      blend_active = true;
      blend_start_tick = tick;
      next_switch_tick = tick + (uint32_t)(kModeSwitchS * 1000 / kTickMs);
      ESP_LOGI(TAG, "Auto crawl: %s → %s (blending %.1fs)",
               blend_from_alt ? "ALT" : "BOTH",
               alt_mode ? "ALT" : "BOTH",
               (double)kBlendDurationS);
    }

    float eff_period = auto_run_hard_swing_ ? kCrawlPeriodS / HARD_SWING_SPEED_X : kCrawlPeriodS;
    float phase = (tick * kTickMs / 1000.0f) / eff_period;
    phase = phase - floorf(phase);

    int lh_angle, rh_angle;
    if (blend_active)
    {
      float blend_t = (tick - blend_start_tick) * kTickMs / 1000.0f;
      if (blend_t >= kBlendDurationS)
      {
        blend_active = false;
        calcCrawl(phase, alt_mode, lh_angle, rh_angle);
      }
      else
      {
        float t = blend_t / kBlendDurationS;
        float mix = t * t * (3.0f - 2.0f * t);

        int old_lh, old_rh, new_lh, new_rh;
        calcCrawl(phase, blend_from_alt, old_lh, old_rh);
        calcCrawl(phase, alt_mode, new_lh, new_rh);

        lh_angle = old_lh + (int)((new_lh - old_lh) * mix);
        rh_angle = old_rh + (int)((new_rh - old_rh) * mix);
      }
    }
    else
    {
      calcCrawl(phase, alt_mode, lh_angle, rh_angle);
    }

    SetServoAngle(1, lh_angle);
    SetServoAngle(2, rh_angle);

    tick++;
    vTaskDelay(pdMS_TO_TICKS(kTickMs));
  }
}

esp_err_t HandleAutoPlay(httpd_req_t *req)
{
  if (req->method == HTTP_POST)
  {
    char buf[64] = {};
    httpd_req_recv(req, buf, sizeof(buf) - 1);
    const char *p = strstr(buf, "\"enable\":");
    if (p) { p += 9; auto_run_running_ = (atoi(p) != 0); }
    else if (!strstr(buf, "hard_swing"))
      auto_run_running_ = !auto_run_running_;

    p = strstr(buf, "\"hard_swing\":");
    if (p) { p += 13; auto_run_hard_swing_ = (strncmp(p, "true", 4) == 0 || atoi(p) == 1); }
  }

  char resp[96];
  snprintf(resp, sizeof(resp),
           "{\"autoplay\":%s,\"hard_swing\":%s}",
           auto_run_running_ ? "true" : "false",
           auto_run_hard_swing_ ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  ESP_LOGI(TAG, "Auto-play %s, hard_swing=%s",
           auto_run_running_ ? "ON" : "OFF",
           auto_run_hard_swing_ ? "ON" : "OFF");
  return ESP_OK;
}

void InitAutoRun()
{
  xTaskCreate(AutoRunTask, "auto_run", 4096, nullptr, 2, nullptr);
  ESP_LOGI(TAG, "Auto-run task created");
}

#endif  // ENABLE_AUTO_RUN
