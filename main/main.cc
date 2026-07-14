#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#define TAG "action_cat"

// === Feature toggles ===
#define ENABLE_AUTO_RUN 1      // 1 = compile auto-run feature, 0 = disable entirely
#define AUTO_RUN_DEFAULT_ON 1  // 1 = active on power-up, 0 = start paused (only when ENABLE_AUTO_RUN=1)

// === Power management ===
#define POWER_CTRL_GPIO GPIO_NUM_7 // Latch HIGH = power on, LOW = power off
#define POWER_OUT_GPIO GPIO_NUM_6  // ADC read, <1V = button pressed
#define POWER_ADC_THRESHOLD 1241   // 1.0V threshold for pressed/released
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
#define BATTERY_READ_TICKS 250 // Every 5s (250 * 20ms)

static adc_oneshot_unit_handle_t adc_handle_ = nullptr;
static adc_cali_handle_t adc_cali_handle_ = nullptr;
static int battery_vbat_filtered_mv_ = 0;
static int battery_level_ = 0;

static int AdcToMv(int raw)
{
  if (adc_cali_handle_)
  {
    int mv = 0;
    if (adc_cali_raw_to_voltage(adc_cali_handle_, raw, &mv) == ESP_OK)
      return mv;
  }
  return raw * 3300 / 4096;
}

static void InitPower()
{
  // --- POWER_CTRL (IO7): latch HIGH immediately ---
  gpio_config_t pwr_ctrl = {
      .pin_bit_mask = 1ULL << POWER_CTRL_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&pwr_ctrl);
  gpio_set_level(POWER_CTRL_GPIO, 1);
  ESP_LOGI(TAG, "POWER_CTRL IO%d HIGH, power latched", POWER_CTRL_GPIO);

  // POWER_OUT (IO6): ADC monitoring via ADC1_CH5
  gpio_config_t pwr_out = {
      .pin_bit_mask = 1ULL << POWER_OUT_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&pwr_out);
  gpio_hold_en(POWER_OUT_GPIO);

  // ADC init
  adc_oneshot_unit_init_cfg_t adc_cfg = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  if (adc_oneshot_new_unit(&adc_cfg, &adc_handle_) == ESP_OK)
  {
    adc_oneshot_chan_cfg_t ch = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_5, &ch);      // IO6 power button
    adc_oneshot_config_channel(adc_handle_, BATTERY_ADC_CHANNEL, &ch); // IO3 battery

    adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_5,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali, &adc_cali_handle_);
  }

  // Power monitor task: symmetric hysteresis debounce + long-press shutdown + battery
  xTaskCreate([](void *)
              {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for stabilization

        // Symmetric hysteresis debounce state (ref: gpio_adcTest)
        int pwr_stable = 1;       // Debounced state: 1=released, 0=pressed
        int pwr_debounce = 5;     // Debounce counter
        uint32_t pwr_hold_ms = 0; // Pressed duration
        uint32_t tick = 0;

        const int debounce_max = POWER_DEBOUNCE_MS / POWER_POLL_MS;

        while (true) {
            tick++;

            // --- Read power button ADC ---
            int raw = 0;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_5, &raw);
            int raw_high = (raw > POWER_ADC_THRESHOLD) ? 1 : 0;  // >1241=released

            // Symmetric hysteresis debounce
            if (raw_high == pwr_stable) {
                if (pwr_debounce < debounce_max) pwr_debounce++;
            } else {
                if (pwr_debounce > 0) pwr_debounce--;
                else { pwr_stable = raw_high; pwr_debounce = 1; }
            }

            // Edge detection
            static int prev_stable = 1;
            if (pwr_stable != prev_stable) {
                if (!pwr_stable) {
                    ESP_LOGI(TAG, "Power button PRESSED");
                    pwr_hold_ms = 0;
                } else {
                    ESP_LOGI(TAG, "Power button RELEASED (held %dms)", (int)pwr_hold_ms);
                }
                prev_stable = pwr_stable;
            }

            // Long-press detection + shutdown
            if (!pwr_stable) {
                pwr_hold_ms += POWER_POLL_MS;
                if (pwr_hold_ms >= POWER_LONG_PRESS_MS) {
                    ESP_LOGW(TAG, "Long press %dms -> SHUTDOWN (IO%d -> LOW)",
                             POWER_LONG_PRESS_MS, POWER_CTRL_GPIO);
                    gpio_set_level(POWER_CTRL_GPIO, 0);
                    // Infinite loop — wait for external power cut
                    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
                }
            }

            // --- Battery voltage (IO3) — every 5s ---
            if (tick % BATTERY_READ_TICKS == 0) {
                int64_t bat_sum = 0;
                for (int n = 0; n < 32; n++) {
                    int bat_raw = 0;
                    adc_oneshot_read(adc_handle_, BATTERY_ADC_CHANNEL, &bat_raw);
                    bat_sum += bat_raw;
                }
                int bat_avg = static_cast<int>(bat_sum / 32);
                int vpin_mv = AdcToMv(bat_avg);
                int vbat_mv = static_cast<int>(vpin_mv * BATTERY_DIVIDER_RATIO);
                if (battery_vbat_filtered_mv_ == 0) {
                    battery_vbat_filtered_mv_ = vbat_mv;
                } else {
                    battery_vbat_filtered_mv_ += (vbat_mv - battery_vbat_filtered_mv_) / 5;
                }
                int vbat_f = battery_vbat_filtered_mv_;
                battery_level_ = (vbat_f - BATTERY_EMPTY_VOLTAGE_MV) * 100 /
                                 (BATTERY_FULL_VOLTAGE_MV - BATTERY_EMPTY_VOLTAGE_MV);
                if (battery_level_ < 0) battery_level_ = 0;
                if (battery_level_ > 100) battery_level_ = 100;
                ESP_LOGI(TAG, "[BAT] %dmV (filt=%dmV) level=%d%%",
                         vbat_mv, vbat_f, battery_level_);
            }

            vTaskDelay(pdMS_TO_TICKS(POWER_POLL_MS));
        } }, "power_mon", 4096, nullptr, 1, nullptr);
  ESP_LOGI(TAG, "Power monitor started (IO6 ADC, long press %dms, debounce %dms)",
           POWER_LONG_PRESS_MS, POWER_DEBOUNCE_MS);
}

// === Servo pins (3x 180° servos, 0-180 coordinate) ===
// Head(IO15): 0=left, 90=center, 180=right
// LeftHand(IO16): 0-180, 90=rest, 180=crawl down
// RightHand(IO17): 0-180, 90=rest, 0=crawl down (mirrored)
#define SERVO_POWER_GPIO GPIO_NUM_4
#define SERVO_HEAD_GPIO GPIO_NUM_15
#define SERVO_LH_GPIO GPIO_NUM_16
#define SERVO_RH_GPIO GPIO_NUM_17
#define SERVO_MAX_ANGLE 180

static constexpr int kServoCount = 3;
static constexpr gpio_num_t kServoPins[kServoCount] = {
    SERVO_HEAD_GPIO, SERVO_LH_GPIO, SERVO_RH_GPIO};
static constexpr ledc_channel_t kServoCh[kServoCount] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
static constexpr const char* kServoNames[kServoCount] = {
    "Head_IO15", "LeftHand_IO16", "RightHand_IO17"};
// Neutral: Head=90(center), LH=90(rest), RH=90(rest)
static int servo_angles_[kServoCount] = {90, 90, 90};

// === LEDC PWM 50Hz ===
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)
#define SERVO_PERIOD_US 20000

// 180° servos: 500-2500μs pulse, 0°~180° user angle
// pulse_us = 500 + angle * 2000 / 180
static uint32_t AngleToDuty(int angle)
{
  angle = std::clamp(angle, 0, SERVO_MAX_ANGLE);
  int32_t pulse_us = 500 + angle * 2000 / SERVO_MAX_ANGLE;
  if (pulse_us < 500) pulse_us = 500;
  if (pulse_us > 2500) pulse_us = 2500;
  return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

static void SetServoAngle(int idx, int angle)
{
  angle = std::clamp(angle, 0, SERVO_MAX_ANGLE);
  servo_angles_[idx] = angle;           // Store logical angle (0-180 user-facing)
  uint32_t duty = AngleToDuty(angle);   // No inversion: 0=500μs, 180=2500μs
  ledc_set_duty(LEDC_LOW_SPEED_MODE, kServoCh[idx], duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, kServoCh[idx]);
}

static void InitServos()
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

// === WiFi AP ===
static void InitWiFi()
{
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  wifi_config_t wcfg = {};
  strcpy((char *)wcfg.ap.ssid, "paxing_hx");
  wcfg.ap.ssid_len = strlen("paxing_hx");
  wcfg.ap.authmode = WIFI_AUTH_OPEN;
  wcfg.ap.max_connection = 4;

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &wcfg);
  esp_wifi_start();
  ESP_LOGI(TAG, "WiFi AP: breathe_he (open)");
}

// === Web page (breathe_he: 呼吸 & 爬行) ===
static const char kHtml[] = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Breathe He</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#1a1a2e;color:#eee;padding:8px;max-width:520px;margin:auto}
h1{text-align:center;font-size:16px;color:#e94560;margin:6px 0}
.card{background:#16213e;border-radius:8px;padding:10px;margin-bottom:8px}
h2{font-size:13px;margin-bottom:5px;color:#4ecca3}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.col{flex:1;min-width:60px}
.btn{padding:8px 14px;border:none;border-radius:4px;font-size:13px;cursor:pointer;color:#fff;margin:3px}
.btn-rec{background:#e94560}.btn-stop{background:#555}.btn-copy{background:#4ecca3;color:#000}
.btn-play{background:#ff6b35}.btn-reset{background:#333}
.btn-on{background:#2ecc71;color:#000}
input,select{width:100%;padding:6px;border:1px solid #333;border-radius:4px;background:#0f3460;color:#eee;font-size:13px;margin:2px 0}
input[type=range]{-webkit-appearance:none;width:100%;height:28px;background:linear-gradient(90deg,#0f3460,#e94560);border-radius:4px;margin:4px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;background:#e94560;border-radius:50%}
.grp-head{background:#1a2e3a;border-left:3px solid #4ea3cc}
.grp-lh{background:#1a3a2e;border-left:3px solid #4ecca3}
.grp-rh{background:#2e1a3a;border-left:3px solid #a34ecc}
.grp-crawl{background:#2e2a1a;border-left:3px solid #e67e22}
.grp-hands{background:#1a2e1a;border-left:3px solid #f39c12}
.lbl{display:flex;justify-content:space-between;font-size:11px;color:#aaa}
.val{font-size:20px;font-weight:bold;color:#4ecca3;text-align:center;min-width:50px}
.grp-head .val{color:#4ea3cc}
.grp-lh .val{color:#4ecca3}
.grp-rh .val{color:#a34ecc}
pre{background:#0a0a1a;color:#4ecca3;padding:8px;border-radius:4px;font-size:10px;overflow-x:auto;max-height:300px;overflow-y:auto;white-space:pre-wrap;word-break:break-all}
.status{padding:4px;text-align:center;font-size:12px;color:#888;min-height:20px}
.hint{font-size:10px;color:#666;margin-top:2px}
.joy-row{display:flex;gap:12px;align-items:center;justify-content:center}
.joy{background:#0a0f1e;border-radius:50%;cursor:pointer;touch-action:none;display:block}
.joy-info{display:flex;flex-direction:column;gap:6px;min-width:70px}
.joy-lbl{font-size:11px;color:#aaa;text-align:center}
.joy-lbl .val{font-size:22px;font-weight:bold;display:block}
.grp-hands .joy{border:3px solid #f39c12;box-shadow:0 0 12px rgba(243,156,18,0.25)}
.mode-row{display:flex;gap:4px;align-items:center}
.mode-tag{font-size:9px;padding:2px 6px;border-radius:4px;background:#333;color:#888}
.mode-tag.active{background:#2ecc71;color:#000}
</style>
</head>
<body>
<h1>Breathe He</h1>
<div class="status" id="status">就绪</div>

<!-- HEAD -->
<div class="card grp-head">
<h2>Head 头部 <span style="font-size:10px;color:#888">IO15 | 0=左 90=中 180=右</span></h2>
<div class="lbl"><span>头部角度</span><span class="val" id="v0">90°</span></div>
<input type="range" id="s0" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setHead(0)">0°左</button>
<button class="btn btn-reset" onclick="setHead(90)">90°中</button>
<button class="btn btn-reset" onclick="setHead(180)">180°右</button>
<button class="btn btn-play" onclick="runHeadNod()" id="btnNod">点头</button>
</div>
</div>

<!-- LEFT HAND -->
<div class="card grp-lh">
<h2>Left Hand 左手 <span style="font-size:10px;color:#888">IO16 | 0=上 90=休息 180=爬下</span></h2>
<div class="lbl"><span>左手角度</span><span class="val" id="v1">90°</span></div>
<input type="range" id="s1" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setLH(0)">0°上</button>
<button class="btn btn-reset" onclick="setLH(90)">休息</button>
<button class="btn btn-reset" onclick="setLH(180)">爬下</button>
</div>
</div>

<!-- RIGHT HAND -->
<div class="card grp-rh">
<h2>Right Hand 右手 <span style="font-size:10px;color:#888">IO17 | 0=爬下 90=休息 180=上</span></h2>
<div class="lbl"><span>右手角度</span><span class="val" id="v2">90°</span></div>
<input type="range" id="s2" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setRH(0)">爬下</button>
<button class="btn btn-reset" onclick="setRH(90)">休息</button>
<button class="btn btn-reset" onclick="setRH(180)">180°上</button>
</div>
</div>
<!-- HANDS JOYSTICK (crawl control) -->
<div class="card grp-hands">
<h2>双手摇杆 (0-180) <span style="font-size:10px;color:#888">↑=0°上 | ↓=180°爬行 | ←→=侧重</span></h2>
<div class="joy-row">
<canvas class="joy" id="joyHands" width="180" height="180"></canvas>
<div class="joy-info">
<div class="joy-lbl"><span>左手</span><span class="val" id="v1j">90°</span></div>
<div class="joy-lbl"><span>右手</span><span class="val" id="v2j">90°</span></div>
</div>
</div>
<div class="row" style="margin-top:4px">
	<button class="btn btn-reset" onclick="handsJoy.setAngles(0,180)">上</button>
	<button class="btn btn-on" onclick="handsJoy.setAngles(90,90)">休息</button>
	<button class="btn btn-reset" onclick="handsJoy.setAngles(180,0)">爬行</button>
<span style="font-size:9px;color:#888">拖动摇杆: ↑0° ↓180° ←→转弯</span>
</div>
</div>

	<!-- CRAWLING -->
	<div class="card grp-crawl">
	<h2>爬行控制</h2>
	<div class="hint">交替=直行 | 不同起止=转弯 | 摇杆←→控转弯</div>
	<div class="row" style="margin-top:6px">
	<div class="col">
	<label style="font-size:11px">模式</label>
	<select id="crawlMode" onchange="onCrawlParamChange()">
	<option value="alt">交替爬行</option>
	<option value="both">同时爬行</option>
	<option value="lh">左单臂转弯</option>
	<option value="rh">右单臂转弯</option>
	</select>
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col">
	<div class="lbl"><span>左起</span><span id="crawlLhStartVal">90</span></div>
	<input type="range" id="crawlLhStart" min="0" max="180" value="90" oninput="onCrawlParamChange()">
	</div>
	<div class="col">
	<div class="lbl"><span>左止</span><span id="crawlLhEndVal">180</span></div>
	<input type="range" id="crawlLhEnd" min="0" max="180" value="180" oninput="onCrawlParamChange()">
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col">
	<div class="lbl"><span>右起</span><span id="crawlRhStartVal">90</span></div>
	<input type="range" id="crawlRhStart" min="0" max="180" value="90" oninput="onCrawlParamChange()">
	</div>
	<div class="col">
	<div class="lbl"><span>右止</span><span id="crawlRhEndVal">0</span></div>
	<input type="range" id="crawlRhEnd" min="0" max="180" value="0" oninput="onCrawlParamChange()">
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col" style="flex:2">
	<div class="lbl"><span>频率</span><span id="crawlFreqVal">0.5 Hz</span></div>
	<input type="range" id="crawlFreq" min="5" max="300" value="100" oninput="onCrawlParamChange()">
	</div>
	<div class="col" style="flex:0 0 70px">
	<input id="crawlFreqNum" type="number" min="0.05" max="3.0" step="0.05" value="1.0" style="text-align:center" oninput="onCrawlFreqNum()">
	<label style="font-size:9px;color:#888">Hz</label>
	</div>
	</div>
	<div class="hint"><span style="color:#e67e22" id="crawlInfo">周期 2.0s</span></div>
<div class="row" style="margin-top:6px">
<button class="btn btn-on" id="btnCrawl" onpointerdown="toggleCrawl()">▶ 开始爬行</button>
<button class="btn btn-play" onpointerdown="crawlStep()">单步爬行</button>
<button class="btn btn-stop" onpointerdown="stopCrawl()">⏹ 停止</button>
	<button class="btn" id="btnAutoPlay" style="background:#2ecc71;color:#000" onpointerdown="toggleAutoPlay()">🤖 自运行动画: ON</button>
</div>
<div id="crawlStatus" style="font-size:11px;color:#e67e22;margin-top:4px;min-height:16px"></div>
	<!-- Battery -->
	<div style="margin-top:6px;display:flex;align-items:center;gap:8px">
	<span style="font-size:11px;color:#aaa">🔋 电量</span>
	<span id="batteryLevel" style="font-size:16px;font-weight:bold;color:#4ecca3">--%</span>
	<span id="batteryMv" style="font-size:10px;color:#888">--mV</span>
	</div>
</div>

<!-- PRESETS -->
	<div class="card">
	<h2>快捷预设</h2>
	<div class="row">
	<button class="btn btn-reset" onclick="preset(90,90,90)">全部休息</button>
	<button class="btn btn-reset" onclick="preset(90,180,0)">爬行姿势</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,180,90,0)">前进(交替)</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('both',90,180,90,0)">前进(同时)</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,30,90,180)">左转</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,180,90,30)">右转</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('lh',90,180,90,90)">左单臂</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('rh',90,90,90,0)">右单臂</button>
	</div>
	</div>

<!-- RECORDING -->
<div class="card">
<h2>录制设置</h2>
<div class="row">
<div class="col"><label style="font-size:11px">时长(秒)</label>
<input id="duration" type="number" value="3" min="1" max="30" onchange="onDurChange()"></div>
<div class="col"><label style="font-size:11px">步数</label>
<input id="steps" type="number" value="30" min="1" max="300" readonly></div>
</div>
<div class="row" style="margin-top:6px">
<input id="name" placeholder="动作名称(英文)" style="flex:1" value="my_action">
<button class="btn btn-rec" id="btnRec" onpointerdown="toggleRecord()">开始录制</button>
</div>
<div class="hint">录制时每100ms记录一帧 (3通道: 头/左手/右手)</div>
</div>

<div class="card" id="outputCard" style="display:none">
<h2>生成代码 <button class="btn btn-copy" onpointerdown="copyCode()">复制</button>
<button class="btn btn-play" onpointerdown="playRecord()">播放</button></h2>
<pre id="code"></pre>
<div class="row"><span style="font-size:10px;color:#888" id="info"></span></div>
</div>

<script>
let recording = false, frames = [], timer = null;
let crawlRunning = false;
let crawlTimer = null;

// REST positions
const REST_H = 90, REST_LH = 90, REST_RH = 90;

function $(id){return document.getElementById(id)}

function onDurChange(){
  let sec = parseInt($('duration').value) || 3;
  $('steps').value = sec * 10;
}
onDurChange();

// ===== API =====
async function api(url, body){
  try{
    let o = body ? {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)} : {method:'GET'};
    let r = await fetch(url, o);
    return await r.text();
  }catch(e){ setStatus('连接失败','#e94560'); return null; }
}

// ===== Get/send angles (3 channels) =====
function getAngles(){
  return [
    parseInt($('s0').value),  // Head
    parseInt($('s1').value),  // LH
    parseInt($('s2').value)   // RH
  ];
}

async function sendAngles(){
  let a = getAngles();
  $('v0').textContent = a[0]+'°';
  $('v1').textContent = a[1]+'°';
  $('v2').textContent = a[2]+'°';
  $('v1j').textContent = a[1]+'°';
  $('v2j').textContent = a[2]+'°';
  await api('/api/servo', {angles:a});
  if(recording) frames.push([...a]);
}

// ===== Hands Joystick (crawl control) =====
class HandsJoystick {
  constructor(canvasId){
    this.canvas = $(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.cx = 90; this.cy = 90; this.r = 78;
    this.kx = 90; this.ky = 90; this.kr = 22;
    this.lh = REST_LH; this.rh = REST_RH;
    this.active = false;
    this.animId = 0;
    this.canvas.addEventListener('pointerdown', e => { cancelAnimationFrame(this.animId); this.active = true; this.move(e); });
    this.canvas.addEventListener('pointermove', e => { if(this.active) this.move(e); });
    this.canvas.addEventListener('pointerup', () => { this.active = false; this.springBack(); });
    this.canvas.addEventListener('pointerleave', () => { this.active = false; this.springBack(); });
    this.canvas.addEventListener('pointercancel', () => { this.active = false; this.springBack(); });
    this.draw();
  }
  move(e){
    e.preventDefault();
    let rect = this.canvas.getBoundingClientRect();
    let sx = this.canvas.width / rect.width;
    let sy = this.canvas.height / rect.height;
    let mx = (e.clientX - rect.left) * sx;
    let my = (e.clientY - rect.top) * sy;
    let dx = mx - this.cx, dy = my - this.cy;
    let dist = Math.sqrt(dx*dx + dy*dy);
    if(dist > this.r) { dx *= this.r / dist; dy *= this.r / dist; }
    this.kx = this.cx + dx;
    this.ky = this.cy + dy;
    this.updateAngles();
    this.draw();
  }
  springBack(){
    let ease = t => t < 0.5 ? 4*t*t*t : 1 - Math.pow(-2*t + 2, 3) / 2;
    let sx = this.kx, sy = this.ky;
    let dx = this.cx - sx, dy = this.cy - sy;
    if(Math.abs(dx) < 0.3 && Math.abs(dy) < 0.3){
      this.kx = this.cx; this.ky = this.cy;
      this.lh = REST_LH; this.rh = REST_RH;
      this.updateAngles();
      this.draw();
      this.animId = 0;
      return;
    }
    let t0 = performance.now();
    let duration = 180;
    let step = (now) => {
      let t = Math.min(1, (now - t0) / duration);
      this.kx = sx + dx * ease(t);
      this.ky = sy + dy * ease(t);
      this.updateAngles();
      this.draw();
      if(t < 1) this.animId = requestAnimationFrame(step);
      else {
        this.kx = this.cx; this.ky = this.cy;
        this.lh = REST_LH; this.rh = REST_RH;
        this.updateAngles();
        this.draw();
        this.animId = 0;
      }
    };
    this.animId = requestAnimationFrame(step);
  }
	  updateAngles(){
	    // Y: +1=top(lh=0/rh=180), -1=bottom(lh=180/rh=0 full crawl)
	    let dy = -(this.ky - this.cy) / this.r;
	    let dx = (this.kx - this.cx) / this.r;

	    // Base position from Y axis
	    let lhBase = REST_LH - Math.round(dy * 90);  // dy=+1->0, dy=0->90, dy=-1->180
	    let rhBase = REST_RH + Math.round(dy * 90);  // dy=+1->180, dy=0->90, dy=-1->0

	    // X-axis steering: reduces one arm displacement from rest
	    // X<0 (left): left arm stays closer to rest -> turns left
	    // X>0 (right): right arm stays closer to rest -> turns right
	    let lhDisp = lhBase - REST_LH;
	    let rhDisp = rhBase - REST_RH;
	    if(dx < 0) { lhDisp *= (1 + dx); }
	    if(dx > 0) { rhDisp *= (1 - dx); }

	    this.lh = REST_LH + Math.round(lhDisp);
	    this.rh = REST_RH + Math.round(rhDisp);
	    this.lh = Math.max(0, Math.min(180, this.lh));
	    this.rh = Math.max(0, Math.min(180, this.rh));

	    $('s1').value = this.lh; $('s2').value = this.rh;
	    $('v1').textContent = this.lh + '°'; $('v2').textContent = this.rh + '°';
	    $('v1j').textContent = this.lh + '°'; $('v2j').textContent = this.rh + '°';
	    sendAngles();
	  }
	  draw() {
	    let c = this.ctx, w = this.canvas.width, h = this.canvas.height;
    c.clearRect(0, 0, w, h);
    c.beginPath(); c.arc(this.cx, this.cy, this.r, 0, Math.PI*2);
    c.strokeStyle = 'rgba(255,255,255,0.2)'; c.lineWidth = 2; c.stroke();
    c.beginPath(); c.moveTo(this.cx-this.r, this.cy); c.lineTo(this.cx+this.r, this.cy);
    c.moveTo(this.cx, this.cy-this.r); c.lineTo(this.cx, this.cy+this.r);
    c.strokeStyle = 'rgba(255,255,255,0.08)'; c.lineWidth = 1; c.stroke();
    c.font = '10px monospace';
    c.fillStyle = '#4ecca3'; c.fillText('0°上', this.cx-10, this.cy-this.r+12);
    c.fillStyle = '#e67e22'; c.fillText('180°爬', this.cx-10, this.cy+this.r-4);
    c.fillStyle = '#888'; c.fillText('LH', this.cx-this.r+2, this.cy+4);
    c.fillText('RH', this.cx+this.r-16, this.cy+4);
    c.beginPath(); c.arc(this.cx, this.cy, 4, 0, Math.PI*2);
    c.fillStyle = 'rgba(255,255,255,0.1)'; c.fill();
    c.beginPath(); c.arc(this.kx, this.ky, this.kr, 0, Math.PI*2);
    let grad = c.createRadialGradient(this.kx-5, this.ky-5, 3, this.kx, this.ky, this.kr);
    grad.addColorStop(0, '#f39c12'); grad.addColorStop(1, '#e67e22');
    c.fillStyle = grad; c.fill();
    c.strokeStyle = 'rgba(255,255,255,0.4)'; c.lineWidth = 2; c.stroke();
  }
	  setAngles(lh, rh){
	    cancelAnimationFrame(this.animId);
	    let lhOff = (lh - REST_LH) / 90;  // -1(0) to +1(180)
	    let rhOff = (rh - REST_RH) / 90;  // -1(0) to +1(180)
	    // Y from the arm that deviates more from rest
	    let dy = (Math.abs(lhOff) > Math.abs(rhOff)) ? -lhOff : rhOff;
	    // X: which arm is closer to rest = steering toward that side
	    let absL = Math.abs(lhOff), absR = Math.abs(rhOff);
	    let dx = (absL < absR) ? -(1 - absL / Math.max(0.01, absR))
	           : (absR < absL) ? (1 - absR / Math.max(0.01, absL))
	           : 0;
	    dy = Math.max(-1, Math.min(1, dy));
	    dx = Math.max(-1, Math.min(1, dx));
	    this.kx = this.cx + dx * this.r;
	    this.ky = this.cy + dy * this.r;
	    this.lh = lh; this.rh = rh;
	    this.draw();
	  }
}
let handsJoy = new HandsJoystick('joyHands');
// ===== Slider handler =====
function onSlider(){ sendAngles(); }

// ===== Single servo setters =====
function setHead(a){ $('s0').value = a; sendAngles(); }
function setLH(a){ $('s1').value = a; handsJoy.setAngles(a, parseInt($('s2').value)); sendAngles(); }
function setRH(a){ $('s2').value = a; handsJoy.setAngles(parseInt($('s1').value), a); sendAngles(); }

function preset(h, lh, rh){
  $('s0').value = h; $('s1').value = lh; $('s2').value = rh;
  handsJoy.setAngles(lh, rh);
  sendAngles();
}

// ===== CRAWLING: arm sweeps from start to end angle =====
// ===== CRAWLING: sin² continuous time-based (same as C++ auto-run) =====
// Compute [lh, rh] at a given phase [0, 1) in the crawl cycle
function calcCrawlAngles(phase){
  let mode = window._crawlMode;
  let lhS = window._crawlLhS, lhE = window._crawlLhE;
  let rhS = window._crawlRhS, rhE = window._crawlRhE;

  // sin² ease: smooth 0→1→0, no hold frames needed
  let val_lh = Math.sin(phase * Math.PI);
  val_lh = val_lh * val_lh;

  let phase_rh = (mode === 'alt') ? (phase + 0.5) % 1 : phase;
  let val_rh = Math.sin(phase_rh * Math.PI);
  val_rh = val_rh * val_rh;

  let lh, rh;
  if (mode === 'lh') {
    lh = lhS + (lhE - lhS) * val_lh;
    rh = REST_RH;
  } else if (mode === 'rh') {
    lh = REST_LH;
    rh = rhS + (rhE - rhS) * val_rh;
  } else {
    lh = lhS + (lhE - lhS) * val_lh;
    rh = rhS + (rhE - rhS) * val_rh;
  }
  return [Math.round(lh), Math.round(rh)];
}

function onCrawlParamChange(){
  let hzX100 = parseInt($('crawlFreq').value);
  let hz = hzX100 / 100;
  let lhS = parseInt($('crawlLhStart').value);
  let lhE = parseInt($('crawlLhEnd').value);
  let rhS = parseInt($('crawlRhStart').value);
  let rhE = parseInt($('crawlRhEnd').value);
  let cycleTime = (1 / hz).toFixed(2);

  $('crawlFreqVal').textContent = hz.toFixed(2) + ' Hz';
  $('crawlFreqNum').value = hz.toFixed(1);
  $('crawlLhStartVal').textContent = lhS;
  $('crawlLhEndVal').textContent = lhE;
  $('crawlRhStartVal').textContent = rhS;
  $('crawlRhEndVal').textContent = rhE;
  $('crawlInfo').textContent = '周期 ' + cycleTime + 's | L:'+lhS+'->'+lhE+' R:'+rhS+'->'+rhE;

  window._crawlMode = $('crawlMode').value;
  window._crawlLhS = lhS; window._crawlLhE = lhE;
  window._crawlRhS = rhS; window._crawlRhE = rhE;
  window._crawlPeriodS = 1 / hz;
}

function onCrawlPreset(mode, lhS, lhE, rhS, rhE){
  $('crawlMode').value = mode;
  $('crawlLhStart').value = lhS; $('crawlLhEnd').value = lhE;
  $('crawlRhStart').value = rhS; $('crawlRhEnd').value = rhE;
  onCrawlParamChange();
  if(crawlRunning) stopCrawl();
  toggleCrawl();
}

function onCrawlFreqNum(){
  let hz = parseFloat($('crawlFreqNum').value);
  if(isNaN(hz) || hz < 0.05) hz = 0.05;
  if(hz > 3.0) hz = 3.0;
  $('crawlFreq').value = Math.round(hz * 100);
  onCrawlParamChange();
}

// Default crawl state
window._crawlMode = 'alt';
window._crawlLhS = 90; window._crawlLhE = 180;
window._crawlRhS = 90; window._crawlRhE = 0;
window._crawlPeriodS = 1.0; // 1 Hz default
onCrawlParamChange();

async function toggleCrawl(){
  if(crawlRunning){ stopCrawl(); return; }
  crawlRunning = true;
  $('btnCrawl').textContent = '⏸ 停止爬行';
  $('btnCrawl').className = 'btn btn-stop';
  let mode = $('crawlMode').value;
  let modeNames = {alt:'交替', both:'同时', lh:'左单臂转弯', rh:'右单臂转弯'};
  $('crawlStatus').textContent = '爬行中: ' + (modeNames[mode]||mode);
  await runCrawlLoop();
}

async function runCrawlLoop(){
  const TICK_MS = 50; // 20Hz update rate
  let t0 = performance.now();

  while (crawlRunning) {
    let elapsed = (performance.now() - t0) / 1000;
    let phase = (elapsed / window._crawlPeriodS) % 1;

    let [lh, rh] = calcCrawlAngles(phase);
    $('s1').value = lh; $('s2').value = rh;
    handsJoy.setAngles(lh, rh);
    await api('/api/servo', {angles: [parseInt($('s0').value), lh, rh]});

    await new Promise(r => { crawlTimer = setTimeout(r, TICK_MS); });
  }
}

function stopCrawl(){
  crawlRunning = false;
  if(crawlTimer) clearTimeout(crawlTimer);
  $('btnCrawl').textContent = '▶ 开始爬行';
  $('btnCrawl').className = 'btn btn-on';
  $('crawlStatus').textContent = '已停止';
}

async function crawlStep(){
  if (crawlRunning) stopCrawl();
  const TICK_MS = 50;
  let t0 = performance.now();
  let period = window._crawlPeriodS;

  while (true) {
    let elapsed = (performance.now() - t0) / 1000;
    if (elapsed >= period) break;
    let phase = elapsed / period;

    let [lh, rh] = calcCrawlAngles(phase);
    $('s1').value = lh; $('s2').value = rh;
    handsJoy.setAngles(lh, rh);
    await api('/api/servo', {angles: [parseInt($('s0').value), lh, rh]});

    await new Promise(r => setTimeout(r, TICK_MS));
  }
  preset(REST_H, REST_LH, REST_RH);
}

let nodding = false;

async function runHeadNod(){
  if(nodding){ nodding = false; return; }
  nodding = true;
  while(nodding){
    let t = ((new Date()).getTime() / 1000) % 2 / 2;
    let a = Math.round(REST_H + Math.sin(t * Math.PI * 2) * 20);
    $('s0').value = a;
    sendAngles();
    await new Promise(r => setTimeout(r, 50));
  }
  $('s0').value = REST_H;
  sendAngles();
}

// ===== Record (3 channels) =====
async function toggleRecord(){
  if(!recording){
    let name = $('name').value.trim();
    if(!name){ setStatus('请输入动作名称','#e94560'); return; }
    frames = [];
    recording = true;
    $('btnRec').style.background = '#555';
    $('outputCard').style.display = 'none';
    setStatus('录制中... 摇动摇杆做动作','#4ecca3');
    let step = 0;
    let total = parseInt($('steps').value) || 30;
    frames.push(getAngles());
    step++;
    setStatus('录制中... '+step+'/'+total,'#4ecca3');
    timer = setInterval(async () => {
      let a = getAngles();
      frames.push([...a]);
      step++;
      setStatus('录制中... '+step+'/'+total,'#4ecca3');
      if(step >= total) toggleRecord();
    }, 100);
  } else {
    recording = false;
    if(timer) clearInterval(timer);
    timer = null;
    $('btnRec').style.background = '#e94560';
    let total = parseInt($('steps').value) || 30;
    if(frames.length > total) frames = frames.slice(0, total);
    while(frames.length < total) frames.push([...frames[frames.length-1]]);
    generateCode();
    setStatus('录制完成: '+frames.length+'帧','#4ecca3');
  }
}

function capitalize(s){
  return s.charAt(0).toUpperCase() + s.slice(1).replace(/[^a-zA-Z0-9_]/g,'_');
}

function generateCode(){
  let name = $('name').value.trim() || 'my_action';
  let totalSteps = frames.length;
  let duration = (totalSteps * 0.1).toFixed(1);
  let code = '// action_' + name + ': ' + totalSteps + ' steps = ' + duration + 's\n';
  code += '// [Head, LeftHand, RightHand]\n';
  code += 'static const ServoActionStep<3> kAction' + capitalize(name) + 'Steps[] = {\n';
  let lines = [];
  for(let i=0; i<frames.length; i++){
    let a = frames[i];
    lines.push('    {{' + a[0] + ',' + a[1] + ',' + a[2] + '}}');
  }
  code += lines.join(',\n') + ',\n';
  code += '};\n';
  code += 'static const ServoActionSeries kAction' + capitalize(name) + 'Series[] = {\n';
  code += '    {kAction' + capitalize(name) + 'Steps[0].angles, ' + totalSteps + '},\n';
  code += '};\n';
  $('outputCard').style.display = 'block';
}

function copyCode(){
  navigator.clipboard.writeText(code).then(()=>setStatus('已复制到剪贴板!','#4ecca3'));
}

async function playRecord(){
  if(frames.length==0) return;
  setStatus('播放中...','#ff6b35');
  for(let i=0; i<frames.length; i++){
    let a = frames[i];
    $('s0').value=a[0]; $('s1').value=a[1]; $('s2').value=a[2];
    await api('/api/servo', {angles:a});
    handsJoy.setAngles(a[1], a[2]);
    await new Promise(r=>setTimeout(r,100));
  }
  setStatus('播放完毕','#4ecca3');
}

// ===== Battery polling =====
async function updateBattery(){
  let r = await api('/api/battery');
  if(r){
    try{
      let b = JSON.parse(r);
      $('batteryLevel').textContent = b.level + '%';
      $('batteryMv').textContent = (b.voltage_mv/1000).toFixed(2) + 'V';
      let c = b.level > 20 ? '#4ecca3' : '#e94560';
      $('batteryLevel').style.color = c;
    }catch(e){}
  }
}
setInterval(updateBattery, 5000);
updateBattery();

// ===== Auto-play toggle (ESP32 built-in auto-run) =====
async function toggleAutoPlay(){
  let r = await api('/api/autoplay', {});
  if(r){
    try{
      let s = JSON.parse(r);
      updateAutoPlayBtn(s.autoplay ? 'ON' : 'OFF');
    }catch(e){}
  }
}
function updateAutoPlayBtn(state){
  let btn = $('btnAutoPlay');
  btn.textContent = '🤖 自运行动画: ' + state;
  btn.style.background = state === 'ON' ? '#2ecc71' : '#555';
  btn.style.color = state === 'ON' ? '#000' : '#fff';
}
// Poll auto-play state on load
(async function(){
  let r = await api('/api/autoplay');
  if(r){
    try{
      let s = JSON.parse(r);
      updateAutoPlayBtn(s.autoplay ? 'ON' : 'OFF');
    }catch(e){}
  }
})();

// Bind head nod button reliably
</script>
</body>
</html>
)raw";

// === HTTP handlers ===
static esp_err_t HandleRoot(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_send(req, kHtml, strlen(kHtml));
  return ESP_OK;
}

#if ENABLE_AUTO_RUN
static volatile bool auto_run_running_ = AUTO_RUN_DEFAULT_ON;
#endif

static esp_err_t HandleServo(httpd_req_t *req)
{
  char buf[256] = {};
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0)
  {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  buf[ret] = 0;

  // Parse {"angles":[head, lh, rh]}
  int angles[3] = {90, 90, 90};
  const char *p = strstr(buf, "\"angles\":");
  if (p)
  {
    p += 9;
    for (int i = 0; i < kServoCount; i++)
    {
      while (*p == ' ' || *p == '[' || *p == ',')
        p++;
      angles[i] = atoi(p);
      while (*p && *p != ',' && *p != ']')
        p++;
    }
  }
  for (int i = 0; i < kServoCount; i++)
    SetServoAngle(i, angles[i]);

#if ENABLE_AUTO_RUN
  // Manual control pauses auto-run to avoid fighting over servos
  if (auto_run_running_)
  {
    auto_run_running_ = false;
    ESP_LOGI(TAG, "Auto-play paused by manual servo command");
  }
#endif

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

static esp_err_t HandleBattery(httpd_req_t *req)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"voltage_mv\":%d,\"level\":%d}",
           battery_vbat_filtered_mv_, battery_level_);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, buf);
  return ESP_OK;
}

#if ENABLE_AUTO_RUN
static esp_err_t HandleAutoPlay(httpd_req_t *req);
#endif

static void StartHttpServer()
{
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 8;
  httpd_handle_t server = nullptr;
  httpd_start(&server, &cfg);

  httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = HandleRoot, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &root);

  httpd_uri_t servo = {.uri = "/api/servo", .method = HTTP_POST, .handler = HandleServo, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &servo);

  httpd_uri_t battery = {.uri = "/api/battery", .method = HTTP_GET, .handler = HandleBattery, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &battery);

#if ENABLE_AUTO_RUN
  httpd_uri_t autoplay_get = {.uri = "/api/autoplay", .method = HTTP_GET, .handler = HandleAutoPlay, .user_ctx = nullptr};
  httpd_uri_t autoplay_post = {.uri = "/api/autoplay", .method = HTTP_POST, .handler = HandleAutoPlay, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &autoplay_get);
  httpd_register_uri_handler(server, &autoplay_post);
#endif

  ESP_LOGI(TAG, "HTTP server started on http://192.168.4.1");
}

#if ENABLE_AUTO_RUN
// === Auto-run continuous motion task ===
// Toggle via /api/autoplay
// Head: sinusoidal nod around 90° center
// Arms: alternates between alternating/simultaneous crawl every 8s
//   Alternating: LH↔RH 180° out of phase (walking gait)
//   Simultaneous: both arms sweep together
// Mode switches use a 1s smoothstep crossfade so the arms never jerk.
static void AutoRunTask(void *arg)
{
  constexpr int kTickMs = 20;
  constexpr int kHeadCenter = 90;
  constexpr int kHeadAmplitude = 20;
  constexpr float kHeadPeriodS = 1.5f;

  constexpr int kLhStart = 90;
  constexpr int kLhEnd = 30;
  constexpr int kRhStart = 90;
  constexpr int kRhEnd = 180;
  constexpr float kCrawlPeriodS = 2.0f;
  constexpr int kModeSwitchS = 8;
  constexpr float kBlendDurationS = 1.0f; // crossfade between modes

  uint32_t tick = 0;
  uint32_t next_switch_tick = (uint32_t)(kModeSwitchS * 1000 / kTickMs);
  bool alt_mode = true; // start with alternating crawl

  // Crossfade state
  bool blend_active = false;
  bool blend_from_alt = false;
  uint32_t blend_start_tick = 0;

  // Helper: compute LH+RH angles for a given mode at a given continuous phase
  auto calcCrawl = [](float phase, bool is_alt, int &lh, int &rh) {
    float val_lh = sinf(phase * M_PI);
    val_lh = val_lh * val_lh; // sin²: 0→1→0
    float phase_rh = is_alt ? (phase + 0.5f) : phase;
    phase_rh = phase_rh - floorf(phase_rh);
    float val_rh = sinf(phase_rh * M_PI);
    val_rh = val_rh * val_rh;
    lh = kLhStart + (int)((kLhEnd - kLhStart) * val_lh);
    rh = kRhStart + (int)((kRhEnd - kRhStart) * val_rh);
  };

  ESP_LOGI(TAG, "Auto-run started: head nod ±%d°, crawl alt/both %ds each, blend %.1fs",
           kHeadAmplitude, kModeSwitchS, kBlendDurationS);

  while (true)
  {
    if (!auto_run_running_)
    {
      // Just sleep, don't touch servos — let manual control take over
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    float t_s = tick * kTickMs / 1000.0f;

    // --- Head nodding: sinusoidal around center (always smooth) ---
    float head_phase = t_s / kHeadPeriodS;
    int head_angle = kHeadCenter + (int)(kHeadAmplitude * sinf(head_phase * 2.0f * M_PI));
    SetServoAngle(0, head_angle);

    // --- Mode switching: fire every kModeSwitchS, never reset phase ---
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

    // --- Crawl: continuous phase from tick (no jump on mode switch) ---
    float phase = (tick * kTickMs / 1000.0f) / kCrawlPeriodS;
    phase = phase - floorf(phase); // fractional part [0, 1)

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
        // Smoothstep crossfade: old mode → new mode
        float t = blend_t / kBlendDurationS;
        float mix = t * t * (3.0f - 2.0f * t); // smoothstep ease

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

static esp_err_t HandleAutoPlay(httpd_req_t *req)
{
  if (req->method == HTTP_POST)
  {
    char buf[32] = {};
    httpd_req_recv(req, buf, sizeof(buf) - 1);
    // Toggle: POST without body toggles, or pass {"enable":0/1}
    const char *p = strstr(buf, "\"enable\":");
    if (p)
    {
      p += 9;
      auto_run_running_ = (atoi(p) != 0);
    }
    else
    {
      auto_run_running_ = !auto_run_running_;
    }
  }
  // GET or POST response: return current state
  char resp[64];
  snprintf(resp, sizeof(resp), "{\"autoplay\":%s}", auto_run_running_ ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, resp);
  ESP_LOGI(TAG, "Auto-play %s", auto_run_running_ ? "ON" : "OFF");
  return ESP_OK;
}

static void InitAutoRun()
{
  xTaskCreate(AutoRunTask, "auto_run", 4096, nullptr, 2, nullptr);
  ESP_LOGI(TAG, "Auto-run task created");
}
#endif  // ENABLE_AUTO_RUN

extern "C" void app_main()
{
  nvs_flash_init();
  InitPower(); // Must be first: latch IO7, monitor IO6
  InitServos();
#if ENABLE_AUTO_RUN
  InitAutoRun(); // Start continuous head nod + crawl on power-up
#endif
  InitWiFi();
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for AP to start
  StartHttpServer();
}
