#include "config.h"
#include "auto_run.h"
#include "http_server.h"
#include "power.h"
#include "servo.h"
#include "wifi.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

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
