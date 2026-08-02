/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of
 * the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in
 * writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**********************************************************
 * main/tests/test_lidar_viewer.c
 *
 * LiDAR "Pin Art" web viewer test
 *
 * Brings the VL53L5CX sensor(s) up, then gets WiFi connected before
 * handing off to components/services/lidar_web_viewer for the HTTP
 * server and polling task. This file is glue only — see
 * lidar_web_viewer.h for what the service itself owns.
 *
 * How WiFi comes up depends on the general MAIA_WIFI_MODE choice
 * (WiFi Configuration menu — not a setting local to this test):
 *
 *   MAIA_WIFI_MODE_AP           - maia_wifi_ap_start(): plain
 *                                  diagnostic AP, join it directly
 *                                  from a browser, no pairing app,
 *                                  and NVS-independent (see the
 *                                  comment at that call site — this
 *                                  is the mode to use for a quick
 *                                  bench test).
 *   anything else (STA / AUTO /
 *   PROVISIONING)               - wifi_prov_service_run(): connects
 *                                  directly if already provisioned
 *                                  (stored creds in NVS), otherwise
 *                                  raises the interactive
 *                                  provisioning SoftAP and needs the
 *                                  ESP SoftAP Prov app to pair.
 *
 **********************************************************/

/* main/CMakeLists.txt compiles this file whenever both
 * CONFIG_MAIA_VL53L5CX_ENABLE and CONFIG_MAIA_WIFI_ENABLE are
 * set (same device-gated convention as every other test in this
 * directory) — regardless of which specific test is actually
 * selected. CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT below, however,
 * lives in a menu gated on MAIA_TEST_LIDAR_VIEWER specifically
 * (see components/maia_board/Kconfig), so it only exists in
 * sdkconfig.h when this test is the one selected. The body below
 * is guarded on that narrower symbol — not just the two device
 * flags — so it compiles to an empty translation unit instead of
 * failing on an undeclared identifier when some other MAIA_TEST_*
 * is selected.
 *
 * The #if guard has to come AFTER the #include block below, not
 * before: CONFIG_MAIA_TEST_LIDAR_VIEWER only becomes visible to the
 * preprocessor once sdkconfig.h has been pulled in transitively
 * through those includes (tests.h is what actually includes it).
 * Guarding earlier makes defined(CONFIG_...) always false and
 * silently compiles the whole file to nothing — this is exactly the
 * bug that shipped here initially and produced a link error
 * ("undefined reference to test_lidar_viewer_run") despite the test
 * being selected in menuconfig. Same fix applied to
 * components/services/lidar_web_viewer/src/lidar_web_viewer.c.
 */

/**********************************************************
 * Included Files
 **********************************************************/

#include "tests.h"
#include "vl53l5cx.h"
#include "maia_wifi.h"
#include "wifi_prov_service.h"
#include "lidar_web_viewer.h"

#include <esp_log.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if defined(CONFIG_MAIA_TEST_LIDAR_VIEWER)

/**********************************************************
 * Pre-processor Definitions
 **********************************************************/

#define TAG  "[TEST_LIDAR_VIEWER]"

/**********************************************************
 * Public Functions
 **********************************************************/

/**********************************************************
 * Name: test_lidar_viewer_run
 *
 * Description:
 *   Entry point called from main.c when
 *   MAIA_TEST_LIDAR_VIEWER is selected. Runs until reset.
 *
 **********************************************************/

void test_lidar_viewer_run(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Starting LiDAR Pin Art web viewer test");

  ret = maia_tof_init();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_tof_init failed: %s", esp_err_to_name(ret));
      return;
    }

  ret = maia_tof_start_ranging();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_tof_start_ranging failed: %s",
               esp_err_to_name(ret));
      return;
    }

  ret = maia_wifi_init(NULL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_wifi_init failed: %s", esp_err_to_name(ret));
      return;
    }

#if defined(CONFIG_MAIA_WIFI_MODE_AP)

  /* MAIA_WIFI_MODE_AP: bench/diagnostic mode. Raise the plain
   * diagnostic AP directly instead of going through
   * wifi_prov_service_run() — that call only ever does one of two
   * things: connect with whatever station credentials are already
   * stored in NVS from a *previous* session (unrelated to this
   * Kconfig choice, and possibly a network that isn't in range right
   * now — see maia_wifi_is_provisioned()), or raise the interactive
   * *provisioning* AP, which needs the ESP SoftAP Prov phone app to
   * pair. Neither of those is "just raise a known AP and let a
   * browser join it directly", which is what selecting AP mode here
   * means. maia_wifi_ap_start() does exactly that and never reads
   * NVS, so this path is unaffected by whatever is stored there.
   */

  ESP_LOGI(TAG, "MAIA_WIFI_MODE_AP: raising diagnostic AP (no pairing "
                "app needed, and independent of any stored WiFi "
                "credentials)");

  ret = maia_wifi_ap_start();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "ap_start failed: %s", esp_err_to_name(ret));
      return;
    }

  ret = lidar_web_viewer_start();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "lidar_web_viewer_start failed: %s",
               esp_err_to_name(ret));
      return;
    }

  ESP_LOGI(TAG, "========================================================");
  ESP_LOGI(TAG, "  Join the AP printed above, then open:");
  ESP_LOGI(TAG, "  http://192.168.4.1:%d/", CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT);
  ESP_LOGI(TAG, "  (192.168.4.1 is esp_netif's default SoftAP gateway "
                "IP — see esp_netif_create_default_wifi_ap() in "
                "maia_wifi.c; nothing here overrides it)");
  ESP_LOGI(TAG, "========================================================");

#else /* MAIA_WIFI_MODE_STA / _AUTO / _PROVISIONING: join a real network */

  ESP_LOGI(TAG, "Connecting WiFi (wifi_prov_service_run) — if this is "
                "the first boot, watch the log for the SoftAP name and "
                "PoP code, and pair from the phone app");

  ret = wifi_prov_service_run();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "WiFi provisioning/connect failed: %s",
               esp_err_to_name(ret));
      return;
    }

  esp_netif_ip_info_t ip;

  ret = maia_wifi_sta_get_ip(&ip);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "sta_get_ip failed: %s", esp_err_to_name(ret));
      return;
    }

  ret = lidar_web_viewer_start();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "lidar_web_viewer_start failed: %s",
               esp_err_to_name(ret));
      return;
    }

  ESP_LOGI(TAG, "========================================================");
  ESP_LOGI(TAG, "  Open this on any browser on the same network:");
  ESP_LOGI(TAG, "  http://" IPSTR ":%d/", IP2STR(&ip.ip),
           CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT);
  ESP_LOGI(TAG, "========================================================");

#endif /* CONFIG_MAIA_WIFI_MODE_AP */

  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

#endif /* CONFIG_MAIA_TEST_LIDAR_VIEWER */
