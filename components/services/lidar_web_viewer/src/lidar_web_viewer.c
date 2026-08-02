/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
 * components/services/lidar_web_viewer/src/lidar_web_viewer.c
 *
 * See lidar_web_viewer.h for scope.
 *
 * The body below (everything past the #include block) is guarded by
 * CONFIG_MAIA_TEST_LIDAR_VIEWER, not just CONFIG_MAIA_VL53L5CX_ENABLE /
 * CONFIG_MAIA_WIFI_ENABLE — the Kconfig symbols read further down
 * (MAIA_LIDAR_VIEWER_HTTP_PORT, MAIA_LIDAR_VIEWER_DEFAULT_MAX_MM,
 * MAIA_VL53L5CX_RESOLUTION_*, MAIA_VL53L5CX_MODE_*) live in menus gated
 * on MAIA_TEST_LIDAR_VIEWER (which itself depends on both ENABLE
 * symbols — see the Kconfig help text), so only that one symbol
 * reliably tells us they exist in sdkconfig.h. Same reasoning as
 * wifi_prov_service.c's guard.
 *
 * The #if has to come AFTER the #include block, not before: sdkconfig.h
 * is only pulled in transitively through these includes, so a CONFIG_*
 * macro is not visible to the preprocessor yet at the top of the file —
 * guarding earlier than this silently compiles the whole file to
 * nothing regardless of Kconfig, since defined(CONFIG_...) is always
 * false before sdkconfig.h has been seen.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lidar_web_viewer.h"
#include "vl53l5cx.h"

#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#if defined(CONFIG_MAIA_TEST_LIDAR_VIEWER)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG  "[LIDAR_VIEWER]"

#if defined(CONFIG_MAIA_VL53L5CX_RESOLUTION_4X4)
#  define GRID_SIZE  4
#else
#  define GRID_SIZE  8
#endif

#define NB_ZONES  (GRID_SIZE * GRID_SIZE)

/* Only the "primary" (index 0) sensor slot is initialized by
 * maia_tof_init() unless MODE_DUAL is set — see the doc comment on
 * maia_tof_init() in vl53l5cx.h. In RIGHT_ONLY mode, index 0 is
 * physically the right sensor; label it accordingly so the page
 * doesn't claim data came from a sensor that was never brought up.
 */

#if defined(CONFIG_MAIA_VL53L5CX_MODE_DUAL)
#  define NUM_ACTIVE_SENSORS  2
#elif defined(CONFIG_MAIA_VL53L5CX_MODE_RIGHT_ONLY)
#  define NUM_ACTIVE_SENSORS  1
#  define SENSOR0_LABEL       "right"
#else
#  define NUM_ACTIVE_SENSORS  1
#  define SENSOR0_LABEL       "left"
#endif

#ifndef SENSOR0_LABEL
#  define SENSOR0_LABEL  "left"
#endif

#define POLL_INTERVAL_MS    30
#define POLL_TASK_STACK     4096
#define POLL_TASK_PRIORITY  5

/* [-32768,255], plus comma, per zone; generous margin over the
 * ~2 sensors * 64 zones this ever actually needs.
 */

#define JSON_BUF_SIZE  8192

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int16_t  distance_mm[NB_ZONES];
  uint8_t  target_status[NB_ZONES];
} sensor_frame_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sensor_frame_t     s_frame[NUM_ACTIVE_SENSORS];
static SemaphoreHandle_t  s_frame_mutex   = NULL;
static httpd_handle_t     s_httpd         = NULL;
static TaskHandle_t       s_poll_task     = NULL;
static volatile bool      s_poll_running  = false;
static char                s_json_buf[JSON_BUF_SIZE];

/* Embedded by EMBED_FILES in CMakeLists.txt — symbol names are derived
 * from the target filename (directory stripped), per ESP-IDF's
 * component-embed convention.
 */

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_task
 *
 * Description:
 *   Polls maia_tof_data_ready()/maia_tof_get_data() for each active
 *   sensor and caches the latest raw frame under s_frame_mutex. Runs
 *   until lidar_web_viewer_stop() clears s_poll_running, then deletes
 *   itself.
 *
 ****************************************************************************/

static void poll_task(void *arg)
{
  (void)arg;

  while (s_poll_running)
    {
      for (int i = 0; i < NUM_ACTIVE_SENSORS; i++)
        {
          bool ready = false;

          if (maia_tof_data_ready((uint8_t)i, &ready) != ESP_OK || !ready)
            {
              continue;
            }

          maia_tof_data_t data;

          if (maia_tof_get_data((uint8_t)i, &data) != ESP_OK)
            {
              continue;
            }

          xSemaphoreTake(s_frame_mutex, portMAX_DELAY);

          for (int z = 0; z < NB_ZONES && z < data.nb_zones; z++)
            {
              s_frame[i].distance_mm[z]    = data.distance_mm[z];
              s_frame[i].target_status[z]  = data.target_status[z];
            }

          xSemaphoreGive(s_frame_mutex);
        }

      vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }

  s_poll_task = NULL;
  vTaskDelete(NULL);
}

/****************************************************************************
 * Name: frame_to_json
 *
 * Description:
 *   Serializes the cached frame(s) into buf as:
 *
 *     {"grid":8,"max_mm_default":2000,
 *      "sensors":[{"id":"left","zones":[[d,s],[d,s],...]}]}
 *
 *   zones[] is row-major (index = row*GRID_SIZE + col), matching the
 *   order maia_tof_get_data() fills distance_mm[]/target_status[] in.
 *   d is the raw distance_mm (may be -1, meaning "filtered/no valid
 *   target" per vl53l5cx.h) and s is the raw target_status — nothing
 *   is clamped or reinterpreted here, see the SCOPE note in the
 *   header for why.
 *
 * Returned Value:
 *   Number of bytes written (excluding the NUL terminator).
 *
 ****************************************************************************/

static int frame_to_json(char *buf, size_t buf_len)
{
  int n = 0;

  n += snprintf(buf + n, buf_len - n,
                "{\"grid\":%d,\"max_mm_default\":%d,\"sensors\":[",
                GRID_SIZE, CONFIG_MAIA_LIDAR_VIEWER_DEFAULT_MAX_MM);

  xSemaphoreTake(s_frame_mutex, portMAX_DELAY);

  for (int i = 0; i < NUM_ACTIVE_SENSORS; i++)
    {
      const char *label = (i == 0) ? SENSOR0_LABEL : "right";

      n += snprintf(buf + n, buf_len - n, "%s{\"id\":\"%s\",\"zones\":[",
                    (i == 0) ? "" : ",", label);

      for (int z = 0; z < NB_ZONES; z++)
        {
          n += snprintf(buf + n, buf_len - n, "%s[%d,%d]",
                        (z == 0) ? "" : ",",
                        s_frame[i].distance_mm[z],
                        s_frame[i].target_status[z]);
        }

      n += snprintf(buf + n, buf_len - n, "]}");
    }

  xSemaphoreGive(s_frame_mutex);

  n += snprintf(buf + n, buf_len - n, "]}");

  return n;
}

/****************************************************************************
 * Name: root_get_handler
 *
 * Description:
 *   Serves the embedded web/index.html verbatim.
 *
 ****************************************************************************/

static esp_err_t root_get_handler(httpd_req_t *req)
{
  size_t len = (size_t)(index_html_end - index_html_start);

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)index_html_start, len);
}

/****************************************************************************
 * Name: frame_get_handler
 *
 * Description:
 *   Serves the current frame as JSON (see frame_to_json()).
 *
 ****************************************************************************/

static esp_err_t frame_get_handler(httpd_req_t *req)
{
  int len = frame_to_json(s_json_buf, sizeof(s_json_buf));

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, s_json_buf, len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

esp_err_t lidar_web_viewer_start(void)
{
  if (s_httpd != NULL)
    {
      ESP_LOGW(TAG, "Already running");
      return ESP_OK;
    }

  s_frame_mutex = xSemaphoreCreateMutex();
  if (s_frame_mutex == NULL)
    {
      return ESP_ERR_NO_MEM;
    }

  /* Seed every zone as "no reading yet" so the first page load and
   * the first few polls (before the sensor's first frame lands) show
   * a full grid of "sem leitura" pins instead of a truncated array.
   */

  memset(s_frame, 0, sizeof(s_frame));
  for (int i = 0; i < NUM_ACTIVE_SENSORS; i++)
    {
      for (int z = 0; z < NB_ZONES; z++)
        {
          s_frame[i].distance_mm[z]   = -1;
          s_frame[i].target_status[z] = 255;
        }
    }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.server_port   = CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT;
  config.stack_size    = 8192;
  config.lru_purge_enable = true;

  esp_err_t ret = httpd_start(&s_httpd, &config);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
      vSemaphoreDelete(s_frame_mutex);
      s_frame_mutex = NULL;
      return ret;
    }

  static const httpd_uri_t root_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = root_get_handler,
  };

  static const httpd_uri_t frame_uri = {
    .uri      = "/frame.json",
    .method   = HTTP_GET,
    .handler  = frame_get_handler,
  };

  httpd_register_uri_handler(s_httpd, &root_uri);
  httpd_register_uri_handler(s_httpd, &frame_uri);

  s_poll_running = true;
  xTaskCreate(poll_task, "lidar_viewer_poll", POLL_TASK_STACK, NULL,
              POLL_TASK_PRIORITY, &s_poll_task);

  ESP_LOGI(TAG, "Pin Art viewer serving on port %d (%dx%d grid, %d sensor(s))",
           CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT, GRID_SIZE, GRID_SIZE,
           NUM_ACTIVE_SENSORS);

  return ESP_OK;
}

esp_err_t lidar_web_viewer_stop(void)
{
  if (s_httpd == NULL)
    {
      return ESP_OK;
    }

  s_poll_running = false;

  /* poll_task self-deletes at its next loop check (<= POLL_INTERVAL_MS
   * away); give it a margin before tearing down what it reads/writes.
   */

  vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS * 3));

  httpd_stop(s_httpd);
  s_httpd = NULL;

  vSemaphoreDelete(s_frame_mutex);
  s_frame_mutex = NULL;

  return ESP_OK;
}

#endif /* CONFIG_MAIA_TEST_LIDAR_VIEWER */
