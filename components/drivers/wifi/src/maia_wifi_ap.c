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
 * components/drivers/wifi/src/maia_wifi_ap.c
 *
 * MAIA WiFi Driver — L3 Access Point (diagnostic mode)
 *
 * This is a bench/diagnostic AP only — it exists to validate the radio
 * and antenna without a router nearby (spec/wifi_subsystem.md section
 * 14, SCAN sub-test). It serves no portal and accepts no credentials.
 * The provisioning setup AP is a separate concern owned by
 * components/services/wifi_prov_service/ once that component exists.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_wifi_priv.h"

#include <string.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_mac.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_WIFI_AP]"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ap_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: on_wifi_event
 ****************************************************************************/

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data)
{
  (void)arg;
  (void)base;

  switch (id)
    {
      case WIFI_EVENT_AP_START:
        maia_wifi_set_state(MAIA_WIFI_STATE_AP_RUNNING);
        maia_wifi_notify(MAIA_WIFI_EVENT_AP_STARTED, NULL);
        break;

      case WIFI_EVENT_AP_STACONNECTED:
        {
          wifi_event_ap_staconnected_t *evt = data;

          ESP_LOGI(TAG, "Client joined: " MACSTR, MAC2STR(evt->mac));
          maia_wifi_notify(MAIA_WIFI_EVENT_AP_CLIENT_JOINED, NULL);
        }
        break;

      case WIFI_EVENT_AP_STADISCONNECTED:
        {
          wifi_event_ap_stadisconnected_t *evt = data;

          ESP_LOGI(TAG, "Client left: " MACSTR, MAC2STR(evt->mac));
          maia_wifi_notify(MAIA_WIFI_EVENT_AP_CLIENT_LEFT, NULL);
        }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_ap_events_register
 *
 * Description:
 *   Internal — called once from maia_wifi_init().
 *
 ****************************************************************************/

esp_err_t maia_wifi_ap_events_register(void)
{
  return esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &on_wifi_event, NULL, NULL);
}

/****************************************************************************
 * Name: maia_wifi_ap_start
 ****************************************************************************/

esp_err_t maia_wifi_ap_start(void)
{
  esp_err_t ret;
  size_t pass_len = strlen(CONFIG_MAIA_WIFI_AP_PASSWORD);

  if (pass_len > 0 && pass_len < 8)
    {
      ESP_LOGE(TAG, "AP password must be empty (open) or >= 8 chars, "
                    "got %d", pass_len);
      return ESP_ERR_INVALID_ARG;
    }

  ret = esp_wifi_set_mode(WIFI_MODE_AP);
  if (ret != ESP_OK)
    {
      return ret;
    }

  wifi_config_t cfg = { 0 };

  strlcpy((char *)cfg.ap.ssid, CONFIG_MAIA_WIFI_AP_SSID_PREFIX,
          sizeof(cfg.ap.ssid));

#if CONFIG_MAIA_WIFI_AP_SSID_APPEND_MAC
  char suffix[5];

  if (maia_wifi_get_mac_suffix(suffix, sizeof(suffix)) == ESP_OK)
    {
      size_t base_len = strlen((char *)cfg.ap.ssid);
      size_t room = sizeof(cfg.ap.ssid) - base_len;

      snprintf((char *)cfg.ap.ssid + base_len, room, "-%s", suffix);
    }
#endif

  cfg.ap.ssid_len = strlen((char *)cfg.ap.ssid);
  cfg.ap.channel = CONFIG_MAIA_WIFI_AP_CHANNEL;
  cfg.ap.max_connection = CONFIG_MAIA_WIFI_AP_MAX_CONN;
  cfg.ap.authmode = pass_len > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

  if (pass_len > 0)
    {
      strlcpy((char *)cfg.ap.password, CONFIG_MAIA_WIFI_AP_PASSWORD,
              sizeof(cfg.ap.password));
    }

  ret = esp_wifi_set_config(WIFI_IF_AP, &cfg);
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = esp_wifi_start();
  if (ret != ESP_OK)
    {
      return ret;
    }

  g_ap_started = true;

  ESP_LOGI(TAG, "AP started: SSID=\"%s\" channel=%d", cfg.ap.ssid,
           cfg.ap.channel);

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_ap_stop
 ****************************************************************************/

esp_err_t maia_wifi_ap_stop(void)
{
  if (!g_ap_started)
    {
      return ESP_OK;
    }

  esp_wifi_stop();
  g_ap_started = false;
  maia_wifi_set_state(MAIA_WIFI_STATE_IDLE);

  return ESP_OK;
}
