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
 *
 * ---------------------------------------------------------------------------
 * components/drivers/wifi/src/maia_wifi_sta.c
 *
 * MAIA WiFi Driver — L2 Station
 *
 * Pure mechanism: bring the interface up, attempt one association,
 * report what happened. No retry, no backoff, no give-up decision — see
 * the SCOPE note in maia_wifi.h for why that policy deliberately lives
 * in the caller (main/tests/test_wifi.c today) instead of here.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_wifi_priv.h"

#include <string.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_WIFI_STA]"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sta_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: on_wifi_event
 *
 * Description:
 *   Translate raw esp_wifi STA events into maia_wifi_event_t. No
 *   decision-making: a disconnect is reported with its raw reason code
 *   and the state is set back to IDLE — it is up to the caller whether
 *   and when to call maia_wifi_sta_connect() again.
 *
 ****************************************************************************/

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data)
{
  (void)arg;
  (void)base;

  switch (id)
    {
      case WIFI_EVENT_STA_CONNECTED:
        maia_wifi_notify(MAIA_WIFI_EVENT_STA_CONNECTED, NULL);
        break;

      case WIFI_EVENT_STA_DISCONNECTED:
        {
          wifi_event_sta_disconnected_t *evt = data;
          uint8_t reason = evt->reason;

          ESP_LOGW(TAG, "Disconnected, reason=%d", reason);
          maia_wifi_set_state(MAIA_WIFI_STATE_IDLE);
          maia_wifi_notify(MAIA_WIFI_EVENT_STA_DISCONNECTED, &reason);
        }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: on_ip_event
 ****************************************************************************/

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id,
                        void *data)
{
  (void)arg;
  (void)base;

  if (id == IP_EVENT_STA_GOT_IP)
    {
      maia_wifi_set_state(MAIA_WIFI_STATE_STA_CONNECTED);
      maia_wifi_notify(MAIA_WIFI_EVENT_STA_GOT_IP, NULL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_sta_events_register
 *
 * Description:
 *   Internal — called once from maia_wifi_init(). Registers the WiFi
 *   and IP event handlers.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_events_register(void)
{
  esp_err_t ret;

  ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &on_wifi_event, NULL, NULL);
  if (ret != ESP_OK)
    {
      return ret;
    }

  return esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &on_ip_event, NULL, NULL);
}

/****************************************************************************
 * Name: maia_wifi_sta_start
 ****************************************************************************/

esp_err_t maia_wifi_sta_start(const maia_wifi_creds_t *creds)
{
  esp_err_t ret;

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret != ESP_OK)
    {
      return ret;
    }

  if (creds != NULL)
    {
      wifi_config_t cfg = { 0 };

      strlcpy((char *)cfg.sta.ssid, creds->ssid, sizeof(cfg.sta.ssid));
      strlcpy((char *)cfg.sta.password, creds->password,
              sizeof(cfg.sta.password));
      cfg.sta.threshold.authmode = strlen(creds->password) > 0 ?
                                    WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

      ret = esp_wifi_set_config(WIFI_IF_STA, &cfg);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }
  else if (!maia_wifi_is_provisioned())
    {
      return ESP_ERR_INVALID_STATE;
    }

#if CONFIG_MAIA_WIFI_STA_POWER_SAVE
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
#else
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  ret = esp_wifi_start();
  if (ret != ESP_OK)
    {
      return ret;
    }

  g_sta_started = true;
  maia_wifi_set_state(MAIA_WIFI_STATE_IDLE);

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_sta_connect
 ****************************************************************************/

esp_err_t maia_wifi_sta_connect(void)
{
  esp_err_t ret;

  if (!g_sta_started)
    {
      return ESP_ERR_INVALID_STATE;
    }

  maia_wifi_set_state(MAIA_WIFI_STATE_STA_CONNECTING);

  ret = esp_wifi_connect();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(ret));
    }

  return ret;
}

/****************************************************************************
 * Name: maia_wifi_sta_stop
 ****************************************************************************/

esp_err_t maia_wifi_sta_stop(void)
{
  if (!g_sta_started)
    {
      return ESP_OK;
    }

  esp_wifi_disconnect();
  esp_wifi_stop();
  g_sta_started = false;
  maia_wifi_set_state(MAIA_WIFI_STATE_IDLE);

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_sta_get_ip
 ****************************************************************************/

esp_err_t maia_wifi_sta_get_ip(esp_netif_ip_info_t *ip)
{
  if (maia_wifi_get_state() != MAIA_WIFI_STATE_STA_CONNECTED)
    {
      return ESP_ERR_INVALID_STATE;
    }

  return esp_netif_get_ip_info(maia_wifi_sta_netif(), ip);
}

/****************************************************************************
 * Name: maia_wifi_sta_get_rssi
 ****************************************************************************/

esp_err_t maia_wifi_sta_get_rssi(int8_t *rssi)
{
  wifi_ap_record_t rec;
  esp_err_t ret;

  if (maia_wifi_get_state() != MAIA_WIFI_STATE_STA_CONNECTED)
    {
      return ESP_ERR_INVALID_STATE;
    }

  ret = esp_wifi_sta_get_ap_info(&rec);
  if (ret != ESP_OK)
    {
      return ret;
    }

  *rssi = rec.rssi;

  return ESP_OK;
}
