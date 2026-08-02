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
 * components/drivers/wifi/src/maia_wifi.c
 *
 * MAIA WiFi Driver — L1 Core
 * netif/event-loop setup, state machine, MAC suffix, AUTO boot decision.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_wifi.h"
#include "maia_wifi_priv.h"

#include <string.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <nvs_flash.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_WIFI]"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static maia_wifi_callback_t g_cb = NULL;
static maia_wifi_state_t    g_state = MAIA_WIFI_STATE_IDLE;
static bool                 g_initialized = false;
static esp_netif_t         *g_netif_sta = NULL;
static esp_netif_t         *g_netif_ap = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netif_ready
 *
 * Description:
 *   Lazily create the STA and AP netifs. Both are created once and kept
 *   around for the life of the driver — esp_netif_create_default_wifi_*
 *   is not meant to be called repeatedly across mode switches.
 *
 ****************************************************************************/

static void netif_ready(void)
{
  if (g_netif_sta == NULL)
    {
      g_netif_sta = esp_netif_create_default_wifi_sta();
    }

  if (g_netif_ap == NULL)
    {
      g_netif_ap = esp_netif_create_default_wifi_ap();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_notify
 *
 * Description:
 *   Internal — invoke the registered callback, if any. Shared by the
 *   sta/ap/scan translation units.
 *
 ****************************************************************************/

void maia_wifi_notify(maia_wifi_event_t event, void *data)
{
  if (g_cb != NULL)
    {
      g_cb(event, data);
    }
}

/****************************************************************************
 * Name: maia_wifi_set_state
 *
 * Description:
 *   Internal — set the driver state machine state.
 *
 ****************************************************************************/

void maia_wifi_set_state(maia_wifi_state_t state)
{
  if (state != g_state)
    {
      ESP_LOGI(TAG, "State: %d -> %d", g_state, state);
      g_state = state;
    }
}

/****************************************************************************
 * Name: maia_wifi_sta_netif
 * Name: maia_wifi_ap_netif
 *
 * Description:
 *   Internal — accessors for the shared netif handles.
 *
 ****************************************************************************/

esp_netif_t *maia_wifi_sta_netif(void)
{
  return g_netif_sta;
}

esp_netif_t *maia_wifi_ap_netif(void)
{
  return g_netif_ap;
}

/****************************************************************************
 * Name: maia_wifi_init
 ****************************************************************************/

esp_err_t maia_wifi_init(maia_wifi_callback_t cb)
{
  esp_err_t ret;

  if (g_initialized)
    {
      ESP_LOGW(TAG, "Already initialized");
      return ESP_OK;
    }

  g_cb = cb;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
      return ret;
    }

  ret = esp_netif_init();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
      return ret;
    }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s",
               esp_err_to_name(ret));
      return ret;
    }

  netif_ready();

  wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&init_cfg);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
      return ret;
    }

  ret = maia_wifi_sta_events_register();
  if (ret != ESP_OK)
    {
      return ret;
    }

  ret = maia_wifi_ap_events_register();
  if (ret != ESP_OK)
    {
      return ret;
    }

  g_initialized = true;
  g_state = MAIA_WIFI_STATE_IDLE;

  ESP_LOGI(TAG, "WiFi driver initialized");

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_deinit
 ****************************************************************************/

esp_err_t maia_wifi_deinit(void)
{
  if (!g_initialized)
    {
      return ESP_OK;
    }

  maia_wifi_sta_stop();
  maia_wifi_ap_stop();

  esp_wifi_deinit();

  g_initialized = false;
  g_state = MAIA_WIFI_STATE_IDLE;

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_get_state
 ****************************************************************************/

maia_wifi_state_t maia_wifi_get_state(void)
{
  return g_state;
}

/****************************************************************************
 * Name: maia_wifi_get_mac_suffix
 ****************************************************************************/

esp_err_t maia_wifi_get_mac_suffix(char *out, size_t len)
{
  uint8_t mac[6];
  esp_err_t ret;

  if (out == NULL || len < 5)
    {
      return ESP_ERR_INVALID_SIZE;
    }

  ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
  if (ret != ESP_OK)
    {
      /* Radio not started yet — the base MAC is still readable via the
       * efuse-backed default, which is what esp_wifi_get_mac() falls
       * back to before esp_wifi_start().
       */

      ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
      if (ret != ESP_OK)
        {
          return ret;
        }
    }

  snprintf(out, len, "%02X%02X", mac[4], mac[5]);

  return ESP_OK;
}

/****************************************************************************
 * Name: maia_wifi_is_provisioned
 ****************************************************************************/

bool maia_wifi_is_provisioned(void)
{
  wifi_config_t cfg;

  /* esp_wifi_get_config() requires the driver to be initialized (it
   * reads/writes through the driver's own state, backed by NVS), but not
   * started — this works whether or not esp_wifi_start() has run.
   */

  if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK)
    {
      return false;
    }

  return strlen((const char *)cfg.sta.ssid) > 0;
}
