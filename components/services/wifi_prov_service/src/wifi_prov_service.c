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
 * components/services/wifi_prov_service/src/wifi_prov_service.c
 *
 * See wifi_prov_service.h for scope and the no-conflict argument
 * against components/drivers/wifi/.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "wifi_prov_service.h"
#include "maia_wifi.h"

#include <string.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

/* components/services/ is listed in EXTRA_COMPONENT_DIRS (root
 * CMakeLists.txt) so that this component is discovered at all — but
 * ESP-IDF builds every component under EXTRA_COMPONENT_DIRS
 * unconditionally, regardless of whether anything actually REQUIRES
 * it (unlike components discovered under the default components/
 * dir, which get pruned when unreachable). main/CMakeLists.txt only
 * adds this component to REQUIRES when CONFIG_MAIA_WIFI_ENABLE is
 * set, but that alone does not stop this translation unit from being
 * compiled — and every Kconfig symbol referenced below
 * (MAIA_WIFI_STA_MAX_RETRY, MAIA_WIFI_PROV_*, and even
 * WIFI_PROV_SECURITY_0/1 via the protocomm security-version selects)
 * lives inside "depends on MAIA_WIFI_ENABLE" menus and simply does
 * not exist in sdkconfig.h when it is off. Guarding the whole body
 * below — not just the individual symbol uses — is what makes this
 * file compile to an empty, harmless translation unit in that case.
 */

#if CONFIG_MAIA_WIFI_ENABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[WIFI_PROV]"

#define PROV_BIT_SUCCESS  BIT0
#define PROV_BIT_FAIL     BIT1
#define PROV_BIT_END      BIT2

#define STA_CONNECT_ATTEMPT_TIMEOUT_MS  15000

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  EventGroupHandle_t          eg;
  wifi_prov_sta_fail_reason_t fail_reason;
} prov_ctx_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: build_service_name
 *
 * Description:
 *   "PROV_MAIA_A4F3" from MAIA_WIFI_PROV_SSID_PREFIX plus the MAC
 *   suffix, mirroring maia_wifi_ap_start()'s diagnostic-AP naming.
 *
 ****************************************************************************/

static void build_service_name(char *out, size_t len)
{
  strlcpy(out, CONFIG_MAIA_WIFI_PROV_SSID_PREFIX, len);

#if CONFIG_MAIA_WIFI_PROV_SSID_APPEND_MAC
  char suffix[5];

  if (maia_wifi_get_mac_suffix(suffix, sizeof(suffix)) == ESP_OK)
    {
      size_t base_len = strlen(out);
      size_t room = len - base_len;

      snprintf(out + base_len, room, "_%s", suffix);
    }
#endif
}

/****************************************************************************
 * Name: build_ap_password
 *
 * Description:
 *   Resolves the provisioning AP's WPA2 password per
 *   MAIA_WIFI_PROV_AP_SECURITY: MAC-derived, static, or NULL (open).
 *   Returns NULL for open — that is what
 *   wifi_prov_mgr_start_provisioning() expects for service_key.
 *
 ****************************************************************************/

#if CONFIG_MAIA_WIFI_PROV_AP_WPA2_MAC
static const char *build_ap_password(char *out, size_t len)
{
  uint8_t mac[6];

  if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK &&
      esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK)
    {
      /* Should not happen once esp_wifi_init() has succeeded — this
       * is a last-resort, still-valid-WPA2-length fallback so a MAC
       * read failure never leaves the AP with an unset password.
       */

      strlcpy(out, "maiamute2026", len);
      return out;
    }

  snprintf(out, len, "MAIA%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
  return out;
}
#endif

/****************************************************************************
 * Name: build_pop
 *
 * Description:
 *   MAIA_WIFI_PROV_POP_STATIC if set, else MAIA_WIFI_PROV_POP_DIGITS
 *   random digits from the hardware RNG.
 *
 ****************************************************************************/

#if CONFIG_MAIA_WIFI_PROV_SEC1
static void build_pop(char *out, size_t len)
{
  if (strlen(CONFIG_MAIA_WIFI_PROV_POP_STATIC) > 0)
    {
      strlcpy(out, CONFIG_MAIA_WIFI_PROV_POP_STATIC, len);
      return;
    }

  uint32_t digits = CONFIG_MAIA_WIFI_PROV_POP_DIGITS;
  uint32_t modulus = 1;

  for (uint32_t i = 0; i < digits; i++)
    {
      modulus *= 10;
    }

  snprintf(out, len, "%0*" PRIu32, (int)digits, esp_random() % modulus);
}
#endif

/****************************************************************************
 * Name: prov_event_cb
 *
 * Description:
 *   wifi_prov_mgr_config_t.app_event_handler — the manager's own,
 *   dedicated callback slot. Independent of maia_wifi_callback_t, so
 *   there is no conflict with whatever the driver's single caller-
 *   supplied callback is already doing (see wifi_prov_service.h).
 *
 ****************************************************************************/

static void prov_event_cb(void *user_data, wifi_prov_cb_event_t event,
                          void *event_data)
{
  prov_ctx_t *ctx = user_data;

  switch (event)
    {
      case WIFI_PROV_START:
        ESP_LOGI(TAG, "Provisioning started");
        break;

      case WIFI_PROV_CRED_RECV:
        {
          wifi_sta_config_t *cfg = event_data;

          ESP_LOGI(TAG, "Received credentials for SSID \"%s\"",
                   (const char *)cfg->ssid);
        }
        break;

      case WIFI_PROV_CRED_FAIL:
        {
          wifi_prov_sta_fail_reason_t *reason = event_data;

          ctx->fail_reason = *reason;
          ESP_LOGE(TAG, "Provisioning failed: %s",
                   *reason == WIFI_PROV_STA_AUTH_ERROR ?
                   "wrong WiFi password" : "WiFi network not found");
          xEventGroupSetBits(ctx->eg, PROV_BIT_FAIL);
        }
        break;

      case WIFI_PROV_CRED_SUCCESS:
        ESP_LOGI(TAG, "Provisioning succeeded, station connected");
        xEventGroupSetBits(ctx->eg, PROV_BIT_SUCCESS);
        break;

      case WIFI_PROV_END:
        xEventGroupSetBits(ctx->eg, PROV_BIT_END);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: connect_stored_credentials
 *
 * Description:
 *   Already provisioned: connect directly using the driver's L2
 *   primitives, bounded by MAIA_WIFI_STA_MAX_RETRY attempts. No AP,
 *   no manager — deliberately simpler than the reconnect-policy demo
 *   in main/tests/test_wifi.c (out-of-range vs auth-failure
 *   classification, exponential backoff): this is the service's own,
 *   plainer "just try N times" policy for the boring case where
 *   credentials already work.
 *
 ****************************************************************************/

static esp_err_t connect_stored_credentials(void)
{
  esp_err_t ret = maia_wifi_sta_start(NULL);

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "sta_start failed: %s", esp_err_to_name(ret));
      return ret;
    }

  for (uint32_t attempt = 0; attempt < CONFIG_MAIA_WIFI_STA_MAX_RETRY;
       attempt++)
    {
      maia_wifi_sta_connect();

      TickType_t deadline = xTaskGetTickCount() +
                            pdMS_TO_TICKS(STA_CONNECT_ATTEMPT_TIMEOUT_MS);

      while (xTaskGetTickCount() < deadline)
        {
          if (maia_wifi_get_state() == MAIA_WIFI_STATE_STA_CONNECTED)
            {
              return ESP_OK;
            }

          vTaskDelay(pdMS_TO_TICKS(200));
        }

      ESP_LOGW(TAG, "Connect attempt %" PRIu32 "/%d timed out", attempt + 1,
               CONFIG_MAIA_WIFI_STA_MAX_RETRY);
    }

  maia_wifi_sta_stop();

  return ESP_FAIL;
}

/****************************************************************************
 * Name: run_interactive_provisioning
 *
 * Description:
 *   Not yet provisioned: raise the SoftAP and run the manager until
 *   success, failure, or MAIA_WIFI_PROV_TIMEOUT_MIN elapses.
 *
 ****************************************************************************/

static esp_err_t run_interactive_provisioning(void)
{
  prov_ctx_t ctx = { 0 };

  ctx.eg = xEventGroupCreate();
  if (ctx.eg == NULL)
    {
      ESP_LOGE(TAG, "Failed to create event group");
      return ESP_ERR_NO_MEM;
    }

  wifi_prov_mgr_config_t cfg = {
      .scheme = wifi_prov_scheme_softap,
      .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
      .app_event_handler = {
          .event_cb = prov_event_cb,
          .user_data = &ctx,
      },
      .wifi_prov_conn_cfg = {
          .wifi_conn_attempts = CONFIG_MAIA_WIFI_STA_MAX_RETRY,
      },
  };

  esp_err_t ret = wifi_prov_mgr_init(cfg);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "wifi_prov_mgr_init failed: %s", esp_err_to_name(ret));
      vEventGroupDelete(ctx.eg);
      return ret;
    }

  char ssid[MAIA_WIFI_SSID_MAX_LEN + 1];

  build_service_name(ssid, sizeof(ssid));

  const char *service_key = NULL;

#if CONFIG_MAIA_WIFI_PROV_AP_WPA2_MAC
  char derived_password[13];

  service_key = build_ap_password(derived_password, sizeof(derived_password));
#elif CONFIG_MAIA_WIFI_PROV_AP_WPA2_STATIC
  service_key = CONFIG_MAIA_WIFI_PROV_AP_PASSWORD;
#endif
  /* CONFIG_MAIA_WIFI_PROV_AP_OPEN: service_key stays NULL (open AP) */

  ESP_LOGW(TAG, "==========================================================");
  ESP_LOGW(TAG, " WiFi setup: join AP \"%s\"", ssid);
  if (service_key != NULL)
    {
      ESP_LOGW(TAG, "             password  \"%s\"", service_key);
    }
  else
    {
      ESP_LOGW(TAG, "             (open network)");
    }

#if CONFIG_MAIA_WIFI_PROV_SEC1
  char pop[CONFIG_MAIA_WIFI_PROV_POP_DIGITS + 1];

  build_pop(pop, sizeof(pop));
  ESP_LOGW(TAG, "             then enter code \"%s\" in the app", pop);
#endif
  ESP_LOGW(TAG, "             app: ESP SoftAP Prov (Espressif)");
  ESP_LOGW(TAG, "==========================================================");

#if CONFIG_MAIA_WIFI_PROV_SEC1
  ret = wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,
                                         (const void *)pop, ssid,
                                         service_key);
#else
  ret = wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL, ssid,
                                         service_key);
#endif

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "start_provisioning failed: %s", esp_err_to_name(ret));
      wifi_prov_mgr_deinit();
      vEventGroupDelete(ctx.eg);
      return ret;
    }

  TickType_t timeout_ticks = CONFIG_MAIA_WIFI_PROV_TIMEOUT_MIN > 0 ?
      pdMS_TO_TICKS((uint32_t)CONFIG_MAIA_WIFI_PROV_TIMEOUT_MIN * 60000) :
      portMAX_DELAY;

  EventBits_t bits = xEventGroupWaitBits(ctx.eg,
                                         PROV_BIT_SUCCESS | PROV_BIT_FAIL,
                                         pdFALSE, pdFALSE, timeout_ticks);

  esp_err_t result;

  if (bits & PROV_BIT_SUCCESS)
    {
      /* WIFI_PROV_CRED_SUCCESS auto-stops the service (default
       * behavior — wifi_prov_mgr_disable_auto_stop() was not called),
       * which emits WIFI_PROV_END on its own. Do not call
       * wifi_prov_mgr_stop_provisioning() here — it already is.
       */

      result = ESP_OK;
    }
  else if (bits & PROV_BIT_FAIL)
    {
      ESP_LOGE(TAG, "Wrong credentials submitted, aborting provisioning "
                    "(bounded by MAIA_WIFI_STA_MAX_RETRY attempts)");
      wifi_prov_mgr_stop_provisioning();
      result = ESP_FAIL;
    }
  else
    {
      ESP_LOGW(TAG, "Provisioning timed out after %d minute(s)",
               CONFIG_MAIA_WIFI_PROV_TIMEOUT_MIN);
      wifi_prov_mgr_stop_provisioning();
      result = ESP_ERR_TIMEOUT;
    }

  xEventGroupWaitBits(ctx.eg, PROV_BIT_END, pdFALSE, pdFALSE,
                      pdMS_TO_TICKS(5000));
  wifi_prov_mgr_deinit();
  vEventGroupDelete(ctx.eg);

  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wifi_prov_service_run
 ****************************************************************************/

esp_err_t wifi_prov_service_run(void)
{
  /* Idempotent — if the caller already called maia_wifi_init(), this
   * is a no-op (see maia_wifi_init()'s g_initialized guard). Makes
   * this service usable standalone, without depending on the caller
   * having brought the driver up first.
   */

  esp_err_t ret = maia_wifi_init(NULL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_wifi_init failed: %s", esp_err_to_name(ret));
      return ret;
    }

  if (maia_wifi_is_provisioned())
    {
      ESP_LOGI(TAG, "Already provisioned, connecting directly");
      return connect_stored_credentials();
    }

  ESP_LOGI(TAG, "Not provisioned, starting interactive provisioning");
  return run_interactive_provisioning();
}

#endif /* CONFIG_MAIA_WIFI_ENABLE */
