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
 * main/tests/test_wifi.c
 *
 * MAIA - WiFi radio driver test
 *
 * Exercises components/drivers/wifi (L1-L4: core, station, diagnostic
 * AP, scan) via the MAIA_TEST_WIFI_MODE Kconfig choice.
 *
 * The driver is mechanism only (see the SCOPE note in maia_wifi.h): it
 * never retries a failed connection and never decides what to do when
 * unprovisioned. MAIA_TEST_WIFI_MODE_CONNECT is where that policy is
 * exercised — built entirely on top of the driver's public primitives,
 * standing in for the app/service layer that will eventually own it.
 * Nothing in this file reaches into driver internals.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tests.h"
#include "maia_wifi.h"
#include "wifi_prov_service.h"
#include "maia_board.h"
#include <string.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[TEST_WIFI]"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: event_cb
 *
 * Description:
 *   Common event callback for all sub-tests — just logs what happened.
 *
 ****************************************************************************/

static void event_cb(maia_wifi_event_t event, void *data)
{
  switch (event)
    {
      case MAIA_WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, ">>> STA associated (no IP yet)");
        break;

      case MAIA_WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, ">>> STA disconnected, reason=%d",
                 data != NULL ? *(uint8_t *)data : -1);
        break;

      case MAIA_WIFI_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, ">>> STA got IP");
        break;

      case MAIA_WIFI_EVENT_AP_STARTED:
        ESP_LOGI(TAG, ">>> AP started");
        break;

      case MAIA_WIFI_EVENT_AP_CLIENT_JOINED:
        ESP_LOGI(TAG, ">>> AP client joined");
        break;

      case MAIA_WIFI_EVENT_AP_CLIENT_LEFT:
        ESP_LOGI(TAG, ">>> AP client left");
        break;

      case MAIA_WIFI_EVENT_SCAN_DONE:
        ESP_LOGI(TAG, ">>> Scan done");
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: state_name
 *
 ****************************************************************************/

static const char *state_name(maia_wifi_state_t state)
{
  switch (state)
    {
      case MAIA_WIFI_STATE_IDLE:           return "IDLE";
      case MAIA_WIFI_STATE_STA_CONNECTING: return "STA_CONNECTING";
      case MAIA_WIFI_STATE_STA_CONNECTED:  return "STA_CONNECTED";
      case MAIA_WIFI_STATE_AP_RUNNING:     return "AP_RUNNING";
      default:                             return "UNKNOWN";
    }
}

#if defined(CONFIG_MAIA_TEST_WIFI_MODE_SUITE)

/****************************************************************************
 * Name: test_wifi_suite
 *
 * Description:
 *   Basic test suite: init/deinit, MAC suffix, is_provisioned(), and
 *   state transitions with no radio activity.
 *
 ****************************************************************************/

static void test_wifi_suite(void)
{
  char suffix[5];
  esp_err_t ret;

  ESP_LOGI(TAG, "=== TEST: Basic Suite ===");

  ESP_LOGI(TAG, "State after init: %s", state_name(maia_wifi_get_state()));

  ret = maia_wifi_get_mac_suffix(suffix, sizeof(suffix));
  if (ret == ESP_OK)
    {
      ESP_LOGI(TAG, "MAC suffix: %s", suffix);
    }
  else
    {
      ESP_LOGE(TAG, "get_mac_suffix failed: %s", esp_err_to_name(ret));
    }

  bool provisioned = maia_wifi_is_provisioned();

  ESP_LOGI(TAG, "is_provisioned(): %s", provisioned ? "true" : "false");
  ESP_LOGI(TAG, "  (stored station SSID, or the Kconfig seed if this is "
                "the first boot with a non-empty MAIA_WIFI_STA_SSID)");

  ESP_LOGI(TAG, "Basic suite complete");
}

#elif defined(CONFIG_MAIA_TEST_WIFI_MODE_SCAN)

/****************************************************************************
 * Name: test_wifi_scan
 *
 * Description:
 *   Scan and print visible networks. Fastest antenna sanity check —
 *   see the WROOM-1U note in the Kconfig help.
 *
 ****************************************************************************/

#define SCAN_MAX_RESULTS  20

static void test_wifi_scan(void)
{
  static maia_wifi_ap_info_t results[SCAN_MAX_RESULTS];
  size_t found = 0;
  esp_err_t ret;

  ESP_LOGI(TAG, "=== TEST: Network Scan ===");

  /* Scanning needs an active STA netif even without an association, so
   * bring the station interface up without connecting to anything.
   */

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "set_mode failed: %s", esp_err_to_name(ret));
      return;
    }

  ret = esp_wifi_start();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
      return;
    }

  ret = maia_wifi_scan(results, SCAN_MAX_RESULTS, &found);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
      return;
    }

  if (found == 0)
    {
      ESP_LOGW(TAG, "No networks found — check the antenna is fitted "
                    "(WROOM-1U needs an external U.FL antenna)");
      return;
    }

  ESP_LOGI(TAG, "Found %d network(s):", found);

  for (size_t i = 0; i < found; i++)
    {
      ESP_LOGI(TAG, "  %2d. %-32s RSSI=%4ddBm ch=%2d %s", i,
               results[i].ssid, results[i].rssi, results[i].channel,
               results[i].needs_password ? "[secured]" : "[open]");
    }
}

#elif defined(CONFIG_MAIA_TEST_WIFI_MODE_CONNECT)

/****************************************************************************
 * Reconnect policy — test/app-level, not driver code
 *
 * The maia_wifi driver does not retry a failed connection on its own;
 * it only reports MAIA_WIFI_EVENT_STA_DISCONNECTED with the raw
 * hardware reason code (see the SCOPE note in maia_wifi.h). Deciding
 * how many times to retry, how long to back off, and when to give up
 * and stop the radio to save battery is a product decision, so it is
 * implemented here, in the code that CALLS the driver — standing in
 * for the app/service layer that will eventually own it
 * (spec/wifi_subsystem.md section 6.1).
 ****************************************************************************/

typedef enum
{
  RECONNECT_REASON_AUTH = 0,   /* Wrong password / security handshake  */
  RECONNECT_REASON_NO_AP,      /* Out of range                         */
  RECONNECT_REASON_TRANSIENT,  /* Everything else                      */
} reconnect_reason_t;

static reconnect_reason_t classify_reason(uint8_t reason)
{
  switch (reason)
    {
      case WIFI_REASON_AUTH_FAIL:
      case WIFI_REASON_HANDSHAKE_TIMEOUT:
      case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      case WIFI_REASON_MIC_FAILURE:
      case WIFI_REASON_NOT_AUTHED:
        return RECONNECT_REASON_AUTH;

      case WIFI_REASON_NO_AP_FOUND:
      case WIFI_REASON_BEACON_TIMEOUT:
      case WIFI_REASON_ASSOC_LEAVE:
      case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
      case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
      case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return RECONNECT_REASON_NO_AP;

      default:
        return RECONNECT_REASON_TRANSIENT;
    }
}

#define RETRY_BACKOFF_CAP_MS  30000

static uint32_t retry_backoff_ms(uint32_t attempt)
{
  uint64_t backoff = (uint64_t)CONFIG_MAIA_WIFI_STA_RETRY_BACKOFF_MS << attempt;

  if (backoff > RETRY_BACKOFF_CAP_MS || backoff == 0)
    {
      return RETRY_BACKOFF_CAP_MS;
    }

  return (uint32_t)backoff;
}

typedef struct
{
  bool    got_ip;
  bool    disconnected;
  uint8_t reason;
} sta_msg_t;

static QueueHandle_t s_sta_evt_q = NULL;

/****************************************************************************
 * Name: connect_event_cb
 *
 * Description:
 *   Registered with maia_wifi_init() for the CONNECT sub-test,
 *   regardless of which MAIA_WIFI_MODE sub-branch ends up running.
 *   Logs like event_cb(), and additionally forwards STA events to a
 *   queue so connect_with_retry_policy() (running on the test task,
 *   not the event-loop task) can react to them.
 *
 *   s_sta_evt_q only exists while connect_with_retry_policy() is on
 *   the stack (Auto/Station branches) — the Provisioning branch
 *   connects via wifi_prov_service_run() instead and never creates
 *   it, yet this same callback still fires on its GOT_IP/DISCONNECTED
 *   events (one callback is registered per maia_wifi_init() call, for
 *   every branch). The NULL check below is what makes that safe:
 *   without it, a successful connection through the Provisioning
 *   branch reaches xQueueSend(NULL, ...), which asserts and reboots
 *   the board right at the moment it got an IP.
 *
 ****************************************************************************/

static void connect_event_cb(maia_wifi_event_t event, void *data)
{
  sta_msg_t msg = { 0 };

  event_cb(event, data);

  if (s_sta_evt_q == NULL)
    {
      return;
    }

  switch (event)
    {
      case MAIA_WIFI_EVENT_STA_GOT_IP:
        msg.got_ip = true;
        xQueueSend(s_sta_evt_q, &msg, 0);
        break;

      case MAIA_WIFI_EVENT_STA_DISCONNECTED:
        msg.disconnected = true;
        msg.reason = data != NULL ? *(uint8_t *)data : 0;
        xQueueSend(s_sta_evt_q, &msg, 0);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: connect_with_retry_policy
 *
 * Description:
 *   Calls maia_wifi_sta_connect() and reacts to the driver's raw
 *   events: out-of-range disconnects back off immediately without
 *   counting against the retry budget (spec section 6.1 — a dog
 *   walking out of range is normal, not a fault); auth and transient
 *   failures retry with exponential backoff up to
 *   MAIA_WIFI_STA_MAX_RETRY, then give up.
 *
 * Returned Value:
 *   true if a MAIA_WIFI_EVENT_STA_GOT_IP was observed before the
 *   overall timeout; false otherwise (out of range, retries exhausted,
 *   or timed out).
 *
 ****************************************************************************/

static bool connect_with_retry_policy(uint32_t overall_timeout_ms)
{
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(overall_timeout_ms);
  uint32_t retry_count = 0;
  bool connected = false;

  s_sta_evt_q = xQueueCreate(4, sizeof(sta_msg_t));
  if (s_sta_evt_q == NULL)
    {
      ESP_LOGE(TAG, "Failed to create event queue");
      return false;
    }

  maia_wifi_sta_connect();

  while (xTaskGetTickCount() < deadline)
    {
      sta_msg_t msg;

      if (xQueueReceive(s_sta_evt_q, &msg, pdMS_TO_TICKS(500)) != pdTRUE)
        {
          continue;
        }

      if (msg.got_ip)
        {
          connected = true;
          break;
        }

      if (msg.disconnected)
        {
          reconnect_reason_t cls = classify_reason(msg.reason);

          if (cls == RECONNECT_REASON_NO_AP)
            {
              ESP_LOGW(TAG, "Out of range — not retrying, this is not "
                            "a credential problem");
              break;
            }

          retry_count++;
          if (retry_count >= CONFIG_MAIA_WIFI_STA_MAX_RETRY)
            {
              if (cls == RECONNECT_REASON_AUTH)
                {
                  ESP_LOGE(TAG, "Auth failed %" PRIu32 " times, giving "
                                "up — credentials are likely wrong",
                           retry_count);
                }
              else
                {
                  ESP_LOGW(TAG, "Transient failures exhausted retries, "
                                "giving up without blaming credentials");
                }
              break;
            }

          uint32_t delay_ms = retry_backoff_ms(retry_count - 1);

          ESP_LOGI(TAG, "Retry %" PRIu32 "/%d in %" PRIu32 "ms",
                   retry_count, CONFIG_MAIA_WIFI_STA_MAX_RETRY, delay_ms);
          vTaskDelay(pdMS_TO_TICKS(delay_ms));
          maia_wifi_sta_connect();
        }
    }

  vQueueDelete(s_sta_evt_q);
  s_sta_evt_q = NULL;

  return connected;
}

/****************************************************************************
 * Name: led_feedback_task
 *
 * Description:
 *   Drives the status LED from maia_wifi_get_state(), per the pattern
 *   in spec/wifi_subsystem.md section 12.2:
 *     AP_RUNNING      - blink at MAIA_LED_BLINK_FREQ_AP_HZ
 *     STA_CONNECTING  - blink at MAIA_LED_BLINK_FREQ_STA_HZ
 *     STA_CONNECTED   - solid on
 *     IDLE            - off
 *
 *   This is UI glue, not driver code — same reasoning as the reconnect
 *   policy above: the driver only reports state, it does not decide
 *   what the board's indicators do about it. Lives here for the same
 *   reason connect_with_retry_policy() does: there is no app/service
 *   layer yet to own it (spec section 15, phase 5).
 *
 *   Polls rather than subscribing to events — simpler, and a 50ms
 *   polling granularity is unobservable against blink periods of
 *   100ms+ (i.e. <= 5Hz, the fastest rate MAIA_LED_BLINK_FREQ_AP_HZ's
 *   range allows).
 *
 ****************************************************************************/

#if CONFIG_MAIA_TEST_WIFI_LED_FEEDBACK

#define LED_POLL_MS  50

static void led_feedback_task(void *arg)
{
  (void)arg;

  bool led_on = false;
  TickType_t next_toggle = 0;

  while (1)
    {
      maia_wifi_state_t state = maia_wifi_get_state();
      uint32_t half_period_ms;

      switch (state)
        {
          case MAIA_WIFI_STATE_AP_RUNNING:
            half_period_ms = 500 / CONFIG_MAIA_LED_BLINK_FREQ_AP_HZ;
            break;

          case MAIA_WIFI_STATE_STA_CONNECTING:
            half_period_ms = 500 / CONFIG_MAIA_LED_BLINK_FREQ_STA_HZ;
            break;

          case MAIA_WIFI_STATE_STA_CONNECTED:
            maia_led_set(true);
            vTaskDelay(pdMS_TO_TICKS(LED_POLL_MS));
            continue;

          case MAIA_WIFI_STATE_IDLE:
          default:
            maia_led_set(false);
            vTaskDelay(pdMS_TO_TICKS(LED_POLL_MS));
            continue;
        }

      TickType_t now = xTaskGetTickCount();

      if (now >= next_toggle)
        {
          led_on = !led_on;
          maia_led_set(led_on);
          next_toggle = now + pdMS_TO_TICKS(half_period_ms);
        }

      vTaskDelay(pdMS_TO_TICKS(LED_POLL_MS));
    }
}

#endif /* CONFIG_MAIA_TEST_WIFI_LED_FEEDBACK */

/****************************************************************************
 * Name: test_wifi_connect
 *
 * Description:
 *   Runs whatever MAIA_WIFI_MODE selects:
 *     Auto          - boot decision: stored credentials -> connect,
 *                      else Kconfig seed -> connect, else raise the
 *                      diagnostic AP.
 *     Station       - Kconfig credentials only, never falls back to
 *                      the AP.
 *     AP            - diagnostic AP only, no connection attempt.
 *     Provisioning  - delegates entirely to wifi_prov_service_run()
 *                      (components/services/wifi_prov_service/): if
 *                      already provisioned, connects directly;
 *                      otherwise raises a SoftAP and runs the
 *                      ESP-IDF wifi_provisioning manager so a phone
 *                      can supply credentials.
 *
 *   Runs led_feedback_task() alongside so the status LED reflects
 *   whichever of the above is active — see led_feedback_task() above
 *   for the exact pattern and why it lives here, not in the driver.
 *
 ****************************************************************************/

#define CONNECT_TEST_TIMEOUT_MS  60000

static void test_wifi_connect(void)
{
  maia_wifi_creds_t creds = { 0 };
  const maia_wifi_creds_t *start_creds = NULL;
  esp_err_t ret;

#if CONFIG_MAIA_TEST_WIFI_LED_FEEDBACK
  TaskHandle_t led_task = NULL;

  xTaskCreate(led_feedback_task, "wifi_led", 2048, NULL, 5, &led_task);
#endif

  ESP_LOGI(TAG, "=== TEST: Exercise Connectivity ===");

#if defined(CONFIG_MAIA_WIFI_MODE_AP)

  ESP_LOGI(TAG, "MAIA_WIFI_MODE_AP: raising diagnostic AP only");

  ret = maia_wifi_ap_start();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "ap_start failed: %s", esp_err_to_name(ret));
      return;
    }

  ESP_LOGI(TAG, "AP running. Join it from a phone to see join/leave "
                "events logged. This AP serves nothing — it is a radio "
                "sanity check only. Running until reset.");

  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }

#elif defined(CONFIG_MAIA_WIFI_MODE_PROVISIONING)

  ESP_LOGI(TAG, "MAIA_WIFI_MODE_PROVISIONING: delegating to "
                "wifi_prov_service_run()");

  ret = wifi_prov_service_run();
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Provisioning/connect failed: %s", esp_err_to_name(ret));
      return;
    }

  /* wifi_prov_service_run() already connected the station — whether
   * via the manager's own internal retry loop (interactive path) or
   * its own bounded connect loop (already-provisioned path). No need
   * to call maia_wifi_sta_start()/_connect() again; the driver's
   * state has been tracking reality passively the whole time (see
   * the SCOPE note in maia_wifi.h).
   */

  {
    esp_netif_ip_info_t ip;
    int8_t rssi;

    if (maia_wifi_sta_get_ip(&ip) == ESP_OK)
      {
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ip.ip));
      }

    if (maia_wifi_sta_get_rssi(&rssi) == ESP_OK)
      {
        ESP_LOGI(TAG, "RSSI: %d dBm", rssi);
      }
  }

  ESP_LOGI(TAG, "Final state: %s", state_name(maia_wifi_get_state()));
  ESP_LOGI(TAG, "Connectivity test complete");

#else /* MAIA_WIFI_MODE_AUTO or MAIA_WIFI_MODE_STA */

#if defined(CONFIG_MAIA_WIFI_MODE_AUTO)
  ESP_LOGI(TAG, "MAIA_WIFI_MODE_AUTO: is_provisioned()=%s",
           maia_wifi_is_provisioned() ? "true" : "false");

  if (!maia_wifi_is_provisioned())
    {
      if (strlen(CONFIG_MAIA_WIFI_STA_SSID) == 0)
        {
          ESP_LOGI(TAG, "No stored credentials and no seed configured, "
                        "raising diagnostic AP");

          ret = maia_wifi_ap_start();
          if (ret != ESP_OK)
            {
              ESP_LOGE(TAG, "ap_start failed: %s", esp_err_to_name(ret));
              return;
            }

          ESP_LOGI(TAG, "AP running. Running until reset.");
          while (1)
            {
              vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }

      ESP_LOGI(TAG, "No stored credentials, seeding from Kconfig");
      strlcpy(creds.ssid, CONFIG_MAIA_WIFI_STA_SSID, sizeof(creds.ssid));
      strlcpy(creds.password, CONFIG_MAIA_WIFI_STA_PASSWORD,
              sizeof(creds.password));
      start_creds = &creds;
    }
  else
    {
      ESP_LOGI(TAG, "Stored credentials found, connecting");
      start_creds = NULL;
    }
#else /* MAIA_WIFI_MODE_STA */
  ESP_LOGI(TAG, "MAIA_WIFI_MODE_STA: using Kconfig credentials only");

  strlcpy(creds.ssid, CONFIG_MAIA_WIFI_STA_SSID, sizeof(creds.ssid));
  strlcpy(creds.password, CONFIG_MAIA_WIFI_STA_PASSWORD,
          sizeof(creds.password));

  if (strlen(creds.ssid) == 0)
    {
      ESP_LOGE(TAG, "MAIA_WIFI_STA_SSID is empty — set it in menuconfig "
                    "under WiFi Configuration > Station");
      return;
    }

  start_creds = &creds;
#endif

  ret = maia_wifi_sta_start(start_creds);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "sta_start failed: %s", esp_err_to_name(ret));
      return;
    }

  if (connect_with_retry_policy(CONNECT_TEST_TIMEOUT_MS))
    {
      esp_netif_ip_info_t ip;
      int8_t rssi;

      if (maia_wifi_sta_get_ip(&ip) == ESP_OK)
        {
          ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ip.ip));
        }

      if (maia_wifi_sta_get_rssi(&rssi) == ESP_OK)
        {
          ESP_LOGI(TAG, "RSSI: %d dBm", rssi);
        }
    }

  ESP_LOGI(TAG, "Final state: %s", state_name(maia_wifi_get_state()));
  maia_wifi_sta_stop();
  ESP_LOGI(TAG, "Connectivity test complete");

#endif /* MAIA_WIFI_MODE_AP / MAIA_WIFI_MODE_PROVISIONING */
}

#endif /* CONFIG_MAIA_TEST_WIFI_MODE_CONNECT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_wifi_run
 *
 * Description:
 *   Entry point called from main.c when MAIA_TEST_WIFI is selected.
 *   Dispatches to the sub-test picked by MAIA_TEST_WIFI_MODE.
 *
 ****************************************************************************/

void test_wifi_run(void)
{
  esp_err_t ret;

  ESP_LOGI(TAG, "Starting WiFi driver test");

#if defined(CONFIG_MAIA_TEST_WIFI_MODE_CONNECT)
  ret = maia_wifi_init(&connect_event_cb);
#else
  ret = maia_wifi_init(&event_cb);
#endif

  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "maia_wifi_init failed: %s", esp_err_to_name(ret));
      return;
    }

#if defined(CONFIG_MAIA_TEST_WIFI_MODE_SUITE)
  test_wifi_suite();
#elif defined(CONFIG_MAIA_TEST_WIFI_MODE_SCAN)
  test_wifi_scan();
#elif defined(CONFIG_MAIA_TEST_WIFI_MODE_CONNECT)
  test_wifi_connect();
#endif

  ESP_LOGI(TAG, "WiFi driver test completed");
}
