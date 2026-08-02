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
 * components/drivers/wifi/include/maia_wifi.h
 *
 * MAIA WiFi Driver
 * Radio control only: station, diagnostic access point, scan.
 *
 * Reference: spec/wifi_subsystem.md
 *
 * ---------------------------------------------------------------------------
 * SCOPE — mechanism, not policy
 * ---------------------------------------------------------------------------
 * This driver owns the radio and nothing else, in the NuttX sense of a
 * driver: it exposes primitives (bring the station interface up, attempt
 * an association, raise a diagnostic AP, scan) and reports raw events
 * (associated, disconnected + hardware reason code, got an IP). It does
 * not decide what to do about any of that.
 *
 * Two things that might look like they belong here do not, on purpose:
 *
 *   Reconnect policy — how many times to retry a failed connection, how
 *   long to back off, and when to give up and stop the radio to save
 *   battery is a product decision, not a radio-control primitive. This
 *   driver hands the raw WIFI_EVENT_STA_DISCONNECTED reason code to the
 *   caller via MAIA_WIFI_EVENT_STA_DISCONNECTED and stops there. See
 *   main/tests/test_wifi.c (MAIA_TEST_WIFI_MODE_CONNECT) for a
 *   reference implementation of that policy built entirely on top of
 *   this driver's public API — today that reference implementation
 *   lives in test code because
 *   components/services/wifi_prov_service/ (or an app-level equivalent)
 *   does not exist yet. When it does, the policy moves there, not here.
 *
 *   Boot orchestration — "connect with stored credentials if there are
 *   any, else seed from Kconfig, else raise the diagnostic AP" is an
 *   application decision about what the device should do, not a driver
 *   operation. This driver exposes the primitives that decision is made
 *   from (maia_wifi_is_provisioned(), maia_wifi_sta_start(),
 *   maia_wifi_ap_start()) and nothing that makes the decision itself.
 *   See test_wifi_auto() in main/tests/test_wifi.c for the reference
 *   implementation, for the same reason as above.
 *
 * What is implemented here:
 *
 *   L1  Core ................ init/deinit, event loop, state machine
 *   L2  Station .............. bring-up, one connection attempt, teardown
 *   L3  Access Point ......... diagnostic AP (no provisioning UX)
 *   L4  Scan .................. visible network list
 *
 * L5 (WPS) and the provisioning-aware states/events lands with the
 * provisioning service, per spec section 15.
 *
 * ---------------------------------------------------------------------------
 * CREDENTIAL STORAGE
 * ---------------------------------------------------------------------------
 * There is no MAIA-owned credential store here, deliberately. esp_wifi
 * already persists the station config it is given
 * (CONFIG_ESP_WIFI_NVS_ENABLED), and maia_wifi_is_provisioned() reads
 * exactly that record — the same one esp-idf's wifi_provisioning manager
 * will read once the service layer exists. Adding a second, MAIA-owned
 * copy would create two sources of truth that can disagree. Reading
 * that record back is a query, not a decision, which is why it stays
 * here rather than moving out with the boot-orchestration logic above —
 * it does not decide what to do with the answer.
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_WIFI_INCLUDE_MAIA_WIFI_H
#define __COMPONENTS_DRIVERS_WIFI_INCLUDE_MAIA_WIFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_err.h>
#include <esp_netif_types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAIA_WIFI_SSID_MAX_LEN      32
#define MAIA_WIFI_PASSWORD_MAX_LEN  64

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  MAIA_WIFI_STATE_IDLE = 0,       /* Radio stopped, or STA up but not
                                    * associating/associated             */
  MAIA_WIFI_STATE_STA_CONNECTING, /* maia_wifi_sta_connect() called,
                                    * waiting for the outcome             */
  MAIA_WIFI_STATE_STA_CONNECTED,  /* Associated with an IP               */
  MAIA_WIFI_STATE_AP_RUNNING,     /* Diagnostic AP active                */
} maia_wifi_state_t;

typedef enum
{
  MAIA_WIFI_EVENT_STA_CONNECTED = 0, /* L2 associated (no IP yet)        */
  MAIA_WIFI_EVENT_STA_DISCONNECTED,  /* Lost/failed association. data
                                       * points to a stack-local uint8_t
                                       * holding the raw WIFI_REASON_*
                                       * code, valid only for the
                                       * duration of this callback —
                                       * copy it if you need it after
                                       * returning.                      */
  MAIA_WIFI_EVENT_STA_GOT_IP,        /* Usable — safe to sync now        */
  MAIA_WIFI_EVENT_AP_STARTED,        /* Diagnostic AP is up              */
  MAIA_WIFI_EVENT_AP_CLIENT_JOINED,
  MAIA_WIFI_EVENT_AP_CLIENT_LEFT,
  MAIA_WIFI_EVENT_SCAN_DONE,
} maia_wifi_event_t;

typedef struct
{
  char     ssid[MAIA_WIFI_SSID_MAX_LEN + 1];
  int8_t   rssi;
  uint8_t  channel;
  bool     needs_password;  /* false only for an open (unsecured) AP    */
} maia_wifi_ap_info_t;

typedef struct
{
  char ssid[MAIA_WIFI_SSID_MAX_LEN + 1];
  char password[MAIA_WIFI_PASSWORD_MAX_LEN + 1];
} maia_wifi_creds_t;

/* data points at a maia_wifi_ap_info_t for AP_CLIENT_* events, at a
 * uint8_t reason code for STA_DISCONNECTED, otherwise NULL.
 */

typedef void (*maia_wifi_callback_t)(maia_wifi_event_t event, void *data);

/****************************************************************************
 * Public Function Prototypes — L1 Core
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_init
 *
 * Description:
 *   Initialize the WiFi driver: netif, default event loop (if not already
 *   running), NVS (if not already initialized), and the WiFi driver
 *   itself. Must be called once before any other maia_wifi_* function.
 *
 * Input Parameters:
 *   cb - Callback invoked from the event loop task for every
 *        maia_wifi_event_t. May be NULL if the caller only polls
 *        maia_wifi_get_state().
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t maia_wifi_init(maia_wifi_callback_t cb);

/****************************************************************************
 * Name: maia_wifi_deinit
 *
 * Description:
 *   Stop the radio and release driver resources.
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t maia_wifi_deinit(void);

/****************************************************************************
 * Name: maia_wifi_get_state
 *
 * Description:
 *   Get the current driver state machine state.
 *
 * Returned Value:
 *   Current maia_wifi_state_t.
 *
 ****************************************************************************/

maia_wifi_state_t maia_wifi_get_state(void);

/****************************************************************************
 * Name: maia_wifi_get_mac_suffix
 *
 * Description:
 *   Format the last two bytes of the station base MAC as 4 uppercase hex
 *   characters (e.g. "A4F3"). Used to disambiguate SSIDs when more than
 *   one MAIA unit may be nearby.
 *
 * Input Parameters:
 *   out - Buffer to receive the NUL-terminated suffix.
 *   len - Size of out. Must be >= 5.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_SIZE if len < 5.
 *
 ****************************************************************************/

esp_err_t maia_wifi_get_mac_suffix(char *out, size_t len);

/****************************************************************************
 * Name: maia_wifi_is_provisioned
 *
 * Description:
 *   Report whether esp_wifi holds a non-empty station SSID in its
 *   persisted config (CONFIG_ESP_WIFI_NVS_ENABLED). This is the exact
 *   check esp-idf's wifi_provisioning manager performs internally, so it
 *   stays consistent with the future provisioning service without this
 *   driver owning a second copy of the credentials.
 *
 *   This is a read-only query — it does not decide what to do with the
 *   answer. See the SCOPE note above for where that decision lives.
 *
 * Returned Value:
 *   true if a station SSID is stored; false otherwise or on error.
 *
 ****************************************************************************/

bool maia_wifi_is_provisioned(void);

/****************************************************************************
 * Public Function Prototypes — L2 Station
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_sta_start
 *
 * Description:
 *   Bring up the station interface: set mode, apply credentials (or
 *   validate that some are already stored), apply the power-save
 *   setting, and start the driver. Does NOT attempt to associate — call
 *   maia_wifi_sta_connect() separately. Splitting bring-up from
 *   association is what lets a caller retry a connection attempt
 *   without repeating the whole setup.
 *
 * Input Parameters:
 *   creds - Credentials to connect with. Pass NULL to use whatever is
 *           already stored in esp_wifi's persisted config.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_STATE if creds is NULL and
 *   nothing is stored.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_start(const maia_wifi_creds_t *creds);

/****************************************************************************
 * Name: maia_wifi_sta_connect
 *
 * Description:
 *   Attempt one association using whatever config maia_wifi_sta_start()
 *   applied. Safe to call again after a MAIA_WIFI_EVENT_STA_DISCONNECTED
 *   to retry — this driver does not do that on its own (see the SCOPE
 *   note above).
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_STATE if maia_wifi_sta_start()
 *   has not been called.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_connect(void);

/****************************************************************************
 * Name: maia_wifi_sta_stop
 *
 * Description:
 *   Stop the station interface.
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_stop(void);

/****************************************************************************
 * Name: maia_wifi_sta_get_ip
 *
 * Description:
 *   Get the station's current IP configuration.
 *
 * Input Parameters:
 *   ip - Output. Only valid while state is MAIA_WIFI_STATE_STA_CONNECTED.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_STATE if not connected.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_get_ip(esp_netif_ip_info_t *ip);

/****************************************************************************
 * Name: maia_wifi_sta_get_rssi
 *
 * Description:
 *   Get the current station link RSSI.
 *
 * Input Parameters:
 *   rssi - Output, in dBm.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_STATE if not connected.
 *
 ****************************************************************************/

esp_err_t maia_wifi_sta_get_rssi(int8_t *rssi);

/****************************************************************************
 * Public Function Prototypes — L3 Access Point (diagnostic)
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_ap_start
 *
 * Description:
 *   Raise a plain WPA2 access point per the MAIA_WIFI_AP_* Kconfig
 *   symbols. This is a bench/diagnostic AP, not the setup portal — it
 *   serves nothing and accepts no credentials. Use it to validate the
 *   radio and antenna without a router nearby (spec section 14, SCAN).
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_INVALID_ARG if the configured password
 *   is non-empty and shorter than 8 characters (WPA2 minimum).
 *
 ****************************************************************************/

esp_err_t maia_wifi_ap_start(void);

/****************************************************************************
 * Name: maia_wifi_ap_stop
 *
 * Description:
 *   Stop the diagnostic access point.
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t maia_wifi_ap_stop(void);

/****************************************************************************
 * Public Function Prototypes — L4 Scan
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_scan
 *
 * Description:
 *   Perform a blocking active scan and return the visible networks
 *   sorted by descending RSSI. Requires the station interface to be
 *   started (maia_wifi_sta_start()) — WiFi scanning needs an active STA
 *   netif even if not associated.
 *
 * Input Parameters:
 *   results - Output array.
 *   max     - Capacity of results.
 *   found   - Output: number of entries written to results.
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t maia_wifi_scan(maia_wifi_ap_info_t *results, size_t max,
                         size_t *found);

#endif /* __COMPONENTS_DRIVERS_WIFI_INCLUDE_MAIA_WIFI_H */
