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
 * components/services/wifi_prov_service/include/wifi_prov_service.h
 *
 * MAIA WiFi Provisioning Service
 *
 * Owns the policy of getting the device from "no credentials" to
 * "connected", using the ESP-IDF wifi_provisioning manager (protocomm
 * over SoftAP) as the mechanism. This is the service layer the design
 * in spec/wifi_subsystem.md always intended to sit above
 * components/drivers/wifi/ — that driver is mechanism only (see the
 * SCOPE note in maia_wifi.h) and knows nothing about provisioning.
 *
 * ---------------------------------------------------------------------------
 * SCOPE — what this delivers today
 * ---------------------------------------------------------------------------
 * Implemented: the manager lifecycle (init, start, security/PoP, event
 * handling, stop, deinit), a plain connect path when already
 * provisioned, and serial-log feedback (PoP code, AP name/password,
 * outcome). Client: the stock "ESP SoftAP Prov" app (Espressif, free,
 * Android/iOS), which speaks the manager's standard protocomm
 * endpoints. No board UI is driven from here — no OLED, no LED, no
 * buttons — matching the current phase (Kconfig + serial log only).
 *
 * Not implemented here — later phases per spec/wifi_subsystem.md:
 *   - The custom captive-portal / browser path (section 8.4) — zero
 *     install, but a separate chunk of work (HTTP handlers, captive
 *     DNS, an HTML page).
 *   - OLED/LED/button integration (section 12) — needs the display
 *     and button drivers wired to this service.
 *   - Custom protocomm endpoints for MaiaProvApp (section 9) — the
 *     manager already exposes CONFIG_MAIA_WIFI_PROV_CUSTOM_EP as an
 *     unused symbol space if this service later needs it; nothing is
 *     stubbed out here without code behind it.
 *
 * ---------------------------------------------------------------------------
 * NO CONFLICT WITH components/drivers/wifi/
 * ---------------------------------------------------------------------------
 * This service calls maia_wifi_init() for L1 bring-up (netif, event
 * loop, NVS, esp_wifi_init) and reuses maia_wifi_is_provisioned() —
 * both idempotent / read-only, safe to call regardless of what already
 * ran. It never calls maia_wifi_ap_start(): the provisioning manager
 * owns the AP (APSTA mode, its own SoftAP config, its own internal
 * connect-retry loop) for the whole session, and the driver's STA/AP
 * event handlers keep observing WIFI_EVENT/IP_EVENT the entire time —
 * harmlessly, since they only translate events into driver state and
 * never fight the manager for control of the radio. See
 * maia_wifi_sta.c's on_wifi_event() and maia_wifi_ap.c's
 * on_wifi_event(): neither one calls esp_wifi_connect()/disconnect()
 * or esp_wifi_set_mode() on its own — only maia_wifi_sta_start()/
 * _connect()/_stop() and maia_wifi_ap_start()/_stop() do, and this
 * service never calls those while the manager is active.
 *
 ****************************************************************************/

#ifndef __COMPONENTS_SERVICES_WIFI_PROVISIONING_INCLUDE_WIFI_PROV_SERVICE_H
#define __COMPONENTS_SERVICES_WIFI_PROVISIONING_INCLUDE_WIFI_PROV_SERVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wifi_prov_service_run
 *
 * Description:
 *   Blocking. Ensures the device is provisioned and connected:
 *
 *     - If maia_wifi_is_provisioned() is already true, connects
 *       directly using the stored credentials (bounded by
 *       MAIA_WIFI_STA_MAX_RETRY attempts) — no AP is raised.
 *
 *     - Otherwise, raises a SoftAP and runs the wifi_provisioning
 *       manager (security level and AP link security per Kconfig)
 *       until a phone completes pairing, the attempt fails, or
 *       MAIA_WIFI_PROV_TIMEOUT_MIN elapses. A proof-of-possession
 *       code is drawn from the hardware RNG (or taken from
 *       MAIA_WIFI_PROV_POP_STATIC) and printed to the serial log.
 *
 *   On success the station is already connected — the caller does not
 *   need to call maia_wifi_sta_start()/_connect() afterward; it can
 *   query maia_wifi_get_state()/_get_ip()/_get_rssi() immediately.
 *
 * Returned Value:
 *   ESP_OK if connected; ESP_ERR_TIMEOUT if MAIA_WIFI_PROV_TIMEOUT_MIN
 *   elapsed with no result; ESP_FAIL if provisioning or the direct
 *   connect attempt failed.
 *
 ****************************************************************************/

esp_err_t wifi_prov_service_run(void);

#endif /* __COMPONENTS_SERVICES_WIFI_PROVISIONING_INCLUDE_WIFI_PROV_SERVICE_H */
