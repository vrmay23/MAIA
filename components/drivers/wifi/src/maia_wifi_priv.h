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
 * components/drivers/wifi/src/maia_wifi_priv.h
 *
 * MAIA WiFi Driver — internal shared interface between the L1/L2/L3/L4
 * translation units. Not installed, not part of the public API.
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_WIFI_SRC_MAIA_WIFI_PRIV_H
#define __COMPONENTS_DRIVERS_WIFI_SRC_MAIA_WIFI_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_wifi.h"
#include <esp_err.h>
#include <esp_netif.h>

/****************************************************************************
 * Internal Function Prototypes — maia_wifi.c
 ****************************************************************************/

void maia_wifi_notify(maia_wifi_event_t event, void *data);
void maia_wifi_set_state(maia_wifi_state_t state);
esp_netif_t *maia_wifi_sta_netif(void);
esp_netif_t *maia_wifi_ap_netif(void);

/****************************************************************************
 * Internal Function Prototypes — maia_wifi_sta.c
 ****************************************************************************/

esp_err_t maia_wifi_sta_events_register(void);

/****************************************************************************
 * Internal Function Prototypes — maia_wifi_ap.c
 ****************************************************************************/

esp_err_t maia_wifi_ap_events_register(void);

#endif /* __COMPONENTS_DRIVERS_WIFI_SRC_MAIA_WIFI_PRIV_H */
