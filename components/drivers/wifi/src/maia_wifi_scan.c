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
 * components/drivers/wifi/src/maia_wifi_scan.c
 *
 * MAIA WiFi Driver — L4 Scan
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "maia_wifi_priv.h"

#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_wifi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[MAIA_WIFI_SCAN]"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmp_rssi_desc
 *
 * Description:
 *   qsort comparator — descending RSSI, so the strongest network is
 *   first.
 *
 ****************************************************************************/

static int cmp_rssi_desc(const void *a, const void *b)
{
  const wifi_ap_record_t *ra = a;
  const wifi_ap_record_t *rb = b;

  return (int)rb->rssi - (int)ra->rssi;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: maia_wifi_scan
 ****************************************************************************/

esp_err_t maia_wifi_scan(maia_wifi_ap_info_t *results, size_t max,
                         size_t *found)
{
  esp_err_t ret;
  uint16_t num_aps = 0;
  wifi_ap_record_t *records;

  if (results == NULL || found == NULL || max == 0)
    {
      return ESP_ERR_INVALID_ARG;
    }

  *found = 0;

  wifi_scan_config_t scan_cfg = {
      .show_hidden = false,
  };

  ret = esp_wifi_scan_start(&scan_cfg, true /* block */);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_wifi_scan_start failed: %s", esp_err_to_name(ret));
      return ret;
    }

  ret = esp_wifi_scan_get_ap_num(&num_aps);
  if (ret != ESP_OK || num_aps == 0)
    {
      maia_wifi_notify(MAIA_WIFI_EVENT_SCAN_DONE, NULL);
      return ret;
    }

  records = calloc(num_aps, sizeof(wifi_ap_record_t));
  if (records == NULL)
    {
      return ESP_ERR_NO_MEM;
    }

  ret = esp_wifi_scan_get_ap_records(&num_aps, records);
  if (ret != ESP_OK)
    {
      free(records);
      return ret;
    }

  qsort(records, num_aps, sizeof(wifi_ap_record_t), cmp_rssi_desc);

  size_t n = num_aps < max ? num_aps : max;

  for (size_t i = 0; i < n; i++)
    {
      strlcpy(results[i].ssid, (char *)records[i].ssid,
              sizeof(results[i].ssid));
      results[i].rssi = records[i].rssi;
      results[i].channel = records[i].primary;
      results[i].needs_password = records[i].authmode != WIFI_AUTH_OPEN;
    }

  *found = n;

  free(records);

  ESP_LOGI(TAG, "Scan found %d network(s), returned %d", num_aps, n);

  maia_wifi_notify(MAIA_WIFI_EVENT_SCAN_DONE, NULL);

  return ESP_OK;
}
