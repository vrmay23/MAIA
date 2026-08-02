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
 * components/services/lidar_web_viewer/include/lidar_web_viewer.h
 *
 * MAIA LiDAR Web Viewer — "Pin Art" 3D visualization
 *
 * Serves a local HTML/JS page (no external dependencies, nothing
 * fetched from a CDN) that renders the live VL53L5CX zone grid as a
 * grid of pins: each zone is one pin, pushed out proportionally to
 * how close its target is — same principle as the mechanical Pin Art
 * toy. The page free-orbits in 3D so the operator can inspect the
 * scanned surface contour from any angle.
 *
 * ---------------------------------------------------------------------------
 * SCOPE
 * ---------------------------------------------------------------------------
 * This service owns exactly two things:
 *   1. An esp_http_server instance serving the embedded page (GET /)
 *      and the current frame as JSON (GET /frame.json).
 *   2. A background task polling components/drivers/vl53l5cx for
 *      whichever sensor(s) CONFIG_MAIA_VL53L5CX_MODE has enabled, and
 *      caching the latest raw frame for the HTTP handler to serve.
 *
 * It does not decide when to bring up WiFi or how to reconnect — the
 * caller (main/tests/test_lidar_viewer.c, standing in for an
 * app/service layer, same as wifi_prov_service.c's callers) is
 * expected to have a station connection up (e.g. via
 * wifi_prov_service_run()) before calling lidar_web_viewer_start().
 *
 * No clamping, filtering, or scaling happens on this side: distance_mm
 * and target_status are sent to the browser exactly as
 * maia_tof_get_data() returned them. All the "what does this pin's
 * height/color actually mean" scaling — including the adjustable max
 * range — happens client-side, in JS, where it can be changed live
 * without reflashing. See web/index.html.
 *
 ****************************************************************************/

#ifndef __COMPONENTS_SERVICES_LIDAR_WEB_VIEWER_INCLUDE_LIDAR_WEB_VIEWER_H
#define __COMPONENTS_SERVICES_LIDAR_WEB_VIEWER_INCLUDE_LIDAR_WEB_VIEWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lidar_web_viewer_start
 *
 * Description:
 *   Start the HTTP server (CONFIG_MAIA_LIDAR_VIEWER_HTTP_PORT) and the
 *   background polling task. Requires maia_tof_init() and
 *   maia_tof_start_ranging() to already have been called — this
 *   service only reads frames, it does not own sensor lifecycle.
 *   Idempotent: calling it again while already running returns ESP_OK
 *   without restarting anything.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL / esp_http_server error codes on
 *   failure to bind the listening socket.
 *
 ****************************************************************************/

esp_err_t lidar_web_viewer_start(void);

/****************************************************************************
 * Name: lidar_web_viewer_stop
 *
 * Description:
 *   Stop the polling task and the HTTP server, releasing all
 *   resources. Safe to call even if never started.
 *
 * Returned Value:
 *   ESP_OK on success.
 *
 ****************************************************************************/

esp_err_t lidar_web_viewer_stop(void);

#endif /* __COMPONENTS_SERVICES_LIDAR_WEB_VIEWER_INCLUDE_LIDAR_WEB_VIEWER_H */
