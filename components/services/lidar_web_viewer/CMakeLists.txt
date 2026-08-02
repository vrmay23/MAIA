# ===========================================================================
# components/services/lidar_web_viewer/CMakeLists.txt
#
# LiDAR "Pin Art" web viewer: serves an embedded, dependency-free HTML/JS
# page plus a JSON frame endpoint over esp_http_server. Sits above
# components/drivers (vl53l5cx) the same way wifi_prov_service sits above
# the wifi driver — this is policy/presentation, not sensor mechanism.
#
# Like wifi_prov_service, this lives under components/services/, which is
# in EXTRA_COMPONENT_DIRS (see the root CMakeLists.txt) and therefore
# always built regardless of REQUIRES — see the whole-file
# "#if defined(CONFIG_MAIA_TEST_LIDAR_VIEWER)" guard in
# src/lidar_web_viewer.c for how it stays a harmless empty translation
# unit when the feature is off.
# ===========================================================================

idf_component_register(
    SRCS
        "src/lidar_web_viewer.c"
    INCLUDE_DIRS
        "include"
    EMBED_FILES
        "web/index.html"
    REQUIRES
        drivers
        esp_http_server
)
