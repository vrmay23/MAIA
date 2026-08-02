# ===========================================================================
# components/drivers/wifi/CMakeLists.txt
#
# Not used directly — ESP-IDF component discovery is non-recursive, so
# only components/drivers itself (the aggregator) is registered as a
# component. This file exists for readers, matching the (also unused)
# CMakeLists.txt present in the mpu6050/ssd1306/lsm6dsox driver folders.
# The sources and include dirs below are the ones actually wired into
# components/drivers/CMakeLists.txt.
# ===========================================================================

idf_component_register(
    SRCS
        "src/maia_wifi.c"
        "src/maia_wifi_sta.c"
        "src/maia_wifi_ap.c"
        "src/maia_wifi_scan.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
)
