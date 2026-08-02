# ===========================================================================
# components/services/wifi_prov_service/CMakeLists.txt
#
# WiFi provisioning service: owns the policy of getting the device from
# "no credentials" to "connected", using the ESP-IDF wifi_provisioning
# manager (protocomm over SoftAP) as the mechanism. Sits above
# components/drivers/wifi/, which stays mechanism-only.
#
# Unlike components/drivers/*/CMakeLists.txt (vestigial — the real
# aggregator is components/drivers/CMakeLists.txt), this is a real,
# standalone IDF component: components/services/ is listed in
# EXTRA_COMPONENT_DIRS (see the root CMakeLists.txt) precisely so each
# services/<name>/ folder is discovered and built independently.
# ===========================================================================

idf_component_register(
    SRCS
        "src/wifi_prov_service.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        drivers
        wifi_provisioning
        esp_wifi
)
