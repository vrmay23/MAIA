# ===========================================================================
# components/drivers/CMakeLists.txt
#
# Drivers component wrapper.
# VL53L5CX ULD sources pulled from managed_components/
# via idf_component.yml. ST porting/ is excluded —
# MAIA provides platform.h in vl53l5cx/include/ which
# shadows the ST version via INCLUDE_DIRS ordering.
# ===========================================================================

set(ULD_DIR
    "${CMAKE_SOURCE_DIR}/managed_components/stm32-vl53l5cx")
set(ULD_MODULES_DIR "${ULD_DIR}/modules")

set(ULD_SRCS "")
set(ULD_INCLUDE_DIRS "")

if(EXISTS "${ULD_MODULES_DIR}/vl53l5cx_api.c")
    list(APPEND ULD_SRCS
        "${ULD_MODULES_DIR}/vl53l5cx_api.c")
    list(APPEND ULD_INCLUDE_DIRS
        "${ULD_MODULES_DIR}")
    message(STATUS "VL53L5CX ULD found")
else()
    message(WARNING
        "VL53L5CX ULD not found. Run idf.py build "
        "to download via Component Manager.")
endif()

idf_component_register(
    SRCS
        "button/src/button.c"
        "ds18b20/src/ds18b20.c"
        "drv2605l/src/drv2605l.c"
        "mpu6050/src/mpu6050.c"
        "ssd1306/src/ssd1306.c"
        "vl53l5cx/src/vl53l5cx.c"
        "vl53l5cx/src/vl53l5cx_platform.c"
        ${ULD_SRCS}
    INCLUDE_DIRS
        "button/include"
        "ds18b20/include"
        "drv2605l/include"
        "mpu6050/include"
        "ssd1306/include"
        "vl53l5cx/include"
        ${ULD_INCLUDE_DIRS}
    PRIV_INCLUDE_DIRS
        "."
    REQUIRES
        driver
        maia_board
        esp_timer
)
