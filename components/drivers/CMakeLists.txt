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

if(CONFIG_MAIA_VL53L5CX_ENABLE)
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
endif()

# Drivers that read device specific Kconfig symbols can only be built
# while that device is enabled: the symbols live in menus with a
# "depends on <dev>_ENABLE" and are simply not emitted otherwise.
# button.c and drv2605l.c are config independent, so they always build.

set(DRV_SRCS
    "button/src/button.c"
    "drv2605l/src/drv2605l.c")

if(CONFIG_MAIA_DS18B20_ENABLE)
    list(APPEND DRV_SRCS "ds18b20/src/ds18b20.c")
endif()

if(CONFIG_MAIA_SSD1306_ENABLE)
    list(APPEND DRV_SRCS "ssd1306/src/ssd1306.c")
endif()

if(CONFIG_MAIA_VL53L5CX_ENABLE)
    list(APPEND DRV_SRCS
        "vl53l5cx/src/vl53l5cx.c"
        "vl53l5cx/src/vl53l5cx_platform.c")
endif()

# MAIA_IMU_DEVICE is a Kconfig choice: only the selected IMU is built.

if(CONFIG_MAIA_IMU_MPU6050)
    list(APPEND DRV_SRCS "mpu6050/src/mpu6050.c")
endif()

if(CONFIG_MAIA_IMU_LSM6DSOX)
    list(APPEND DRV_SRCS "lsm6dsox/src/lsm6dsox.c")
endif()

idf_component_register(
    SRCS
        ${DRV_SRCS}
        ${ULD_SRCS}
    INCLUDE_DIRS
        "button/include"
        "ds18b20/include"
        "drv2605l/include"
        "lsm6dsox/include"
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
