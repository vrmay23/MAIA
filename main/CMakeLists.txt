# ===========================================================================
# main/CMakeLists.txt
#
# Main application component configuration
# ===========================================================================

# Start with main.c as base source
set(MAIN_SRCS "main.c")

# Conditionally add test sources when test mode is enabled
# This allows keeping test code separate from production code
if(CONFIG_MAIA_TEST_ENABLE)
    list(APPEND MAIN_SRCS "tests/test_blink.c")
    list(APPEND MAIN_SRCS "tests/test_button.c")

    # A test can only be built while its device is enabled, for the same
    # reason as the drivers: the device's Kconfig symbols disappear with
    # the menu. The matching "depends on" on each MAIA_TEST_* keeps the
    # menu from offering a test whose sources are not compiled.

    if(CONFIG_MAIA_DS18B20_ENABLE)
        list(APPEND MAIN_SRCS "tests/test_ds18b20.c")
    endif()

    if(CONFIG_MAIA_DRV2605L_ENABLE)
        list(APPEND MAIN_SRCS "tests/test_drv2605l.c")
    endif()

    if(CONFIG_MAIA_SSD1306_ENABLE)
        list(APPEND MAIN_SRCS "tests/test_ssd1306.c")
    endif()

    if(CONFIG_MAIA_VL53L5CX_ENABLE)
        list(APPEND MAIN_SRCS "tests/test_vl53l5cx.c")
    endif()

    if(CONFIG_MAIA_IMU_MPU6050)
        list(APPEND MAIN_SRCS "tests/test_mpu6050.c")
    endif()

    if(CONFIG_MAIA_WIFI_ENABLE)
        list(APPEND MAIN_SRCS "tests/test_wifi.c")
    endif()

    # Add more test files here as needed:
    # list(APPEND MAIN_SRCS "tests/test_i2c.c")
    # list(APPEND MAIN_SRCS "tests/test_sensors.c")
endif()

# Register the main component with ESP-IDF build system
idf_component_register(
    # Source files to compile
    SRCS
        ${MAIN_SRCS}
    
    # Public include directories
    # Only "." is needed (main/ directory itself)
    INCLUDE_DIRS
        "."
    
    # Component dependencies
    # These components must be built before main
    #
    # wifi_prov_service is REQUIRES unconditionally (not gated on
    # CONFIG_MAIA_WIFI_ENABLE) on purpose — ESP-IDF resolves each
    # component's REQUIRES/PRIV_REQUIRES in an early pass, before
    # Kconfig values from the actual sdkconfig are reliably available,
    # so conditioning REQUIRES on a CONFIG_* value silently produces
    # the Kconfig *default* rather than what is actually configured
    # (confirmed by hitting this directly: the component vanished from
    # main's REQUIRES even with MAIA_WIFI_ENABLE=y set). SRCS
    # conditionals below do not have this problem — they are evaluated
    # later, with the real sdkconfig in hand — which is why disabling
    # the *content* of wifi_prov_service.c is instead handled inside
    # that file itself, via a whole-file "#if CONFIG_MAIA_WIFI_ENABLE"
    # guard: the component always builds, but compiles to an empty,
    # harmless translation unit when WiFi is disabled.
    REQUIRES
        app                # Application logic component
        maia_board         # Board support package (BSP)
        drivers            # Hardware drivers (button, sensors, etc)
        wifi_prov_service  # WiFi provisioning policy (services/wifi_prov_service)
        console
        # Add more component dependencies here as needed:
        # services   # High-level services (if created)
)
