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
    list(APPEND MAIN_SRCS "tests/test_ds18b20.c")
    list(APPEND MAIN_SRCS "tests/test_drv2605l.c")
    # Add more test files here as needed:
    # list(APPEND MAIN_SRCS "tests/test_i2c.c")
    # list(APPEND MAIN_SRCS "tests/test_sensors.c")
    # list(APPEND MAIN_SRCS "tests/test_wifi.c")
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
    REQUIRES
        app          # Application logic component
        maia_board   # Board support package (BSP)
        drivers      # Hardware drivers (button, sensors, etc)
        console
        # Add more component dependencies here as needed:
        # services   # High-level services (if created)
)