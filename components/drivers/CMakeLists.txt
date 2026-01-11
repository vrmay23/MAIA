# ===========================================================================
# components/drivers/CMakeLists.txt
#
# Drivers component wrapper
# This component acts as a container for all hardware drivers
# ===========================================================================

# Register the drivers component with ESP-IDF build system
idf_component_register(
    # Add button source directly in SRCS
    SRCS
        "button/src/button.c"
        # Add more driver sources here as needed:
        # "vl53l5cx/src/vl53l5cx.c"
        # "mpu6050/src/mpu6050.c"
        # "drv2605l/src/drv2605l.c"
        # "ssd1306/src/ssd1306.c"
        # "ds18b20/src/ds18b20.c"
    
    # Public include directories
    # These headers are visible to components that depend on "drivers"
    INCLUDE_DIRS
        "button/include"
        # Add more driver includes here as needed:
        # "vl53l5cx/include"
        # "mpu6050/include"
        # "drv2605l/include"
        # "ssd1306/include"
        # "ds18b20/include"
    
    # Private include directories
    # Only visible within this component
    PRIV_INCLUDE_DIRS
        "."
    
    # Component dependencies
    # These components must be built before drivers
    REQUIRES
        driver        # ESP-IDF driver component (GPIO, I2C, SPI, etc)
        maia_board    # Board support package (hardware abstraction)
        esp_timer     # ESP-IDF timer component (for esp_timer.h)
)