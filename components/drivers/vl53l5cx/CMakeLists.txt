# ============================================================
# components/drivers/vl53l5cx/CMakeLists.txt
#
# VL53L5CX ToF sensor driver
#
# This CMakeLists.txt is used by the driver aggregator
# (components/drivers/CMakeLists.txt) to locate sources
# and includes. It also resolves the ST ULD sources
# downloaded by the IDF Component Manager into
# managed_components/.
# ============================================================

# Path to the ULD downloaded by idf_component.yml
# The Component Manager places it under:
# managed_components/stm32-vl53l5cx/

set(ULD_DIR
    "${CMAKE_SOURCE_DIR}/managed_components/stm32-vl53l5cx")

set(ULD_MODULES_DIR "${ULD_DIR}/modules")

# ULD source files (mandatory for basic ranging)
set(ULD_SRCS
    "${ULD_MODULES_DIR}/vl53l5cx_api.c"
)

# MAIA driver sources
set(MAIA_TOF_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vl53l5cx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vl53l5cx_platform.c"
)

# All sources
set(VL53L5CX_SRCS ${MAIA_TOF_SRCS} ${ULD_SRCS})

# Include directories
set(VL53L5CX_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ULD_MODULES_DIR}"
    "${ULD_DIR}/porting"
)
