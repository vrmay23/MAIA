# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# components/services/<name> is one level deeper than IDF's default
# component search (which only looks at direct children of
# components/), so components/services itself needs to be listed
# explicitly for anything under it to be discovered at all.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components/services")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(maiamute)
