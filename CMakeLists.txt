cmake_minimum_required(VERSION 3.14)

project(geo_framework VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/geo_framework-${PROJECT_VERSION}")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)

if (MSVC)
    add_compile_options(/W4 /MP)
else() # GCC
    add_compile_options(-Wall -Wextra -fPIC)
endif()

add_subdirectory(numeric)
add_subdirectory(mesh)
add_subdirectory(io)
add_subdirectory(field)

option(GEO_FRAMEWORK_BUILD_EXAMPLES "Build examples" ON)

if (GEO_FRAMEWORK_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

install(EXPORT geo_framework-targets
  FILE geo_framework-targets.cmake
  NAMESPACE geo::
  DESTINATION lib/cmake/geo_framework-${PROJECT_VERSION}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/geo_framework-config.cmake"
  INSTALL_DESTINATION lib/cmake/geo_framework
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/geo_framework-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/geo_framework-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/geo_framework-config-version.cmake"
  DESTINATION lib/cmake/geo_framework-${PROJECT_VERSION}
)

export(EXPORT geo_framework-targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/geo_framework-targets.cmake"
  NAMESPACE geo::
)
