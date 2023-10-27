add_library(geo_numeric INTERFACE)
add_library(geo::numeric ALIAS geo_numeric)

find_package(Eigen3 REQUIRED NO_MODULE)

target_include_directories(geo_numeric
INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}>
)

target_link_libraries(geo_numeric
INTERFACE
    Eigen3::Eigen
)

install(TARGETS geo_numeric
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

install(DIRECTORY include/
    DESTINATION include/geo_framework-${PROJECT_VERSION}
)
