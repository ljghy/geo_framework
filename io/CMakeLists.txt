file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(geo_io ${sources})
add_library(geo::io ALIAS geo_io)

target_include_directories(geo_io
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}>
)

find_package(Eigen3 REQUIRED NO_MODULE)

target_link_libraries(geo_io
PUBLIC
    geo_mesh
    Eigen3::Eigen
)

install(TARGETS geo_io
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

install(DIRECTORY include/
    DESTINATION include/geo_framework-${PROJECT_VERSION}
)
