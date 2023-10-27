file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(geo_field ${sources})
add_library(geo::field ALIAS geo_field)

target_include_directories(geo_field
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}>
)

find_package(Eigen3 REQUIRED NO_MODULE)

target_link_libraries(geo_field
PUBLIC
    geo_mesh
    geo_numeric
    Eigen3::Eigen
)

install(TARGETS geo_field
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

install(DIRECTORY include/
    DESTINATION include/geo_framework-${PROJECT_VERSION}
)
