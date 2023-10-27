file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(geo_mesh ${sources})
add_library(geo::mesh ALIAS geo_mesh)

target_include_directories(geo_mesh
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}>
)

find_package(Eigen3 REQUIRED NO_MODULE)

target_link_libraries(geo_mesh
PUBLIC
    Eigen3::Eigen
)

install(TARGETS geo_mesh
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

install(DIRECTORY include/
    DESTINATION include/geo_framework-${PROJECT_VERSION}
)
