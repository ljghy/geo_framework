add_library(acceleration INTERFACE)

find_package(Eigen3 REQUIRED NO_MODULE)

target_include_directories(acceleration
INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(acceleration
INTERFACE
    Eigen3::Eigen
)
