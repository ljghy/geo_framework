find_package(glfw3 REQUIRED)

include(FetchContent)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.89.9-docking
)

set(imgui_VERSION 1.18.9)

FetchContent_MakeAvailable(imgui)

add_library(imgui)

target_sources(imgui
PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui
PUBLIC
    $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
    $<BUILD_INTERFACE:${imgui_SOURCE_DIR}/backends>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}/imgui-${imgui_VERSION}>
)

target_link_libraries(imgui
PUBLIC
    glfw
)

install(TARGETS imgui
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

file(GLOB IMGUI_HEADERS ${imgui_SOURCE_DIR}/*.h)
list(APPEND IMGUI_HEADERS
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3_loader.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
)

install(FILES ${IMGUI_HEADERS}
    DESTINATION include/geo_framework-${PROJECT_VERSION}/imgui-${imgui_VERSION}
)

file(GLOB sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(geo_viewer ${sources})
add_library(geo::viewer ALIAS geo_viewer)

target_include_directories(geo_viewer
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/geo_framework-${PROJECT_VERSION}>
)

find_package(glm REQUIRED)
find_package(opengl_framework REQUIRED)

target_link_libraries(geo_viewer
PUBLIC
    imgui
PRIVATE
    glm::glm
    opengl_framework::opengl_framework
)

install(TARGETS geo_viewer
    EXPORT geo_framework-targets
    DESTINATION lib/geo_framework-${PROJECT_VERSION}
)

install(DIRECTORY include/
    DESTINATION include/geo_framework-${PROJECT_VERSION}
)
