set(EXAMPLES 
    TraceZeroLevelSet
    HeatGeodesicDistance
    Seg2SDF
    SmoothKVectorField
    BasicViewer
)

find_package(opengl_framework REQUIRED)

foreach(example ${EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example}
    PRIVATE 
        geo_numeric
        geo_mesh
        geo_io
        geo_field
        geo_viewer
        opengl_framework::opengl_framework
    )
endforeach()
