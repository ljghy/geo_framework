set(EXAMPLES 
    TraceZeroLevelSet
    HeatGeodesicDistance
    Seg2SDF
    SmoothKVectorField
)

foreach(example ${EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PRIVATE geo_numeric geo_mesh geo_io geo_field)
endforeach()
