set(TESTS 
    testTraceZeroLevelSet
    testCotanLaplacian
    testGradient
    testDivergence
    testHeatGeodesic
    testSeg2SDF
    testConnectionLaplacian
)

foreach(test ${TESTS})
    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} PRIVATE geo_numeric geo_mesh geo_io geo_field)
endforeach()
