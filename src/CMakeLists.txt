add_executable(pointcloud_read 
    pointcloud_read.cpp
)

target_link_libraries(pointcloud_read
    ${NEEDS_LIBS}
)

