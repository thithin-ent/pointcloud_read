cmake_minimum_required(VERSION 3.10)

project( pointcloude_read )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL 1.8 REQUIRED)   
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR}/include )

add_definitions( ${PCL_DEFINITIONS} )

set( NEEDS_LIBS 
    ${PCL_LIBRARIES}
)

# add_subdirectory(lib)
add_subdirectory(src)
