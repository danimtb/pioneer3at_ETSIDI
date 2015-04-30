# The name of our project is "HELLO". CMakeLists files in this project can 
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and 
# to the root binary directory of the project as ${HELLO_BINARY_DIR}. 
cmake_minimum_required (VERSION 2.6) 
project (libLMS1xx)

OPTION( BUILD_static "Build static library" OFF )
OPTION( BUILD_test "Build test application" OFF )

if( BUILD_static )
add_library (LMS1xx STATIC LMS1xx.cpp)
else( BUILD_static )
add_library (LMS1xx SHARED LMS1xx.cpp)
endif( BUILD_static )

if( BUILD_test )
add_executable ( test test.cpp )
target_link_libraries ( test LMS1xx )
endif( BUILD_test )

install( TARGETS LMS1xx LIBRARY DESTINATION lib )
install( FILES LMS1xx.h DESTINATION include )