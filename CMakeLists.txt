cmake_minimum_required(VERSION 2.8)

project( captureImage )
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package( OpenCV REQUIRED )
FIND_PACKAGE( PkgConfig REQUIRED )
FIND_PACKAGE( freenect2 REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)
add_compile_options(-std=c++11)

include_directories(
	${OPENNI2_DIR}/Include
    	${NITE2_DIR}/Include
	${OPENGL_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${OPENCV_INCLUDE}
    	${CMAKE_CURRENT_SOURCE_DIR}
	)
include_directories("/usr/include/libusb-1.0/")
include_directories( ${freenect2_INCLUDE_DIRS} )

add_executable(captureImage ./captureImage.cpp) 
target_link_libraries(captureImage ${freenect2_LIBRARIES} ${OpenCV_LIBS})