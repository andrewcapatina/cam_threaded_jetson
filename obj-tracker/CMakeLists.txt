cmake_minimum_required(VERSION 3.1.0)
add_library(obj-tracker obj-tracker.cpp)

set(THREADS_PREFER_PTHREAD_FLAG ON)
add_subdirectory(servo_driver)
add_subdirectory(nrf24)

find_package(Threads REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
#message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CUDA_INCLUDE_DIRS} )
include_directories( jetson-utils )
include_directories( jetson-inference )

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

target_link_libraries(  obj-tracker
			Threads::Threads
                        servo_driver
                        nrf24
                        jetson-inference
                        jetson-utils
                        ${CUDA_LIBRARIES}
                        ${OpenCV_LIBRARIES})
