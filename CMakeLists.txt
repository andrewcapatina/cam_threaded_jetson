cmake_minimum_required(VERSION 3.1.0)

project(camera_application_threaded)

set(THREADS_PREFER_PTHREAD_FLAG ON)

add_subdirectory(obj-tracker)

find_package(Threads REQUIRED)

cuda_add_executable(camera_application_threaded main.cpp)

target_link_libraries(camera_application_threaded Threads::Threads obj-tracker)
