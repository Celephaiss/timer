
cmake_minimum_required( VERSION 3.10 )
find_package (glog 0.4.0 REQUIRED)

add_executable(timer main.cpp)

target_link_libraries(timer glog)