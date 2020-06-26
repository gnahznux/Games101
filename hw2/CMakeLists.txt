cmake_minimum_required(VERSION 3.10)
project(RasterizerHw2)


set(CMAKE_CXX_STANDARD 17)


add_executable(RasterizerHw2 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(RasterizerHw2 ${OpenCV_LIBRARIES})
