cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

# find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

file(COPY  ./models
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

add_executable(Rasterizer_3 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer_3 ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)