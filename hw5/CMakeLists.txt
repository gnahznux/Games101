cmake_minimum_required(VERSION 3.10)
project(RayTracing_1)

# set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing_1 main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
target_compile_options(RayTracing_1 PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type )
target_compile_features(RayTracing_1 PUBLIC cxx_std_17)
# target_link_libraries(RayTracing PUBLIC )
