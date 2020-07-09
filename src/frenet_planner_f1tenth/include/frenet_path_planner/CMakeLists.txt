cmake_minimum_required(VERSION 3.7)
project(TestFrenet)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O2")

include_directories(include)

link_directories(include)

set(LIBRARIES

)

add_executable(FRENET_TEST
        demo/test.cpp
        src/cubic_spline_planner.cpp 
        src/polynomials.cpp 
        src/frenet_optimal_trajectory.cpp 
)

find_package(PythonLibs 2.7)
target_include_directories(FRENET_TEST PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(FRENET_TEST ${LIBRARIES} ${PYTHON_LIBRARIES})
