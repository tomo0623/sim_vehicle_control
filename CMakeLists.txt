cmake_minimum_required(VERSION 3.8)
project(SIM_RK4 CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 REQUIRED COMPONENTS Development)

# include_directories(${Python3_INCLUDE_DIRS} ./matplotlib)

add_executable(${CMAKE_PROJECT_NAME} 
                ./src/main.cpp ./src/controller.cpp ./src/dynamics.cpp
                ./matplotlib/python_graph_plotter.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} Eigen3::Eigen ${Python3_LIBRARIES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS} ./matplotlib ./Filter)
