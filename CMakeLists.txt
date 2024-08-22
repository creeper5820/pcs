cmake_minimum_required(VERSION 3.20)

project(pcs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src/ui/")
aux_source_directory(src/ SRC)
aux_source_directory(src/ui/workspace/ SRC)
aux_source_directory(src/ui/workspace/viewer/ SRC)
aux_source_directory(src/core/viewer/ SRC)
aux_source_directory(src/core/operator/ SRC)

find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
find_package(PCL COMPONENTS io common visualization REQUIRED)
find_package(VTK REQUIRED)

include_directories(src)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
    ${PROJECT_NAME}
    ${SRC}
)

target_link_libraries(
    ${PROJECT_NAME} 
    ${PCL_LIBRARIES} 
    Qt5::Widgets
)
