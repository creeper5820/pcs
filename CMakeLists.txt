cmake_minimum_required(VERSION 3.20)

project(pcs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src/qwidget/")
file(GLOB_RECURSE PROJECT_SRC src/*.cpp)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL COMPONENTS io common visualization REQUIRED)
find_package(VTK REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${Qt5Core_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS} 
)

add_definitions(
    ${Qt5Core_DEFINITIONS} 
    ${PCL_DEFINITIONS} 
)

add_executable(
    ${PROJECT_NAME}
    ${PROJECT_SRC}
)

target_link_libraries(
    ${PROJECT_NAME} 
    ${PCL_LIBRARIES} 
    Qt5::Core
)  
