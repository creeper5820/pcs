cmake_minimum_required(VERSION 3.20)

project(pcs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL COMPONENTS io common visualization REQUIRED)
find_package(VTK REQUIRED)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src/qwidget/")
file(GLOB_RECURSE PROJECT_CPP "src/*.cpp")
file(GLOB_RECURSE PROJECT_HPP "src/*.hpp")


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
    ${PROJECT_CPP}
    ${PROJECT_HPP}
    "res/resource.qrc"
)

target_link_libraries(
    ${PROJECT_NAME} 
    ${PCL_LIBRARIES} 
    Qt5::Core
)  
