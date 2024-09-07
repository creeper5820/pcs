cmake_minimum_required(VERSION 3.20)

project(pcs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Debug")
# add_compile_options(-Os -O3)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(spdlog REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL 1.12 COMPONENTS io common filters REQUIRED)
find_package(VTK COMPONENTS 
  CommonColor
  CommonCore
  CommonDataModel
  FiltersSources
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "ui/")
file(GLOB_RECURSE PROJECT_CPP "src/*.cc")
file(GLOB_RECURSE PROJECT_HPP "src/*.hh")


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
    ${VTK_LIBRARIES}
    Qt5::Core
    spdlog::spdlog
)

vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}
  MODULES ${VTK_LIBRARIES}
)