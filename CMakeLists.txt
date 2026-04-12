cmake_minimum_required(VERSION 3.22)

project(pcs VERSION 0.1 LANGUAGES C CXX)

enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Os -O3)

if(NOT DEFINED VTK_DIR AND DEFINED ENV{VTK_DIR})
    set(VTK_DIR "$ENV{VTK_DIR}" CACHE PATH "Path to the VTK package configuration")
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets Network OpenGLWidgets)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common filters)
find_package(VTK COMPONENTS
    CommonColor
    CommonCore
    CommonDataModel
    FiltersCore
    FiltersGeneral
    FiltersModeling
    FiltersSources
    GUISupportQt
    InteractionStyle
    IOGeometry
    RenderingAnnotation
    RenderingContextOpenGL2
    RenderingCore
    RenderingFreeType
    RenderingGL2PSOpenGL2
    RenderingOpenGL2
)

include(FetchContent)
FetchContent_Declare(
    creeper-qt
    GIT_REPOSITORY https://github.com/creeper5820/creeper-qt.git
    GIT_TAG main
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(creeper-qt)

file(GLOB_RECURSE PROJECT_CC CONFIGURE_DEPENDS "src/*.cc")
file(GLOB_RECURSE PROJECT_HH CONFIGURE_DEPENDS "src/*.hh")

add_executable(${PROJECT_NAME})
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        ${PROJECT_CC}
        ${PROJECT_HH}
)
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        Qt6::Widgets
        Qt6::Network
        Qt6::OpenGLWidgets
        creeper-qt::widgets
        spdlog::spdlog
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
)
vtk_module_autoinit(
    TARGETS ${PROJECT_NAME}
    MODULES ${VTK_LIBRARIES}
)

add_subdirectory(test)
