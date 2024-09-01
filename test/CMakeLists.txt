cmake_minimum_required(VERSION 3.20)

project(picker)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

find_package(PCL COMPONENTS 
  io common 
  REQUIRED
)
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

include_directories(
  ${PCL_INCLUDE_DIRS} 
)

add_definitions(
    ${PCL_DEFINITIONS} 
)

add_executable(
    ${PROJECT_NAME}
    picker.cc
)

target_link_libraries(
    ${PROJECT_NAME} PRIVATE
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES} 
)

vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}
  MODULES ${VTK_LIBRARIES}
)