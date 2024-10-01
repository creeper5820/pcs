cmake_minimum_required(VERSION 3.20)

project(examples)

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
  CommonTransforms
  FiltersSources
  FiltersGeometry
  InteractionStyle
  RenderingAnnotation
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  ImagingCore
  ImagingSources
)

include_directories(
  ${PCL_INCLUDE_DIRS} 
)

add_definitions(
  ${PCL_DEFINITIONS} 
)

### PICLER
add_executable(
   picker
  picker.cc
)

target_link_libraries(
  picker PRIVATE
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS picker
  MODULES ${VTK_LIBRARIES}
)

### GRID
add_executable(
  grid
  grid.cc
)

target_link_libraries(
  grid PRIVATE
  ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS grid
  MODULES ${VTK_LIBRARIES}
)