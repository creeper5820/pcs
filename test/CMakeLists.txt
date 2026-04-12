add_executable(
    process_model_to_pointcloud
    process-model-to-pointcloud.cc
    ../src/core/events/process/model-to-pointcloud.cc
)

target_include_directories(
    process_model_to_pointcloud
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
    process_model_to_pointcloud
    PRIVATE
        spdlog::spdlog
        ${VTK_LIBRARIES}
)

add_test(
    NAME process_model_to_pointcloud
    COMMAND process_model_to_pointcloud
)

vtk_module_autoinit(
    TARGETS process_model_to_pointcloud
    MODULES ${VTK_LIBRARIES}
)
