add_executable(
    process_model_to_pointcloud
    process-model-to-pointcloud.cc
    ../src/core/renderer.cc
    ../src/core/renderer.vtk.cc
    ../src/core/units/points.cc
    ../src/core/handle/points.cc
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
        Qt6::Widgets
        Qt6::OpenGLWidgets
        spdlog::spdlog
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
)

add_custom_command(
    TARGET process_model_to_pointcloud
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        /usr/lib/libboost_iostreams.so
        $<TARGET_FILE_DIR:process_model_to_pointcloud>/libboost_iostreams.so.1.87.0
)

add_test(
    NAME process_model_to_pointcloud
    COMMAND process_model_to_pointcloud
)

set_tests_properties(
    process_model_to_pointcloud
    PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:process_model_to_pointcloud>:/usr/lib"
)

add_executable(
    model_handle_obj_without_mtl
    model-handle-obj-without-mtl.cc
    ../src/core/renderer.cc
    ../src/core/renderer.vtk.cc
    ../src/core/units/model.cc
    ../src/core/handle/model.cc
)

target_include_directories(
    model_handle_obj_without_mtl
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
    model_handle_obj_without_mtl
    PRIVATE
        Qt6::Widgets
        Qt6::OpenGLWidgets
        spdlog::spdlog
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
)

add_test(
    NAME model_handle_obj_without_mtl
    COMMAND model_handle_obj_without_mtl
)

set_tests_properties(
    model_handle_obj_without_mtl
    PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:model_handle_obj_without_mtl>:/usr/lib"
)

vtk_module_autoinit(
    TARGETS process_model_to_pointcloud
            model_handle_obj_without_mtl
    MODULES ${VTK_LIBRARIES}
)
