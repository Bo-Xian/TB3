add_executable(motion_detection main.cpp nodehandle.cpp)
target_link_libraries(motion_detection ${catkin_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(motion_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(motion_detection PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(motion_detection ${OpenCV_LIBRARIES})
