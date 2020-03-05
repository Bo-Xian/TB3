add_executable(motion main.cpp nodehandle.cpp)
target_link_libraries(motion ${catkin_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(motion PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(motion ${OpenCV_LIBRARIES})
