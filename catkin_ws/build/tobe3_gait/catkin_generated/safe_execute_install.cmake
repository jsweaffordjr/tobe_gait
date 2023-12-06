execute_process(COMMAND "/home/jerry/tobe_gait/catkin_ws/build/tobe3_gait/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jerry/tobe_gait/catkin_ws/build/tobe3_gait/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
