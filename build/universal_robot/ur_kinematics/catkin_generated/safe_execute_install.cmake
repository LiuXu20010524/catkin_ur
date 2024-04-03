execute_process(COMMAND "/home/liu_xu/liuxu_Documents/catkin_ur/build/universal_robot/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/liu_xu/liuxu_Documents/catkin_ur/build/universal_robot/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
