execute_process(COMMAND "/home/robot/2.120-Project/Project_files/catkin_ws/build/image_pipeline-kinetic/camera_calibration/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robot/2.120-Project/Project_files/catkin_ws/build/image_pipeline-kinetic/camera_calibration/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
