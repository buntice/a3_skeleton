# CMake generated Testfile for 
# Source directory: /home/bunso/ros2_ws/src/a3_skeleton
# Build directory: /home/bunso/ros2_ws/src/a3_skeleton/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_laserprocessing "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_ros/cmake/run_test_isolated.py" "/home/bunso/ros2_ws/src/a3_skeleton/build/test_results/a3_skeleton/test_laserprocessing.gtest.xml" "--package-name" "a3_skeleton" "--output-file" "/home/bunso/ros2_ws/src/a3_skeleton/build/ament_cmake_gtest/test_laserprocessing.txt" "--command" "/home/bunso/ros2_ws/src/a3_skeleton/build/test_laserprocessing" "--gtest_output=xml:/home/bunso/ros2_ws/src/a3_skeleton/build/test_results/a3_skeleton/test_laserprocessing.gtest.xml")
set_tests_properties(test_laserprocessing PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/bunso/ros2_ws/src/a3_skeleton/build/test_laserprocessing" TIMEOUT "60" WORKING_DIRECTORY "/home/bunso/ros2_ws/src/a3_skeleton/build" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/opt/ros/humble/share/ament_cmake_ros/cmake/ament_add_ros_isolated_gtest.cmake;33;ament_add_gtest;/home/bunso/ros2_ws/src/a3_skeleton/CMakeLists.txt;95;ament_add_ros_isolated_gtest;/home/bunso/ros2_ws/src/a3_skeleton/CMakeLists.txt;0;")
subdirs("gtest")
