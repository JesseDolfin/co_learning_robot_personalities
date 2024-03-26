# Install script for directory: /home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jesse/Thesis/code_jesse/qb_hand/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs/srv" TYPE FILE FILES
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
    "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs/cmake" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_srvs/catkin_generated/installspace/qb_device_srvs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/include/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/share/roseus/ros/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/share/common-lisp/ros/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/share/gennodejs/ros/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/jesse/Thesis/code_jesse/qb_hand/devel/lib/python3/dist-packages/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/lib/python3/dist-packages/qb_device_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_srvs/catkin_generated/installspace/qb_device_srvs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs/cmake" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_srvs/catkin_generated/installspace/qb_device_srvs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs/cmake" TYPE FILE FILES
    "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_srvs/catkin_generated/installspace/qb_device_srvsConfig.cmake"
    "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_srvs/catkin_generated/installspace/qb_device_srvsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/qb_device_srvs" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/include/qb_device_srvs/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/qb_device_srvs/srv" TYPE DIRECTORY FILES "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/")
endif()

