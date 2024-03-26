# Install script for directory: /home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/jesse/Thesis/code_jesse/qb_hand/devel/lib/libSerial.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake"
         "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/CMakeFiles/Export/_home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/CMakeFiles/Export/_home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig-noconfig.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial" TYPE FILE FILES "/home/jesse/Thesis/code_jesse/qb_hand/build/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/CMakeFiles/Export/_home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_driver/api/qbdevice-api-7.x.x/serial/SerialConfig-noconfig.cmake")
  endif()
endif()

