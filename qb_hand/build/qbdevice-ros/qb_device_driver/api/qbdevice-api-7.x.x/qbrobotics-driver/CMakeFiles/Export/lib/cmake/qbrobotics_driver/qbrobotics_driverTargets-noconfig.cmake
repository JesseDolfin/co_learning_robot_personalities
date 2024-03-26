#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qbrobotics_driver::qbrobotics_driver" for configuration ""
set_property(TARGET qbrobotics_driver::qbrobotics_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qbrobotics_driver::qbrobotics_driver PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqbrobotics_driver.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS qbrobotics_driver::qbrobotics_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_qbrobotics_driver::qbrobotics_driver "${_IMPORT_PREFIX}/lib/libqbrobotics_driver.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
