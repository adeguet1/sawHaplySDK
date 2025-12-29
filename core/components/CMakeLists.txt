#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawHaplySDK VERSION 2.1.0)

# create a list of required cisst libraries
  set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

  # WebSocket++ dependency
  find_package(WebSocketPP REQUIRED)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawHaplySDK requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()


  # create/configure file for find_package (sawHaplySDK)
  set (sawHaplySDK_INCLUDE_DIR
       "${sawHaplySDK_SOURCE_DIR}/include"
       "${sawHaplySDK_BINARY_DIR}/include")
  set (sawHaplySDK_HEADER_DIR "${sawHaplySDK_SOURCE_DIR}/include/sawHaplySDK")
  set (sawHaplySDK_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawHaplySDK_LIBRARIES sawHaplySDK WebSocketPP::websocketpp)

  include_directories (${sawHaplySDK_INCLUDE_DIR})

  # add Qt code
  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawHaplySDK_LIBRARIES ${sawHaplySDK_LIBRARIES} sawHaplySDKQt)
  endif (CISST_HAS_QT)

  # add all config files for this component
  cisst_add_config_files (sawHaplySDK)

  add_library (sawHaplySDK
               ${sawHaplySDK_HEADER_DIR}/sawHaplySDKExport.h
               code/mtsHaply.cpp
               ${sawHaplySDK_HEADER_DIR}/mtsHaply.h)
  set_target_properties (sawHaplySDK PROPERTIES
                         VERSION ${sawHaplySDK_VERSION}
                         FOLDER "sawHaplySDK")
  cisst_target_link_libraries (sawHaplySDK ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY
           "${sawHaplySDK_SOURCE_DIR}/include/sawHaplySDK"
           DESTINATION include
           COMPONENT sawHaplySDK-dev)

  install (TARGETS sawHaplySDK COMPONENT sawHaplySDK
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
