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

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawHaplySDK requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  # this should be replaced by a find HaplySDK
  set (haply_sdk_FOUND 0)
  if (WIN32)
    message ("this needs to be implemented for Windows")
  elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_path (haply_sdk_DIR
               NAMES "dhdc.h"
               PATH_SUFFIXES "include")
    if (EXISTS "${haply_sdk_DIR}/include/Inverse3.h")
      set (haply_sdk_FOUND 1)
      include_directories (${haply_sdk_DIR}/include)
      # libraries
      link_directories (${haply_sdk_DIR})
      set (haply_sdk_LIBRARIES Haply.HardwareAPI)
      # copy the binary provided for the Haply SDK in cisst/saw library directory
      # find_library (haply_sdk_LIBRARY
      #              NAMES Haply.HardwareAPI
      #              PATHS ${haply_sdk_DIR})

      # file (MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
      # file (COPY ${haply_sdk_LIBRARY}
      #      DESTINATION ${LIBRARY_OUTPUT_PATH})
    endif ()
  else (WIN32)
    # Everything else
    message ("this needs to be implemented for Mac")
  endif (WIN32)

  # if we found the actracsys SDK
  if (haply_sdk_FOUND)
    # create/configure file for find_package (sawHaplySDK)
    set (sawHaplySDK_INCLUDE_DIR
         "${sawHaplySDK_SOURCE_DIR}/include"
         "${sawHaplySDK_BINARY_DIR}/include")
    set (sawHaplySDK_HEADER_DIR "${sawHaplySDK_SOURCE_DIR}/include/sawHaplySDK")
    set (sawHaplySDK_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${haply_sdk_DIR}")
    set (sawHaplySDK_LIBRARIES sawHaplySDK ${haply_sdk_LIBRARIES})

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
    target_link_libraries (sawHaplySDK
                           ${haply_sdk_LIBRARY_DHD}
                           ${haply_sdk_LIBRARY_DRD}
                           usb-1.0)
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

  else (haply_sdk_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Force Dimension SDK.  Please set haply_sdk_DIR using CMake, e.g. ccmake ${CMAKE_CURRENT_BINARY_DIR} or cmake-gui ${CMAKE_CURRENT_BINARY_DIR}")
  endif (haply_sdk_FOUND)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
