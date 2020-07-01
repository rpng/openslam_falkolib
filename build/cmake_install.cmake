# Install script for directory: /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libfalkolib.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/lib/libfalkolib.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/falkolib/Common/GeomUtils.h;/usr/local/include/falkolib/Common/HoughSpectrum.h;/usr/local/include/falkolib/Common/LaserScan.h;/usr/local/include/falkolib/Common/Point.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/falkolib/Common" TYPE FILE FILES
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Common/GeomUtils.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Common/HoughSpectrum.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Common/LaserScan.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Common/Point.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/falkolib/Feature/BSC.h;/usr/local/include/falkolib/Feature/BSCExtractor.h;/usr/local/include/falkolib/Feature/CGH.h;/usr/local/include/falkolib/Feature/CGHExtractor.h;/usr/local/include/falkolib/Feature/Descriptor.h;/usr/local/include/falkolib/Feature/DescriptorExtractor.h;/usr/local/include/falkolib/Feature/FALKO.h;/usr/local/include/falkolib/Feature/FALKOExtractor.h;/usr/local/include/falkolib/Feature/Keypoint.h;/usr/local/include/falkolib/Feature/KeypointExtractor.h;/usr/local/include/falkolib/Feature/OC.h;/usr/local/include/falkolib/Feature/OCExtractor.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/falkolib/Feature" TYPE FILE FILES
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/BSC.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/BSCExtractor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/CGH.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/CGHExtractor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/Descriptor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/DescriptorExtractor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/FALKO.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/FALKOExtractor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/Keypoint.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/KeypointExtractor.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/OC.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Feature/OCExtractor.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/falkolib/Matching/AHTMatcher.h;/usr/local/include/falkolib/Matching/CCDAMatcher.h;/usr/local/include/falkolib/Matching/Matcher.h;/usr/local/include/falkolib/Matching/NNMatcher.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/falkolib/Matching" TYPE FILE FILES
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Matching/AHTMatcher.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Matching/CCDAMatcher.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Matching/Matcher.h"
    "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/include/falkolib/Matching/NNMatcher.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/falkolib/falkolibConfig.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/share/falkolib" TYPE FILE FILES "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/cmake_modules/falkolibConfig.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
