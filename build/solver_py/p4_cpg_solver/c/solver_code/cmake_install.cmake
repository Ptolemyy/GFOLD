# Install script for directory: C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/gfold")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/out/ecos.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/out/ecos.dll")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecos" TYPE FILE FILES
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/external/SuiteSparse_config/SuiteSparse_config.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/cone.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/ctrlc.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/data.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/ecos.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/ecos_bb.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/equil.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/expcone.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/glblopts.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/kkt.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/spla.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/splamm.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/timer.h"
    "C:/Users/32796/Desktop/GFOLD/solver_py/p4_cpg_solver/c/solver_code/include/wright_omega.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos/ecos-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos/ecos-targets.cmake"
         "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/solver_code/CMakeFiles/Export/1486f10f4b53bd9f1a0cd8f44f37b237/ecos-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos/ecos-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos/ecos-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos" TYPE FILE FILES "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/solver_code/CMakeFiles/Export/1486f10f4b53bd9f1a0cd8f44f37b237/ecos-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos" TYPE FILE FILES "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/solver_code/CMakeFiles/Export/1486f10f4b53bd9f1a0cd8f44f37b237/ecos-targets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ecos" TYPE FILE FILES "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/solver_code/ecos-config.cmake")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "C:/Users/32796/Desktop/GFOLD/build/solver_py/p4_cpg_solver/c/solver_code/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
