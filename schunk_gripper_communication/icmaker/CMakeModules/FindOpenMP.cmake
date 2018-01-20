# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is part of the icmaker build system.
#
# This program is free software licensed under the BSD License. You can
# find a copy of this license in the LICENSE folder in the top directory
# of the source code.
#
# © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

include(${CMAKE_ROOT}/Modules/FindOpenMP.cmake)

IF(OPENMP_FOUND)
  SET(OpenMP_DEFINITIONS ${OpenMP_CXX_FLAGS})
  IF(NOT WIN32)
    SET(OpenMP_LIBRARIES ${OpenMP_CXX_FLAGS})
  ENDIF()
ENDIF()
