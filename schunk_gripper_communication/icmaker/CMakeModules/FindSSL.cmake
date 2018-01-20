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

#----------------------------------------------------------------------
# \file
#
# \author  Jan Oberlaender <oberlaender@fzi.de>
# \date    2014-08-13
#
# Try to find SSL.  Once done, this will define:
#  SSL_FOUND:          System has SSL
#  SSL_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  SSL_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  SSL_DEFINITIONS:    Preprocessor definitions.
#  SSL_LIBRARIES:      only the libraries (w/o the '-l')
#  SSL_LDFLAGS:        all required linker flags
#  SSL_LDFLAGS_OTHER:  all other linker flags
#  SSL_CFLAGS:         all required cflags
#  SSL_CFLAGS_OTHER:   the other compiler flags
#  SSL_VERSION:        version of the module
#  SSL_PREFIX:         prefix-directory of the module
#  SSL_INCLUDEDIR:     include-dir of the module
#  SSL_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(SSL ssl
  HEADERS openssl/md5.h
  LIBRARIES ssl crypto
  )
