## ======================================================================== ##
## Copyright 2009-2013 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

SET(FLAGS_SSSE3 "-xssse3")
SET(FLAGS_SSE41 "-xsse41")
SET(FLAGS_SSE42 "-xsse42")
SET(FLAGS_AVX   "-xAVX")
SET(FLAGS_AVX2  "-xCORE-AVX2")

SET(CMAKE_CXX_COMPILER "icpc")
SET(CMAKE_C_COMPILER "icc")
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -static-intel -fvisibility-inlines-hidden -fvisibility=hidden")
SET(CMAKE_CXX_FLAGS_DEBUG "-DDEBUG -g -O0")
SET(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -no-ansi-alias -restrict -fp-model fast -fimf-precision=low -no-prec-div -no-prec-sqrt")
SET(CMAKE_EXE_LINKER_FLAGS "") 

IF (APPLE)
  SET (CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS_INIT} -dynamiclib)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7")
ENDIF (APPLE)

SET(EXT "")
