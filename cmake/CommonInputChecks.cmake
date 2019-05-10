
# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
	message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

# Guard against empty build-type strings
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

# Guard against invalid build-type strings
string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release"
   AND NOT cmake_build_type_tolower STREQUAL "relwithdebinfo")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are Debug, Release, RelWithDebInfo (case-insensitive).")
endif()

# Ensure we have C++ 14 support
#include(CheckCXXCompilerFlag)
#check_cxx_compiler_flag("-std=c++14" ${PROJECT_NAME}_COMPILER_SUPPORT_CPP14)
#if(NOT ${PROJECT_NAME}_COMPILER_SUPPORT_CPP14)
#   message(FATAL_ERROR "The compiler must support C++ 14.")
#endif()