cmake_minimum_required(VERSION 3.17)

cmake_policy(SET CMP0048 NEW)
project(DOFMANAGER VERSION 1.0.0 LANGUAGES CXX)

#########################################
# Variables that configure this project #
#########################################

set(PUBLIC_MODULE_NAME "DofManager")
set(MAIN_COMPONENT "dof_manager")

set(SHEFF_FIRST_DEPENDS "FiniteElement::finite_element")
set(SHEFF_SECOND_DEPENDS )

# Third-party dependencies (length of all three lists must match).
# Since we cannot assume certain pattern for names, using multiple lists gives
# the flexibility needed for various packages.
set(SHEFF_THIRD_DEPENDS_name )
set(SHEFF_THIRD_DEPENDS_package )
set(SHEFF_THIRD_DEPENDS_target )

# External dependencies need to be found from the system (such as MPI)
# toggle_var indicates the variable name that controls whether the package is included.
# The entry can be "1" for packages that are always included.
set(SHEFF_EXTERNAL_DEPENDS_toggle_var "")
# The name of the package in the call find_package(<package name>)
set(SHEFF_EXTERNAL_DEPENDS_package "")
# Any components to get from the package (use comma delimited for multiple components in the same package)
set(SHEFF_EXTERNAL_DEPENDS_components "")
# The targets to link from the package (use comma delimited for multiple targets in the same package)
set(SHEFF_EXTERNAL_DEPENDS_targets "")
# Extra compile definitions if the package is included
set(SHEFF_EXTERNAL_DEPENDS_compile_defs "")

macro(SPECIAL_TARGET_COMMANDS)
  
endmacro()

# Options
option(SHEFF_BuildTests        "Build the unit tests." OFF)
option(SHEFF_BuildPython       "Build Python SWIG module" OFF)
option(SHEFF_BuildFortran      "Build Fortran SWIG module" OFF)

# For debugging, print generator, compiler, and options
message(STATUS "Gen: ${CMAKE_GENERATOR}, CXX: ${CMAKE_CXX_COMPILER_ID}, Mode: ${CMAKE_BUILD_TYPE}")
message(STATUS "Python Bindings: ${SHEFF_BuildPython}, Fortran Bindings: ${SHEFF_BuildPython}")
message(STATUS "Tests: ${SHEFF_BuildTests}")

message(STATUS "SHEFF_FIRST_DEPENDS: ${SHEFF_FIRST_DEPENDS}")
message(STATUS "SHEFF_SECOND_DEPENDS: ${SHEFF_SECOND_DEPENDS}")
message(STATUS "SHEFF_THIRD_DEPENDS_name: ${SHEFF_THIRD_DEPENDS_name}")

include("$ENV{SHEFF_TOOLS}/cmake/SHEFFModuleMain.cmake")
