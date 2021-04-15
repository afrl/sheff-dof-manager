cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0048 NEW)
project(DOFMANAGER VERSION 1.0.0 LANGUAGES CXX)

# Options
option(SHEFF_BuildTests "Build the unit tests." OFF)

# For debugging, print generator, compiler, and options
message(STATUS "Gen: ${CMAKE_GENERATOR}, CXX: ${CMAKE_CXX_COMPILER_ID}, Mode: ${CMAKE_BUILD_TYPE}")
message(STATUS "Tests: ${SHEFF_BuildTests}")

# Add cmake dir to module path
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

##################
# Checking input #
##################

include(${PROJECT_SOURCE_DIR}/cmake/CommonInputChecks.cmake)

########################
# Declare dependencies #
########################

if(NOT TARGET FiniteElement::FiniteElement)
    message(STATUS "FiniteElement did not already exist, finding package.")
    find_package(FiniteElement CONFIG REQUIRED HINTS "${PROJECT_SOURCE_DIR}/../../first_party/finite_element/build")
endif()

################################
# Setting up DOFMANAGER target #
################################

# Set the variables for DOFMANAGER
set(DOFMANAGER_INCLUDE ${PROJECT_SOURCE_DIR}/include)
set(DOFMANAGER_SRC ${PROJECT_SOURCE_DIR}/source)
file (GLOB_RECURSE DOFMANAGER_HEADER_LIST "${DOFMANAGER_INCLUDE}/*.h")
file (GLOB_RECURSE DOFMANAGER_SOURCE_LIST "${DOFMANAGER_SRC}/*.h" "${DOFMANAGER_SRC}/*.cpp")
    
# Create target
add_library(dofmanager STATIC)
# Add the header includes and headers to the library
target_include_directories(dofmanager PUBLIC
    $<BUILD_INTERFACE:${DOFMANAGER_INCLUDE}>
    $<INSTALL_INTERFACE:include>)
# Add headers in the include folder as PUBLIC so downstream
# targets have access to them
target_sources(dofmanager PUBLIC ${DOFMANAGER_HEADER_LIST})
# Add headers and sources in the source folder as PRIVATE as they
# will not be needed downstream
target_sources(dofmanager PRIVATE ${DOFMANAGER_SOURCE_LIST})
# Add an alias so that a library can be used inside the build tree, eg when testing
add_library(DofManager::DofManager ALIAS dofmanager)
# Add C++ 14 flag
target_compile_features(dofmanager PUBLIC cxx_std_14)
set_target_properties(dofmanager PROPERTIES FOLDER "sheff")
# Link dependencies
target_link_libraries(dofmanager PUBLIC Utility::Utility)
target_link_libraries(dofmanager PUBLIC SimpleMath::SimpleMath)
target_link_libraries(dofmanager PUBLIC Constitutive::Constitutive)
target_link_libraries(dofmanager PUBLIC Mesh::Mesh)
target_link_libraries(dofmanager PUBLIC FeaIo::FeaIo)
target_link_libraries(dofmanager PUBLIC FiniteElement::FiniteElement)

###########################
# Assigning source groups #
###########################

include("$ENV{SHEFF_TOOLS}/cmake/IncludeSHEFF.cmake")
register_sources_sheff("dof_manager"
                       "${DOFMANAGER_HEADER_LIST}"
                       "${DOFMANAGER_SOURCE_LIST}")
assign_source_groups_sheff()

#################################
# Add unit test targets #
#################################
    
# Build tests
if(SHEFF_BuildTests)
    include(CTest)
    enable_testing()
    add_subdirectory(unit_tests)
endif()

################
# Installation #
################

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/DofManager)

# Install the dofmanager target (library and headers)
install(
    TARGETS dofmanager
    EXPORT DofManagerTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
# This is required so that the exported target has the name DofManager and not dofmanager
set_target_properties(dofmanager PROPERTIES EXPORT_NAME DofManager)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DofManagerConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Create the package config file for find_package(DofManager)
configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/DofManagerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/DofManagerConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install the config and config version files
# Note: if you write custom find modules, add them here
# Refer to https://github.com/pabloariasal/modern-cmake-sample/blob/cfda7b51ff915409fd61d4f2e75b63032bedb182/libjsonutils/CMakeLists.txt
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/DofManagerConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/DofManagerConfigVersion.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/DofManagerTargets.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

###########
# Exports #
###########

# Export just the dofmanager target (not tests, etc.)
export(
    TARGETS dofmanager
    FILE ${CMAKE_CURRENT_BINARY_DIR}/DofManagerTargets.cmake
    NAMESPACE DofManager::
)
