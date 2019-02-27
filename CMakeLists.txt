cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0048 NEW)
project(DOFMANAGER VERSION 1.0.0 LANGUAGES CXX)

# Options
option(DOFMANAGER_BuildDev "Build the project in the dev folder. (intended for debugging this project by devs)" OFF)
option(DOFMANAGER_BuildTests "Build the unit tests." OFF)

##################
# Checking input #
##################

include(${CMAKE_SOURCE_DIR}/cmake/CommonInputChecks.cmake)

########################
# Declare dependencies #
########################


find_package(Utility CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../utility/build" "${CMAKE_SOURCE_DIR}/../utility/install_dir")
find_package(SimpleMath CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../simple_math/build" "${CMAKE_SOURCE_DIR}/../simple_math/install_dir")
find_package(Constitutive CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../constitutive/build" "${CMAKE_SOURCE_DIR}/../constitutive/install_dir")
find_package(FlexMesh CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../flex_mesh/build" "${CMAKE_SOURCE_DIR}/../flex_mesh/install_dir")
find_package(FeaIo CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../fea_io/build" "${CMAKE_SOURCE_DIR}/../fea_io/install_dir")
find_package(FiniteElement CONFIG REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../finite_element/build" "${CMAKE_SOURCE_DIR}/../finite_element/install_dir")


################################
# Setting up DOFMANAGER target #
################################

# Set the variables for DOFMANAGER
set(DOFMANAGER_INCLUDE ${CMAKE_SOURCE_DIR}/include)
set(DOFMANAGER_INCLUDE_SUB ${DOFMANAGER_INCLUDE}/dof_manager)
set(DOFMANAGER_SRC ${CMAKE_SOURCE_DIR}/source)
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
# Link dependencies
target_link_libraries(dofmanager PUBLIC Utility::Utility)
target_link_libraries(dofmanager PUBLIC SimpleMath::SimpleMath)
target_link_libraries(dofmanager PUBLIC Constitutive::Constitutive)
target_link_libraries(dofmanager PUBLIC FlexMesh::FlexMesh)
target_link_libraries(dofmanager PUBLIC FeaIo::FeaIo)
target_link_libraries(dofmanager PUBLIC FiniteElement::FiniteElement)


# Assign source groups
include(${CMAKE_SOURCE_DIR}/cmake/Utilities.cmake)
assign_source_group("dof_manager" ${DOFMANAGER_HEADER_LIST})
assign_source_group("dof_manager" ${DOFMANAGER_SOURCE_LIST})
assign_source_group_Utility()
assign_source_group_SimpleMath()
assign_source_group_Constitutive()
assign_source_group_FlexMesh()
assign_source_group_FeaIo()
assign_source_group_FiniteElement()


#################################
# Add dev and unit test targets #
#################################

# Build dev
if(DOFMANAGER_BuildDev)
    add_subdirectory(dev)
endif()
    
# Build tests
if(DOFMANAGER_BuildTests)
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

# Export just the dofmanager target (not the dev, tests, etc.)
export(
    TARGETS dofmanager
    FILE ${CMAKE_CURRENT_BINARY_DIR}/DofManagerTargets.cmake
    NAMESPACE DofManager::
)

# Register package in user's package registry
export(PACKAGE DofManager)
