###############################################################################
# States that CMake required version must be greater than 2.8
cmake_minimum_required(VERSION 2.8)

###############################################################################
# Project name is not mandatory, but you should use it
project(meka_omnibase_control)

###############################################################################
# Set the PROJECT_ROOT variable
set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

###############################################################################
# Set where to find the FindXXX.cmake
set(CMAKE_MODULE_PATH "${PROJECT_ROOT}/cmake/Modules/" $ENV{M3_CMAKE_MODULES} ${CMAKE_MODULE_PATH})

###############################################################################
# Set all bunch of usefull variables
set(PROJECT_INCLUDE_DIR ${PROJECT_ROOT}/include)
set(PROJECT_PROTO_DIR ${PROJECT_ROOT}/proto)
set(PROJECT_SRC_DIR ${PROJECT_ROOT}/src)
set(PROJECT_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT}/bin)
set(PROJECT_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT}/lib)
set(PROJECT_PYTHON_DIR ${PROJECT_ROOT}/python)

# Forcing the generation dirs to be local
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})

set(RTAI on)

#if(BUILD_LOCAL)
#    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
#endif(BUILD_LOCAL)

###############################################################################
# Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  controller_interface
  control_toolbox
  realtime_tools
  m3meka_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf
)

if(catkin_FOUND)
#  catkin_python_setup()
else(catkin_FOUND)
endif(catkin_FOUND)

set(LIBRARIES ${LIBRARIES} ${catkin_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

###############################################################################
# Find RTAI
find_package(RTAI REQUIRED)
set(LIBRARIES ${LIBRARIES} ${RTAI_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${RTAI_INCLUDE_DIR})
add_definitions(-D__RTAI__)

###############################################################################
# Find M3
find_package(M3 COMPONENTS HARDWARE CHAINS TOOLBOX VEHICLES REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3_INCLUDE_DIR})

###############################################################################
# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})

###############################################################################
# Find google protobuff
find_package(Protobuf REQUIRED)
set(PROTOBUF_IMPORT_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})
file(GLOB ProtoFiles "${PROJECT_PROTO_DIR}/*.proto")

PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles})
set_source_files_properties(${ProtoSources} ${ProtoHeaders} PROPERTIES
                            GENERATED TRUE)
add_custom_target(${PROJECT_NAME}_cpp_protobuf_output ALL DEPENDS ${ProtoSources}) # Force the re-generation c++

PROTOBUF_GENERATE_PYTHON(PySources ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles}) 
add_custom_target(${PROJECT_NAME}_python_protobuf_output ALL DEPENDS ${PySources}) # Force the re-generation python

set(LIBRARIES ${LIBRARIES} ${PROTOBUF_LIBRARIES} ${ProtoHeaders})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})
###############################################################################
# Find Eigen
find_package(Eigen3 3.2 REQUIRED)
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Find boost packages
find_package(Boost 1.34 COMPONENTS filesystem system serialization REQUIRED)
set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

###############################################################################
# Find Yamlcpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
if(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
find_package(YamlCpp  REQUIRED)
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS YAMLCPP_05
   )
endif(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
link_directories(${YAMLCPP_LIBRARY_DIRS})

###############################################################################
# Find OROCOS KDL
find_package(orocos_kdl REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
set(LIBRARIES ${LIBRARIES} ${orocos_kdl_LIBRARIES})

###############################################################################
# other
set(LIBRARIES ${LIBRARIES} omni_kinematics pthread)

###############################################################################
# Include directories
include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/src      # For Protobuf-generated files.
    ${PROJECT_ROOT}/include
    ${PROJECT_ROOT}/omni_kinematics/include
    ${INCLUDE_DIRS}
    ${ProtoHeaders}
    ${M3_SRC} # HACK : Include this 4 directories for the generated protobufs headers from m3core
    ${M3RT_SRC}
    ${CMAKE_BINARY_DIR}/m3core/src
    ${CMAKE_BINARY_DIR}/m3core/src/m3rt/base 
)

set(ALL_SRCS 
    src/caster_control.cpp
    src/meka_omnibase_control.cpp
    src/factory_proxy.cpp
)
set(ALL_HDRS
    include/meka_omnibase_control/caster_control.hpp
    include/meka_omnibase_control/meka_omnibase_control.hpp
)

list(APPEND ALL_SRCS 
    ${ProtoSources}
)

list(APPEND ALL_HDRS
    ${ProtoHeaders}
    ${ProtoFiles}
)

###############################################################################
## Building
set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# This enables writing the state of the base directly to the server output:
add_definitions(-DDEBUG_OUTPUT=0)

add_subdirectory(omni_kinematics)
add_subdirectory(python)

add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES 
                      INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

## Installing
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${ALL_HDRS} DESTINATION include/m3/${PROJECT_NAME})

execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.PREFIXES[-1])" 
   OUTPUT_VARIABLE PYTHON_PREFIX 
) 
file ( TO_CMAKE_PATH "${PYTHON_PREFIX}" PYTHON_PREFIX ) 
execute_process ( 
   COMMAND ${PYTHON_EXECUTABLE} -c 
   	"import site, sys; sys.stdout.write(site.getsitepackages()[-1])" 
   OUTPUT_VARIABLE PYTHON_SITE_DIR 
) 
file ( TO_CMAKE_PATH "${PYTHON_SITE_DIR}" PYTHON_SITE_DIR ) 
string ( REGEX REPLACE "^${PYTHON_PREFIX}/" "" 
   PYTHON_SITE_DIR "${PYTHON_SITE_DIR}" 
) 

install ( FILES ${PySources} 
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library 
) 
