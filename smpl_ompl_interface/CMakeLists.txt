cmake_minimum_required(VERSION 2.8.12)
project(smpl_ompl_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(smpl REQUIRED)

####################################################
# allow overriding install directories for install #
####################################################

set(SMPL_OMPL_INTERFACE_INSTALL_LIB_DIR     lib                             CACHE PATH "Installation directory for libraries")
set(SMPL_OMPL_INTERFACE_INSTALL_INCLUDE_DIR include                         CACHE PATH "Installation directory for header files")
set(SMPL_OMPL_INTERFACE_INSTALL_CMAKE_DIR   lib/cmake/smpl_ompl_interface   CACHE PATH "Installation directory for CMake files")
set(SMPL_OMPL_INTERFACE_INSTALL_BIN_DIR     bin                             CACHE PATH "Installation directory for executables")
foreach(path LIB INCLUDE CMAKE BIN)
    set(var SMPL_INSTALL_${path}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

catkin_package(
    INCLUDE_DIRS include /usr/local/include/
    LIBRARIES smpl_ompl_interface /usr/local/lib/libompl.so)

add_library(smpl_ompl_interface src/ompl_interface.cpp)
target_compile_options(smpl_ompl_interface PUBLIC -std=c++11)
#target_compile_definitions(smpl_ompl_interface PRIVATE -DSMPL_CONSOLE_ROS)
target_include_directories(smpl_ompl_interface PUBLIC include)
target_include_directories(smpl_ompl_interface SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(smpl_ompl_interface SYSTEM PUBLIC /usr/local/include/)
target_include_directories(smpl_ompl_interface SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(smpl_ompl_interface PRIVATE smpl::smpl)
target_link_libraries(smpl_ompl_interface PUBLIC /usr/local/lib/libompl.so)
target_link_libraries(smpl_ompl_interface PRIVATE ${Boost_LIBRARIES})

add_executable(smpl_ompl_test src/smpl_ompl.cpp)
target_compile_options(smpl_ompl_test PRIVATE -std=c++11)
target_include_directories(smpl_ompl_test SYSTEM PRIVATE /usr/local/include/)
target_include_directories(smpl_ompl_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(smpl_ompl_test PRIVATE smpl_ompl_interface)
target_link_libraries(smpl_ompl_test PRIVATE /usr/local/lib/libompl.so)
target_link_libraries(smpl_ompl_test PRIVATE ${Boost_LIBRARIES})
target_link_libraries(smpl_ompl_test PRIVATE smpl::smpl)

################
# Installation #
################

set(SMPL_INCLUDE_DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_INCLUDE_DIR}/smpl_ompl_interface)
set(SMPL_ARCHIVE_DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_LIB_DIR})
set(SMPL_LIBRARY_DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_LIB_DIR})
set(SMPL_RUNTIME_DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_BIN_DIR})

install(
    DIRECTORY include/smpl_ompl_interface/
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    COMPONENT dev
    FILES_MATCHING PATTERN *.h PATTERN *.hpp)
# install(
#     FILES ${CMAKE_CURRENT_BINARY_DIR}/include/smpl/config.h
#     DESTINATION ${SMPL_INCLUDE_DESTINATION}
#     COMPONENT dev)

# if(SMPL_BUILD_SHARED)
#     list(APPEND BUILT_LIBRARIES smpl-shared)
# endif()

install(
    TARGETS smpl_ompl_interface
    RUNTIME DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_LIB_DIR})
install(
    DIRECTORY include/smpl_ompl_interface/
    DESTINATION ${SMPL_OMPL_INTERFACE_INSTALL_BIN_DIR})
