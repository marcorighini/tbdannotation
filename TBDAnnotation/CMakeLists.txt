#
# tbdannotation
#

#
# Building in-tree is not allowed (we take care of your craziness).
#

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "Prevented in-tree built. Please create a build directory outside of the source code and call cmake from there. Thank you.")
endif()

#
# Project configuration
#

project(tbdannotation)
cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT LIBRARY_TYPE)
  set(LIBRARY_TYPE "Shared")
endif(NOT LIBRARY_TYPE)

set(PROJECT_NAME "tbdannotation")
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
set(SETTINGS_ORGANIZATION "Righini Marco, Galligari Alessandro")
set(SETTINGS_APPLICATION ${PROJECT_NAME})
set(PROJECT_AUTHOR "Righini Marco, Galligari Alessandro")
set(PROJECT_YEAR_COPYRIGHT "2013")
set(PROJECT_AUTHOR_EMAIL "righini.marco88@gmail.com, galligari.alessandro@gmail.com")

set(PROJECT_BUILD_MESSAGE ${PROJECT_NAME} " v" ${PROJECT_VERSION} " (" ${LIBRARY_TYPE} "." ${CMAKE_BUILD_TYPE} ")")
set(COPYRIGHT_BUILD_MESSAGE "Copyright (c) " ${PROJECT_YEAR_COPYRIGHT} " " ${PROJECT_AUTHOR}  " <" ${PROJECT_AUTHOR_EMAIL} ">")

set(DEVELOPERS_BUILD_MESSAGE "Developed by")
set(DEV_RIGHINI_MARCO "Righini Marco <righini.marco88@gmail.com>")
set(DEV_GALLIGARI_ALESSANDRO "Galligari Alessandro <galligari.alessandro@gmail.com>")

message("*")
message("* " ${PROJECT_BUILD_MESSAGE})
message("* " ${COPYRIGHT_BUILD_MESSAGE})
message("*")
message("* " ${DEVELOPERS_BUILD_MESSAGE})
message("*     " ${DEV_RIGHINI_MARCO})
message("*     " ${DEV_GALLIGARI_ALESSANDRO})
message("*")

#
# Compile stuff
#

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os -DRELEASE")

add_definitions(
  -DPROJECT_NAME=${PROJECT_NAME}
  -DPROJECT_VERSION="${PROJECT_VERSION}"
)

#
# CMake configuration
#

set(PROJECT_CMAKE_MODULES cmake/modules)
set(PROJECT_INSTALL_DIR install)
set(PROJECT_BUILD_DIR build)
set(PROJECT_SRC_DIR src)
set(PROJECT_DEPS_DIR ../deps)

set(PROJECT_RUNTIME_OUTPUT_DIRECTORY bin)
set(PROJECT_LIBRARY_OUTPUT_DIRECTORY lib)
set(PROJECT_ARCHIVE_OUTPUT_DIRECTORY lib/static)
set(PROJECT_INCLUDE_OUTPUT_DIRECTORY include)
set(PROJECT_SHARE_OUTPUT_DIRECTORY share)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_ARCHIVE_OUTPUT_DIRECTORY})

set(
    CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_SOURCE_DIR}/${PROJECT_CMAKE_MODULES}
)

#
# Referenced packages
#

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_DEPS_DIR}/opencv/build/)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(BOOST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_DEPS_DIR}/boost/build/)
find_package(Boost REQUIRED COMPONENTS serialization filesystem thread program_options)
include_directories(${Boost_INCLUDE_DIR})

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})

find_package(SQLite3 REQUIRED)
include_directories(${SQLITE3_INCLUDE_DIR})

#
# Referenced directories and targets
#

set(TARGET_TBDANNOTATION tbdannotation)

add_subdirectory(${PROJECT_SRC_DIR})


#
# I use QtCreator and I need the lines below, so do not ask. :-)
#

file(
GLOB_RECURSE PROJECT_FILES FOLLOW_SYMLINKS
*.txt *.mk *.c *.cpp *.hpp *.h *.java *.in *.cmake *.m *.mm
*.djinni *.idl *.dox *.sh Makefile *.example *.config *.build
*.xml *.py *.scala README *.sql
)

add_custom_target(
QTCREATOR_FALLBACK ALL
COMMENT "Feel free to ignore this target, but please do not remove it."
SOURCES ${PROJECT_FILES}
)
