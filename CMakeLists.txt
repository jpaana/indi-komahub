cmake_minimum_required(VERSION 3.0)
PROJECT(komahub CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set (KOMAHUB_VERSION_MAJOR 1)
set (KOMAHUB_VERSION_MINOR 0)

find_package(INDI REQUIRED)
find_package(USB1 REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_komahub.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_komahub.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${USB1_INCLUDE_DIRS})

include(CMakeCommon)

############# KOMAHUB ###############
set(komahub_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/komahub.cpp
)

add_executable(indi_komahub ${komahub_SRCS})

target_link_libraries(indi_komahub ${INDI_LIBRARIES} ${USB1_LIBRARIES})

install(TARGETS indi_komahub RUNTIME DESTINATION bin)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_komahub.xml DESTINATION ${INDI_DATA_DIR})
