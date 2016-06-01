cmake_minimum_required (VERSION 2.8.3)
project (ddsnode)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_VERBOSE_MAKEFILE ON)

# Having trouble building both ros2 and dds? Make sure your envs are right (keep the order of sources):
# Source ros2 setup.sh (i.e. ~/ros2_ws/install/setup.sh
# Source release.com from your OpenSplice (i.e. ~/HDE/x86_64.linux/release.com)
# Source envs-*.sh from your OpenSplice (i.e. HDE/x86_64.linux/src/OpenSpliceDDS/envs-x86_64.linux-dev.sh)

include (MacroOpenSplice)
include (osmacros)

find_package (OpenSplice REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${OpenSplice_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
)

set (idls RobotMessages.idl)

file(COPY ${idls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

#TODO - to jest kosztowne i nie sprawdza, czy trzeba
foreach(idl ${idls})
	OpenSplice_IDLGEN (${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1.cpp" VARS_1 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1.h" VARS_2 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1Dcps.cpp" VARS_3 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1Dcps.h" VARS_4 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1Dcps_impl.cpp" VARS_5 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1Dcps_impl.h" VARS_6 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1SplDcps.cpp" VARS_7 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/\\1SplDcps.h" VARS_8 ${idl})
	string (REGEX REPLACE "\(.*\).idl" "./gen/ccpp_\\1.h" VARS_9 ${idl})
        set(OpenSplice_DATAMODEL ${OpenSplice_DATAMODEL} ${VARS_1} ${VARS_2} ${VARS_3} ${VARS_4} ${VARS_5} ${VARS_6} ${VARS_7} ${VARS_8} ${VARS_9})
endforeach(idl)

file(COPY ${idls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(SOURCES
    ddspublisher.cpp
    ddsnode.cpp
    ddssubscriber.cpp
    ddstopics.cpp
    ddsnodefactory.cpp
    ddsqos.cpp)

set(HEADERS
    ddsinclude.h
    ddsnodefactory.h
    ddsnode.h)

add_definitions(${OpenSplice_DEFINITIONS} ${DEFINITIONS})
add_library(ddsnode SHARED ${SOURCES} ${HEADERS} ${OpenSplice_DATAMODEL})
target_link_libraries(ddsnode common Qt5::Core ${OpenSplice_LIBRARIES})
INSTALL(TARGETS ddsnode DESTINATION bin)
INSTALL(FILES ${HEADERS} ${OpenSplice_DATAMODEL} DESTINATION /include/ddsnode)

