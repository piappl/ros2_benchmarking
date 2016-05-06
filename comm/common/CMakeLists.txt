cmake_minimum_required(VERSION 2.8.3)
project(common)

set(SOURCES
    logging.c
    communicationutils.cpp)

set(HEADERS
    communicationutils.h
    logging.h
    rosnodeinterface.h)

set(INSTALL_HEADERS
    logging.h
    communicationutils.h
    messagetypes.h
    rostopics.h
    rosnodeinterface.h)

add_library(common SHARED ${SOURCES} ${HEADERS})
target_link_libraries(common Qt5::Core)
INSTALL(FILES ${INSTALL_HEADERS} DESTINATION /include/common)
INSTALL_TARGETS(/lib RUNTIME_DIRECTORY /lib common)

