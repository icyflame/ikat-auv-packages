cmake_minimum_required(VERSION 2.4.6)
project(ImageThresholdInterface)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#find_package( KDE4 REQUIRED )
#include_directories( ${KDE4_INCLUDES} )
Find_package(OpenCV REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
include_directories(include/${PROJECT_NAME} ${CMAKE_CURRENT_BINARY_DIR})
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
#ADD_DEFINITIONS(${QT_DEFINITIONS})
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/mainwindow.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/main.cpp src/mainwindow.cpp)
add_executable(testing ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
TARGET_LINK_LIBRARIES( testing ${QT_LIBRARIES} ${OpenCV_LIBS})
#${PROJECT_NAME}
