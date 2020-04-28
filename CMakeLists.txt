cmake_minimum_required(VERSION 3.5)

# set the project name
project(Project1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Bring the headers, such as Student.h into the project
include_directories(include/common)
include_directories(include/linux)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-sign-compare -DHAVE_CONFIG_H")


#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")

add_executable(project1 ${SOURCES})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(PNG REQUIRED)
include_directories( include ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS} )

target_link_libraries(project1 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${PNG_LIBRARY})