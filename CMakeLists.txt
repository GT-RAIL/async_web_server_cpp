cmake_minimum_required(VERSION 3.5)
project(async_web_server_cpp)

find_package(ament_cmake_ros REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system regex filesystem)
find_package(OpenSSL)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

###################################################
## Declare things to be passed to other projects ##
###################################################

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/http_server.cpp
  src/http_connection.cpp src/http_request_parser.cpp
  src/http_reply.cpp src/http_request_handler.cpp
  src/websocket_connection.cpp src/websocket_request_handler.cpp
  src/websocket_message.cpp src/http_request.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OPENSSL_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_export_dependencies(OpenSSL)
ament_export_include_directories(include ${Boost_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

ament_package()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)


if(BUILD_TESTING)
  add_subdirectory(test)
endif()
