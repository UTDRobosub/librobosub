#Hey there! You need to install OpenCV first!

cmake_minimum_required(VERSION 3.2)

#Set library name
IF (WIN32)
  # set stuff for windows
  set(LIBRARY_NAME librobosub)
  link_libraries(wsock32 ws2_32)
ELSE()
  set(LIBRARY_NAME robosub)
ENDIF()

project(librobosub CXX)
find_package(OpenCV REQUIRED)
find_package (Threads)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost 1.54.0 COMPONENTS system thread coroutine context regex REQUIRED)
find_package(OpenSSL REQUIRED)
set (CMAKE_CXX_STANDARD 11)

IF (UNIX)
  # include X11
  find_package(X11 REQUIRED)
  link_libraries(${X11_LIBRARIES})
  include_directories(${X11_INCLUDE_DIR})
ENDIF()

# Find files
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/**/*.h)
# Include header files
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
# Create shared library
add_library(${LIBRARY_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${LIBRARY_NAME} ${Boost_LIBRARIES})
# Install library
install(TARGETS ${LIBRARY_NAME} DESTINATION lib/${PROJECT_NAME})
# Install library headers
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
# Install so find_package works
#install(FILES ${LIBRARY_NAME}-config.cmake)
# Enable C++11
target_compile_features(${LIBRARY_NAME} PRIVATE cxx_range_for)

# Export
#set(ROBOSUB_INCLUDE_DIRS ${HEADERS})
install(TARGETS ${LIBRARY_NAME} DESTINATION lib EXPORT ${LIBRARY_NAME}-config)
install(EXPORT ${LIBRARY_NAME}-config DESTINATION lib/${LIBRARY_NAME})

# Tests
add_executable(test-networkudp test/networkudp/networkudptest.cpp)
target_link_libraries(test-networkudp ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-networkudp PRIVATE cxx_range_for)

add_executable(test-networkvideo test/networkvideo/networkvideotest.cpp)
target_link_libraries(test-networkvideo ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-networkvideo PRIVATE cxx_range_for)

add_executable(test-video test/video/videotest.cpp)
target_link_libraries(test-video ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-video PRIVATE cxx_range_for)

add_executable(test-stereo test/stereo/stereotest.cpp)
target_link_libraries(test-stereo ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-stereo PRIVATE cxx_range_for)

add_executable(test-httpservice test/httpservice/httpservicetest.cpp)
target_link_libraries(test-httpservice ${OpenCV_LIBS} ${LIBRARY_NAME} ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES} pthread)
target_include_directories(test-httpservice PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(test-httpservice PRIVATE cxx_range_for)
