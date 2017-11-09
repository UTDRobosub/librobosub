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
# Create shared library
add_library(${LIBRARY_NAME} SHARED ${SOURCES} ${HEADERS})
# Install library
install(TARGETS ${LIBRARY_NAME} DESTINATION lib/${PROJECT_NAME})
# Install library headers
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
# Link OpenCV
target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS})
#Enable C++11
target_compile_features(${LIBRARY_NAME} PRIVATE cxx_range_for)

#Tests
add_executable(test-networkudp test/network/networkudptest.cpp)
target_link_libraries(test-networkudp ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-networkudp PRIVATE cxx_range_for)

add_executable(test-video test/video/videotest.cpp)
target_link_libraries(test-video ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-video PRIVATE cxx_range_for)

add_executable(test-stereo test/stereo/stereotest.cpp)
target_link_libraries(test-stereo ${OpenCV_LIBS} ${LIBRARY_NAME})
target_compile_features(test-stereo PRIVATE cxx_range_for)

#add_executable(test-videostab test/videostab/videostab.cpp)
#target_link_libraries(test-videostab ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
#target_compile_features(test-videostab PRIVATE cxx_range_for)

#Add special windows DLLS
#if(MSVC)
#add_custom_command(TARGET project POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory
#"${PROJECT_SOURCE_DIR}/lib"
#        $<TARGET_FILE_DIR:project>)
#endif()
