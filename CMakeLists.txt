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
find_package(OpenCV 4.0.1 REQUIRED)
find_package (Threads)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
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
file(GLOB WS_HEADERS include/**/ws/*.h include/**/ws/*.hpp)
file(GLOB JSON_HEADERS include/**/json/*.h include/**/json/*.hpp)
# Include header files
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
# Create shared library
add_library(${LIBRARY_NAME} SHARED ${SOURCES} ${HEADERS} ${WS_HEADERS} ${JSON_HEADERS})
target_link_libraries(${LIBRARY_NAME} PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES} pthread)
# Install library
install(TARGETS ${LIBRARY_NAME} DESTINATION lib/${PROJECT_NAME})
# Install library headers
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
install(FILES ${WS_HEADERS} DESTINATION include/${PROJECT_NAME}/ws)
install(FILES ${JSON_HEADERS} DESTINATION include/${PROJECT_NAME}/json)
# Install so find_package works
#install(FILES ${LIBRARY_NAME}-config.cmake)
# Enable C++11
target_compile_features(${LIBRARY_NAME} PUBLIC cxx_range_for)

# Export
#set(ROBOSUB_INCLUDE_DIRS ${HEADERS})
install(TARGETS ${LIBRARY_NAME} DESTINATION lib EXPORT ${LIBRARY_NAME}-config)
install(EXPORT ${LIBRARY_NAME}-config DESTINATION lib/${LIBRARY_NAME})

# Tests
add_executable(test-networktcp test/networktcp/networktcptest.cpp)
target_link_libraries(test-networktcp ${LIBRARY_NAME})
target_compile_features(test-networktcp PRIVATE cxx_range_for)

add_executable(test-networkudp test/networkudp/networkudptest.cpp)
target_link_libraries(test-networkudp ${LIBRARY_NAME})
target_compile_features(test-networkudp PRIVATE cxx_range_for)

add_executable(test-networkvideo test/networkvideo/networkvideotest.cpp)
target_link_libraries(test-networkvideo ${LIBRARY_NAME})
target_compile_features(test-networkvideo PRIVATE cxx_range_for)

add_executable(test-networkvideotcp test/networkvideotcp/networkvideotcptest.cpp)
target_link_libraries(test-networkvideotcp ${LIBRARY_NAME})
target_compile_features(test-networkvideotcp PRIVATE cxx_range_for)

add_executable(test-video test/video/videotest.cpp)
target_link_libraries(test-video ${LIBRARY_NAME})
target_compile_features(test-video PRIVATE cxx_range_for)

add_executable(test-stereo test/stereo/stereotest.cpp)
target_link_libraries(test-stereo ${LIBRARY_NAME})
target_compile_features(test-stereo PRIVATE cxx_range_for)

add_executable(test-serial test/serial/serialtest.cpp)
target_link_libraries(test-serial ${LIBRARY_NAME})
target_compile_features(test-serial PRIVATE cxx_range_for)

add_executable(test-drivetrain test/drivetrain/drivetrain.cpp)
target_link_libraries(test-drivetrain ${LIBRARY_NAME})
target_compile_features(test-drivetrain PRIVATE cxx_range_for)

#add_executable(test-httpservice test/httpservice/httpservicetest.cpp)
#target_link_libraries(test-httpservice ${LIBRARY_NAME})
#target_include_directories(test-httpservice PRIVATE ${Boost_INCLUDE_DIR})
#target_compile_features(test-httpservice PRIVATE cxx_range_for)

add_executable(test-telemetry test/telemetry/telemetrytest.cpp)
target_link_libraries(test-telemetry ${LIBRARY_NAME})
target_include_directories(test-telemetry PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(test-telemetry PRIVATE cxx_range_for)

add_executable(test-cuda test/cuda/cudatest.cpp)
target_link_libraries(test-cuda ${LIBRARY_NAME})
target_compile_features(test-cuda PRIVATE cxx_range_for)

add_executable(test-stereocalib test/stereocalib/stereocalib.cpp)
target_link_libraries(test-stereocalib ${LIBRARY_NAME})
target_include_directories(test-stereocalib PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(test-stereocalib PRIVATE cxx_range_for)

#add_executable(test-telemetryserial test/telemetryserial/telemetryserialtest.cpp)
#target_link_libraries(test-telemetryserial ${LIBRARY_NAME})
#target_include_directories(test-telemetryserial PRIVATE ${Boost_INCLUDE_DIR})
#target_compile_features(test-telemetryserial PRIVATE cxx_range_for)

file(GLOB CAMERACALIB_SOURCES test/interactive-calibration/*.cpp)
add_executable(app-cameracalib ${CAMERACALIB_SOURCES})
target_link_libraries(app-cameracalib ${LIBRARY_NAME})
target_include_directories(app-cameracalib PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(app-cameracalib PRIVATE cxx_range_for)

add_executable(app-cameracalib2 test/interactive-calibrationv2/calib2.cpp)
target_link_libraries(app-cameracalib2 ${LIBRARY_NAME})
target_include_directories(app-cameracalib2 PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(app-cameracalib2 PRIVATE cxx_range_for)

add_executable(app-stereocalib test/interactive-stereocalib/app-stereocalib.cpp)
target_link_libraries(app-stereocalib ${LIBRARY_NAME})
target_include_directories(app-stereocalib PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(app-stereocalib PRIVATE cxx_range_for)

add_executable(test-stereorect test/stereorect/stereorect.cpp)
target_link_libraries(test-stereorect ${LIBRARY_NAME})
target_include_directories(test-stereorect PRIVATE ${Boost_INCLUDE_DIR})
target_compile_features(test-stereorect PRIVATE cxx_range_for)