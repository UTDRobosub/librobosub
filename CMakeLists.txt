#Hey there! You need to install OpenCV first!

cmake_minimum_required(VERSION 3.2)
project(librobosub CXX)
find_package(OpenCV REQUIRED)
find_package (Threads)
set (CMAKE_CXX_STANDARD 11)

# Find files
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/**/*.h)
# Include header files
include_directories(include)
# Create shared library
add_library(librobosub SHARED ${SOURCES} ${HEADERS})
# Install library
install(TARGETS librobosub DESTINATION lib/${PROJECT_NAME})
# Install library headers
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
# Link OpenCV
target_link_libraries(librobosub ${OpenCV_LIBS})

#Tests
add_executable(test-video test/video/videotest.cpp)
target_link_libraries(test-video ${OpenCV_LIBS} librobosub)
target_compile_features(test-video PRIVATE cxx_range_for)

add_executable(test-stereo test/stereo/stereotest.cpp)
target_link_libraries(test-stereo ${OpenCV_LIBS} librobosub)
target_compile_features(test-stereo PRIVATE cxx_range_for)

#add_executable(test-videostab test/videostab/videostab.cpp)
#target_link_libraries(test-videostab ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
#target_compile_features(test-videostab PRIVATE cxx_range_for)

#Enable C++11
target_compile_features(librobosub PRIVATE cxx_range_for)

#Add special windows DLLS
#if(MSVC)
#add_custom_command(TARGET project POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory
#"${PROJECT_SOURCE_DIR}/lib"
#        $<TARGET_FILE_DIR:project>)
#endif()
