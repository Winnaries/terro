cmake_minimum_required(VERSION 3.10)

project(terro)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DCOMPILEDWITHC17)

set(SYSTEM_INCLUDE_DIRS "/usr/local/include")
set(ORB_SLAM3_INCLUDE_DIRS "/Users/winnaries/senior/odometry/ORB_SLAM3")

find_package(OpenCV 4.4)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 4.4 not found.")
    endif()

message("OpenCV VERSION: " ${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
message("Eigen3 VERSION: " ${Eigen3_VERSION})

find_package(Pangolin REQUIRED)
message("Pangolin VERSION: " ${Pangolin_VERSION})

message("")

add_executable(terro src/main.cc)
add_executable(terro_calibrator src/calibrator.cc)

target_compile_features(terro PUBLIC cxx_std_17)
target_compile_features(terro_calibrator PUBLIC cxx_std_17)

include_directories(
    ${SYSTEM_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(terro ORB_SLAM3 ctello -L/usr/local/lib)
target_link_libraries(terro_calibrator 
${OpenCV_LIBS} 
-lfmt ctello 
-L/usr/local/opt/opencv/lib 
-L/usr/local/lib)

install(TARGETS terro DESTINATION /usr/local/bin)
install(TARGETS terro_calibrator DESTINATION /usr/local/bin)