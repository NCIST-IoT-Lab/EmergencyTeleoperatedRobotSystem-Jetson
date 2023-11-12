# Enable C++11
cmake_minimum_required(VERSION 3.20)

SET(CMAKE_CXX_STANDARD 17) #Open3D需要C++14以上版本
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
SET(CMAKE_EXE_LINKER_FLAGS "-lstdc++")

#### AcquiringPointCloud  ####
# Define project name 
project(CasPointCloud)
SET(this_target AcquiringPointCloud)

# Kinect DK相机
find_package(k4a REQUIRED)

# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# include_directories(/usr/local/include/btferret)

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# OPEN3D
find_package(Open3D REQUIRED)
include_directories("/usr/local/include/open3d")
include_directories("/usr/include/python3.8")
include_directories("/usr/include/python2.7")

# JSON
find_package(jsoncpp REQUIRED)

# Boost
# set(BOOST_ROOT, /usr/local/lib/libboost_python38)
find_package(Boost 1.83 REQUIRED COMPONENTS python38)
include_directories(${Boost_INCLUDE_DIRS})

MESSAGE( STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}.")
MESSAGE( STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}.")
MESSAGE( STATUS "Boost_LIB_VERSION = ${Boost_LIB_VERSION}.")

set(PYTHON_DOT_VERSION 3.8)
set(PYTHON_INCLUDE /usr/include/python3.8)
set(PYTHON_LIBRARY /usr/lib/python3.8/config-aarch64-linux-gnu)

SET(EXECUTABLE_OUTPUT_PATH ../../../ETRS/det3d_bin)
SET(BOOST_DIR /usr/lib/x86_64-linux-gnu/)

file(GLOB_RECURSE UTILITY src/utility/*.cpp)

# 指定头文件目录
include_directories(include)
include_directories(include/utility)
include_directories(include/python_invoker)

add_executable(EmergencyTeleoperatedRobotSystem
        src/EmergencyTeleoperatedRobotSystem.cpp 
        src/AzureKinectExtrinsics.cpp 
        src/SoundSourceLocalization.cpp
        src/AzureKinect.cpp 
        src/PointCloud.cpp
        src/Bot.cpp
        src/Network.cpp
        src/Bluetooth.cpp
        src/Device.cpp
        src/Serial.cpp
        ${UTILITY}
        ${PROTO_SRCS} 
        ${PROTO_HDRS} 
        )

add_executable(Boostt
        src/Boostt.cpp 
        src/python_invoker/PythonInvoker.cpp
        src/python_invoker/TypeConverters.cpp
        ${UTILITY}
        )

target_link_libraries(Boostt
        k4a::k4a
        ${Open3D_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${Protobuf_LIBRARIES}
        ${Boost_LIBRARIES}
        asound
        fftw3
        bluetooth
        -lpython3.8
        -lpython2.7
        )

# add_executable(Reconstruction
#         src/Reconstruction.cpp 
#         src/AzureKinectExtrinsics.cpp 
#         src/AzureKinect.cpp 
#         # src/utility/Utility.cpp
#         src/Bot.cpp
#         src/Network.cpp
#         src/Bluetooth.cpp
#         src/Device.cpp
#         src/Serial.cpp
#         ${UTILITY}
#         ${PROTO_SRCS} 
#         ${PROTO_HDRS} 
#         )

# target_link_libraries(Reconstruction
#         k4a::k4a
#         ${Open3D_LIBRARIES}
#         ${Eigen3_LIBRARIES}
#         ${Protobuf_LIBRARIES}
#         asound
#         bluetooth
#         )


# target_link_libraries(Compete
#         k4a::k4a
#         ${Open3D_LIBRARIES}
#         ${Eigen3_LIBRARIES}
#         ${Protobuf_LIBRARIES}
#         asound
#         fftw3
#         )

# add_executable(Rotate
#         src/Rotate.cpp 
#         src/Utility.cpp
#         src/Bot.cpp
#         )

# target_link_libraries(Rotate
#         ${Open3D_LIBRARIES}
#         ${Eigen3_LIBRARIES}
#         asound
#         )



# add_executable(CasControlCar
#         src/CasControlCar.cpp
#         src/CasBot.cpp
#         src/CasUtility.cpp
#         )

target_link_libraries(EmergencyTeleoperatedRobotSystem
        k4a::k4a
        ${Open3D_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${Protobuf_LIBRARIES}
        asound
        fftw3
        bluetooth
        )

# target_link_libraries(CasControlCar
#         ncurses
#         )
        


