cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
project(RB5_camera_manager)
add_definitions(-std=c++11) 


#find_package(Boost REQUIRED COMPONENTS thread)

SET(LINK_FLAGS "-pthread")

SET(LIBS_RB5 " -ldl -lcamera_metadata -lcutils -lutils -llog")

SET(INCLUDE_RB5 /usr/include/qrb5165

            /usr/src/qrb5165-linux-headers-4.19/usr/include

            .

            )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -o0 -ggdb")
# RB5 camera header files.

include_directories(

    .
    
  /usr/include/qrb5165

    /usr/src/qrb5165-linux-headers-4.19/usr/include

#    ${catkin_INCLUDE_DIRS} 


)

# add_library(
#   camera_hal3_main OBJECT
#   CameraHAL3Main.cpp 
# )
# target_include_directories(
#   camera_hal3_main PRIVATE
#   ${INCLUDE_RB5} ${LIBS_RB5}
# )

# add_library(
#   camera_hal3_buffer OBJECT
#   CameraHAL3Buffer.cpp
# )
# target_include_directories(
#   camera_hal3_buffer PRIVATE
#   ${INCLUDE_RB5} ${LIBS_RB5}
# )

# add_library(
#   camera_hal3_config OBJECT
#   CameraHAL3Config.cpp
# )

# target_include_directories(
#   camera_hal3_config PRIVATE
#   ${INCLUDE_RB5} ${LIBS_RB5}
# )

# add_library(
#   camera_hal3_snapshot OBJECT
#   CameraHAL3Snapshot.cpp
# )
# target_include_directories(
#   camera_hal3_snapshot PRIVATE
#   ${INCLUDE_RB5} ${LIBS_RB5}
#   #${CMAKE_CURRENT_SOURCE_DIR}
# )
# add_library(
#   camera_hal3_dev OBJECT
#   CameraHAL3Device.cpp
# )

# target_include_directories(
#   camera_hal3_dev PRIVATE
#   ${INCLUDE_RB5} ${LIBS_RB5}
# )



add_executable(camera_hal3 CameraHAL3Main.cpp 

                            # CameraHAL3Config.cpp 

                            # CameraHAL3Buffer.cpp 

                            # CameraHAL3Snapshot.cpp 

                            # CameraHAL3Device.cpp

                )

target_link_libraries(
  camera_hal3 # target
  pthread
  ${CMAKE_CURRENT_SOURCE_DIR}/camera_hal3_lib.a #makefile build
  -ldl -lcamera_metadata -lcutils -lutils -llog #lib
)

target_include_directories(camera_hal3 PRIVATE

${CMAKE_CURRENT_SOURCE_DIR}

${INCLUDE}
)


#target_link_libraries(camera_hal3  PRIVATE pthread   )


