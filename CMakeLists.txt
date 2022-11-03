cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
project(rb5_camera_manager)
add_definitions(-std=c++11) 


# find_package(Boost REQUIRED COMPONENTS thread)
find_package( catkin REQUIRED COMPONENTS

  image_transport
  nodelet
  sensor_msgs
  roscpp
  cv_bridge
  message_generation
  genmsg
)

# for ROSImage
#generate_messages(DEPENDENCIES sensor_msgs)
#catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)
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

)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(camera_hal3 ${CMAKE_CURRENT_SOURCE_DIR}/CameraHAL3Main.cpp 


                )

#image_transport
add_dependencies(camera_hal3 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(
  camera_hal3 # target
  pthread
  ${CMAKE_CURRENT_SOURCE_DIR}/camera_hal3_lib.a #makefile build
  -ldl -lcamera_metadata -lcutils -lutils -llog #lib

 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

target_include_directories(camera_hal3 PRIVATE

${CMAKE_CURRENT_SOURCE_DIR}

${INCLUDE}
)


#target_link_libraries(camera_hal3  PRIVATE pthread   )


add_executable(stream_hal3 ${CMAKE_CURRENT_SOURCE_DIR}/hal3_continous.cpp )

target_link_libraries( stream_hal3
pthread
${CMAKE_CURRENT_SOURCE_DIR}/camera_hal3_lib.a
-ldl -lcamera_metadata -lcutils -lutils -llog
 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

target_include_directories(stream_hal3 PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}
${INCLUDE}
)
