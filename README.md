# Camera
## Resource
* Camera: https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform/tree/master/camera-hal3-sample
* Image_transport: http://wiki.ros.org/image_transport/Tutorials/PublishingImages

## Build 
1. mkdir -r catkin_ws/src
2. cd catkin_ws/src
3. git pull ***
4. cd catkin_ws && catkin_make


## Run
1. cd catkin_ws
* ( [camera id] [yuv420_8bit_previwe_weight] [yuv420_8bit_height] [jpeg_weight] [jpeg_height] )
3. ./build/stream_hal3 0 1920 1080 1920 1080

## Function
* Save image flie (Streamming) to folder (build/CAPTURE/***.yuv or ***.jpg)

* Run with ROS command (rosrun) --> not working

* OPENCV read image file (build/CAPTURE/***.yuv or ***.jpg) to ROS
  * Read JPG with opencv read with encode format "bgr8" is OK
  * but YUV with opencv read with encode format ---> still not working
  
* OPENCV read buffer(from memory to ROS) 
  * although I have added code and build success, but have not tested the result yet.
  
