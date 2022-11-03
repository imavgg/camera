# camera
Build 
1. mkdir -r catkin_ws/src
2. cd catkin_ws/src
3. git pull ***
4. cd catkin_ws && catkin_make


Run
cd catkin_ws
[camera id] [yuv420_8bit_previwe_weight] [yuv420_8bit_height] [jpeg_weight] [jpeg_height] 
./build/stream_hal3 0 1920 1080 1920 1080

Function
* Save image flie (Streamming) to folder (build/CAPTURE/***.yuv or ***.jpg)

* Run with ROS command (rosrun) --> not working

* OPENCV read image file (build/CAPTURE/***.yuv or ***.jpg) to ROS
  * Read JPG with opencv read with encode format bgr8 is OK
  * but YUV with opencv read with encode format ---> still not working
  
* OPENCV read buffer(from memory to ROS) 
  * although I have added code and build success, but have not tested the result yet.
  
