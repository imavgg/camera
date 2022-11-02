/usr/bin/g++ -o camera_hal3_lib.a  CameraHAL3Config.o CameraHAL3Buffer.o CameraHAL3Snapshot.o CameraHAL3Device.o -pthread -ldl -lcamera_metadata -lcutils -lutils -llog

ar rcs camera_hal3_lib.a CameraHAL3Config.o CameraHAL3Buffer.o CameraHAL3Snapshot.o CameraHAL3Device.o
