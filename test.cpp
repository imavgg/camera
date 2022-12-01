/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>

#include "CameraHAL3Config.h"
#include "CameraHAL3Snapshot.h"

// stream to ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <time.h>
#include <camera_info_manager/camera_info_manager.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <thread>

#define CAPTURE_OUTPUT_DIR "CAPTURE"
using namespace std;

const char* CameraHALLibraryFile = "/usr/lib/hw/camera.qcom.so";
static camera_module_t*     cameraModule;
static bool savePreviewNow = false;
static std::mutex saveFrameMutex;
static std::condition_variable saveFrameCondition;

cv::Mat image;
cv::Mat image_proc; // after cv process

sensor_msgs::ImagePtr msg ;
// Callback from camera module
// Parameters:
//   const struct camera_module_callbacks* callbacks
//   int camera_id
//   int new_status
void camera_device_status_change(const struct camera_module_callbacks*,  int, int)
{}

/**
 * @brief
 *
 * @param callbacks
 * @param camera_id
 * @param new_status
 */
// Callback from camera module
// Parameters:
//   const struct camera_module_callbacks* callbacks
//   const char* camera_id
//   int new_status
void torch_mode_status_change(const struct camera_module_callbacks* , const char*, int)
{}


// Callback struct for camera module.
camera_module_callbacks_t camera_module_callbacks =
{
    camera_device_status_change,
    torch_mode_status_change
};


int initial_module()
{
    struct camera_info camera_info;
    vendor_tag_ops_t vendor_tag_ops;
    int result = 0;
    void *fd = NULL;

    cameraModule = NULL;

    fd = dlopen(CameraHALLibraryFile, RTLD_NOW);
    if (fd == NULL) {
        fprintf(stderr, "Load %s module failed.\n", CameraHALLibraryFile);
        result = -EINVAL;
        cameraModule = NULL;
        return result;
    }

    const char *hal_module_info_sym = HAL_MODULE_INFO_SYM_AS_STR;
    cameraModule = (camera_module_t *)dlsym(fd, hal_module_info_sym);
    if (cameraModule == NULL) {
        fprintf(stderr, "Load symbol %s failed.\n", hal_module_info_sym);
        result = -EINVAL;
        if (fd != NULL) {
            dlclose(fd);
            fd = NULL;
        }
        return result;
    }

    if (strcmp(CAMERA_HARDWARE_MODULE_ID, cameraModule->common.id) != 0) {
        fprintf(stderr, "Load id %s != camera_module_ptr->id=%s\n",
                CAMERA_HARDWARE_MODULE_ID, cameraModule->common.id);
        result = -EINVAL;
        if (fd != NULL) {
            dlclose(fd);
            fd = NULL;
        }
        return result;
    }

    cameraModule->common.dso = fd;
    // printf("Camera HAL library loaded.\n");

    printf("Initialize cameras...\n");
    result = cameraModule->init();
    if (result != 0) {
        fprintf(stderr, "Init camera failed. ret=%d\n", result);
        return result;
    }else{  
        fprintf(stderr, "Init camera success. ret=%d\n", result);
        return result;

    }

    if (cameraModule->get_vendor_tag_ops) {
        vendor_tag_ops = vendor_tag_ops_t();
        cameraModule->get_vendor_tag_ops(&vendor_tag_ops);

        sp<VendorTagDescriptor> vendor_tag_descriptor;
        result = VendorTagDescriptor::createDescriptorFromOps(&vendor_tag_ops, vendor_tag_descriptor);

        if (result != 0) {
            fprintf(stderr, "Generate descriptor from vendor tag operations failed. ret=%d\n", result);
            return result;
        }

        result = VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_descriptor);

        if (result != 0) {
            fprintf(stderr, "Set vendor tag descriptor failed. ret=%d\n", result);
            return result;
        }
    }

    result = cameraModule->set_callbacks(&camera_module_callbacks);
    if (result != 0) {
        fprintf(stderr, "set_callbacks failed. Error=%d\n", result);
        return result;
    }

    printf("Cameras list:\n");
    int num_cameras = cameraModule->get_number_of_cameras();
    for (int i = 0; i < num_cameras; i++) {
        int ret = cameraModule->get_camera_info(i, &camera_info);
        if (ret != 0) {
            printf("Get info from camera %d failed. ret=%d\n", i, ret);
            return ret;
        } else {
            printf(" camera %d\n", i);
        }
    }

    return result;
}




// Callback from camera HAL preview
void preview_callback(BufferBlock* buffer, int frameNum)
{
    // std::unique_lock<std::mutex> lock(saveFrameMutex);
    // if (savePreviewNow) {
    //     char filePath[256];
    //     time_t now;
    //     time(&now);
    //     struct tm *timeValue = localtime(&now);
    //     snprintf(filePath, sizeof(filePath), "%s/preview.yuv",
    //         CAPTURE_OUTPUT_DIR
    //         );
    //     ::SaveFrame(filePath, buffer);
    //     printf("Save preview frame %d to:\n %s\n", frameNum, filePath);
    //     savePreviewNow = false;
    //     saveFrameCondition.notify_all();

    //     image = cv::imread(filePath,cv::IMREAD_COLOR) ;
    //     // yuv to bgr
    //     // cv::cvtColor(image, image_proc, cv::COLOR_YUV2BGR_NV12);

    // }
}


// Callback from camera HAL snapshot
void snapshot_callback(BufferBlock* buffer, int frameNum)
{
    /* --------------if image buffer write to file then read-------------*/
    
    // char filePath[256];
    // time_t now;
    // time(&now);
    // struct tm *timeValue = localtime(&now);
    // snprintf(filePath, sizeof(filePath), "%s/snapshot.jpg",
    //     CAPTURE_OUTPUT_DIR );
    // ::SaveFrame(filePath, buffer);
    // saveFrameCondition.notify_all();
    // image = cv::imread(filePath,cv::IMREAD_COLOR) ;  
    

    /* -----------------if directly read from buffer-------------* */
    
    unsigned long int size  = buffer->size;
    unsigned char* data_c = (unsigned char*)buffer->vaddress;
    printf("read buffer address %c ", data_c);
    printf(", read buffer size: %i ", size);
    printf("snapshot");

    std::vector<uchar> data(data_c, data_c+ size); 
    cv::Mat img_decode , image;
    img_decode= cv::imdecode(data,cv::IMREAD_COLOR);
    image = img_decode;
    // imshow("pic",img_decode);
    // cvWaitKey(10000);
    saveFrameCondition.notify_all();
    printf("Save snapshot frame %d \n", frameNum);

}


// Callback struct for camera streams.
CameraStreamCallbacks cameraStreamCallbacks =
{
    preview_callback,
    snapshot_callback
};


static void dumpPreviewFrame()
{
   std::unique_lock<std::mutex> lock(saveFrameMutex);
   savePreviewNow = true;
}

static void waitSaveFrame()
{
    std::unique_lock<std::mutex> lock(saveFrameMutex);
    saveFrameCondition.wait(lock);
}

void toCV(int count)
{
    ::snapshot();
    waitSaveFrame();
    printf(" %i to cv.\n",count);

}
void toROS(int count)
{
    ros::Time timestamp = ros::Time::now();
    // CV2ROS msg
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->encoding= std::string("bgr8");
    msg->header.frame_id = "hires";
    msg->header.stamp = timestamp;
    msg->height=640;
    msg->width=360;
    msg->is_bigendian = 0;
    printf(" %i to ros.\n",count);

}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "image_publisher");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/cam0/image_raw", 256);

    int ret;
    int camera_id, preview_width, preview_height, snapshot_width, snapshot_height;
    bool finished = false;
    std::string command;


// Initial Camera Paramters
   
    camera_id = 0;

    preview_width = 640;
    preview_height = 360;
    snapshot_width = 640;
    snapshot_height = 360;

    //Camera_configuration setting
    CamxHAL3Config* conf = new CamxHAL3Config();
    conf->cameraId = camera_id;
    conf->previewStream.width = preview_width;
    conf->previewStream.height = preview_height;
    conf->previewStream.format = HAL_PIXEL_FORMAT_YCBCR_420_888;
    conf->snapshotStream.width = snapshot_width;
    conf->snapshotStream.height = snapshot_height;
    conf->snapshotStream.format = HAL_PIXEL_FORMAT_BLOB;
    ret = initial_module();
    if (ret != 0) {
        fprintf(stderr, "Initial camera HAL module failed\n");
        return -1;
    }

    ret = mkdir(CAPTURE_OUTPUT_DIR, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (ret != 0 && errno != EEXIST) {
        fprintf(stderr, "Create output dir %s/ failed. Error=%d\n", CAPTURE_OUTPUT_DIR, errno);
        return -1;
    }

    ret = ::startPreview(cameraModule, conf, &cameraStreamCallbacks);
    if (ret < 0) {
        fprintf(stderr, "exit\n");
        return -1;
    }


    int count =1;

    /*single thread:*/
    ros::Rate loop_rate(50);
    //  While ROS is openning
    
    while (nh.ok()) {


        
        thread mThread2(toCV,count);
                
        /*multi thread*/
        mThread2.join();

        thread mThread3(toROS,count);

        mThread3.join();

        pub.publish(msg);
        // printf("image %i to ros.\n",count);


        ros::spinOnce();
        loop_rate.sleep();
        count ++;



    }
        



}