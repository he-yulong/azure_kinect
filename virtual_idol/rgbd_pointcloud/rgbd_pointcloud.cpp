// rgbd_pointcloud.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"

static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t color_image,
    std::string file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);  // 深度图的宽
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);  // 深度图的高
    k4a_image_t transformed_color_image = NULL;

    // 创建一张与深度图大小相同的 BGRA 的图
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
        &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    // 创建一张与深度图大小相同的 XYZ 的图
    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    // 把彩色图转换为深度相机的几何
    // This produces a color image for which each pixel matches the corresponding pixel coordinates of the depth camera.
    // 转换后的彩色图的每一个像素都对应着深度相机的每个像素坐标
    // depth_image and color_image need to represent the same moment in time. The depth data will be applied to the color image to properly warp the color data to the perspective of the depth camera.
    // depth_image must be of type K4A_IMAGE_FORMAT_DEPTH16. color_image must be of format K4A_IMAGE_FORMAT_COLOR_BGRA32.
    // transformed_color_image image must be of format K4A_IMAGE_FORMAT_COLOR_BGRA32. transformed_color_image must have the width and height of the depth camera in the mode specified by the k4a_calibration_t used to create the transformation_handle with k4a_transformation_create().
    // transformed_color_image should be created by the caller using k4a_image_create() or k4a_image_create_from_buffer().
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
        depth_image,
        color_image,
        transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    // Transforms the depth image into 3 planar images representing X, Y and Z-coordinates of corresponding 3D points.
    // The format of xyz_image must be K4A_IMAGE_FORMAT_CUSTOM. The width and height of xyz_image must match the width and height of depth_image.
    // xyz_image must have a stride in bytes of at least 6 times its width in pixels.
    // Each pixel of the xyz_image consists of three int16_t values, totaling 6 bytes. The three int16_t values are the X, Y, and Z values of the point.
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        depth_image,
        K4A_CALIBRATION_TYPE_DEPTH,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    // 写入文件
    tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static int capture(std::string output_dir, int frames)
{
    uint8_t deviceId = K4A_DEVICE_DEFAULT;  // 设备 ID
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;  // 定义延时时间
    k4a_transformation_t transformation = NULL;  // 用于坐标系变换
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;  // 设备数量，仅用于判断有无设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    int i = 0;
    int z = 0;
    k4a_capture_t* captures = new k4a_capture_t[frames];  // 保存帧的数组

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        goto Exit;
    }

    // 先进行捕获
    for (i; i < frames; i++)
    {
        // Get a capture
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            goto Exit;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }
        captures[i] = capture;
        printf("Frame Captured\n");
    }
    printf("Processing Frames...\n");

    // 捕获之后再依次获取彩色图和深度图
    for (z; z < frames; z++)
    {
        // Get a depth image
        depth_image = k4a_capture_get_depth_image(captures[z]);
        if (depth_image == 0)
        {
            printf("Failed to get depth image from capture\n");
            goto Exit;
        }

        // Get a color image
        color_image = k4a_capture_get_color_image(captures[z]);
        if (color_image == 0)
        {
            printf("Failed to get color image from capture\n");
            goto Exit;
        }

        // Compute color point cloud by warping color image into depth camera geometry
        file_name = output_dir + "\\" + std::to_string(z) + ".xyz";
        if (point_cloud_color_to_depth(transformation, depth_image, color_image, file_name.c_str()) == false)
        {
            goto Exit;
        }
    }
    printf("Finished!");
    returnCode = 0;

Exit:
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return returnCode;
}

static void print_usage()
{
    printf("Usage: transformation.exe capture <output_directory> <frames>\n");
    // printf("Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file]\n");
}

int main(int argc, char** argv)
{
    int returnCode = 0;

    if (argc < 4)
    {
        print_usage();
    }
    else
    {
        std::string mode = std::string(argv[1]);
        if (mode == "capture")
        {
            if (argc == 4)
            {
                // 把存储路径和帧数传过去
                returnCode = capture(argv[2], std::atoi(argv[3]));
            }
            else
            {
                print_usage();
            }
        }
    }

    return returnCode;
}
