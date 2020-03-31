#include "azureKinectCapture.h"
#include <chrono>

AzureKinectCapture::AzureKinectCapture()
{

}

AzureKinectCapture::~AzureKinectCapture()
{
	k4a_image_release(colorImage);
	k4a_image_release(depthImage);
	k4a_image_release(depthPointCloudImage);
	k4a_image_release(colorImageInDepth);
	k4a_image_release(depthImageInColor);
	k4a_transformation_destroy(transformation);
	k4a_device_close(kinectSensor);
}

bool AzureKinectCapture::Initialize()
{
	uint32_t count = k4a_device_get_installed_count();
	int deviceIdx = 0;

	// 打开 Kinect，kinectSensor 变量控制设备。
	// 如果失败，就继续打开下一个设备号，直到超过所有 Kinect 个数为止。
	kinectSensor = NULL;
	while (K4A_FAILED(k4a_device_open(deviceIdx, &kinectSensor)))
	{
		deviceIdx++;
		if (deviceIdx >= count)
		{
			bInitialized = false;
			return bInitialized;
		}
	}

	// 配置是硬编码的
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.synchronized_images_only = true;

	// Start the camera with the given configuration
	bInitialized = K4A_SUCCEEDED(k4a_device_start_cameras(kinectSensor, &config));

	// 获取标定变量，创建 transformation
	k4a_calibration_t calibration;
	if (K4A_FAILED(k4a_device_get_calibration(kinectSensor, config.depth_mode, config.color_resolution, &calibration)))
	{
		bInitialized = false;
		return bInitialized;
	}
	transformation = k4a_transformation_create(&calibration);

	// 如果成功获取 frame，则进入下一步。如果等待时间超过 5 秒，则进入下一步，并且 bInitialized = False
	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	bool bTemp;
	do
	{
		// 如果成功获取，则 bTemp = True
		bTemp = AcquireFrame();

		std::chrono::duration<double> elapsedSeconds = std::chrono::system_clock::now() - start;
		if (elapsedSeconds.count() > 5.0)
		{
			bInitialized = false;
			break;
		}
	} while (!bTemp);

	size_t serialNoSize;
	k4a_device_get_serialnum(kinectSensor, NULL, &serialNoSize);
	serialNumber = std::string(serialNoSize, '\0');
	k4a_device_get_serialnum(kinectSensor, (char*)serialNumber.c_str(), &serialNoSize);

	// 最后返回初始化是否成功。
	return bInitialized;
}

bool AzureKinectCapture::AcquireFrame()
{
	// 常规操作，得到 capture
	if (!bInitialized)
	{
		return false;
	}

	k4a_capture_t capture = NULL;

	k4a_wait_result_t captureResult = k4a_device_get_capture(kinectSensor, &capture, captureTimeoutMs);
	if (captureResult != K4A_WAIT_RESULT_SUCCEEDED)
	{
		k4a_capture_release(capture);
		return false;
	}

	k4a_image_release(colorImage);
	k4a_image_release(depthImage);

	// 得到 color 和 depth
	colorImage = k4a_capture_get_color_image(capture);
	depthImage = k4a_capture_get_depth_image(capture);
	if (colorImage == NULL || depthImage == NULL)
	{
		k4a_capture_release(capture);
		return false;
	}

	// 赋值 pColorRGBX 和 pDepth
	if (pColorRGBX == NULL)
	{
		nColorFrameHeight = k4a_image_get_height_pixels(colorImage);
		nColorFrameWidth = k4a_image_get_width_pixels(colorImage);
		pColorRGBX = new RGB[nColorFrameWidth * nColorFrameHeight];
	}

	if (pDepth == NULL)
	{
		nDepthFrameHeight = k4a_image_get_height_pixels(depthImage);
		nDepthFrameWidth = k4a_image_get_width_pixels(depthImage);
		pDepth = new UINT16[nDepthFrameHeight * nDepthFrameWidth];
	}

	memcpy(pColorRGBX, k4a_image_get_buffer(colorImage), nColorFrameWidth * nColorFrameHeight * sizeof(RGB));
	memcpy(pDepth, k4a_image_get_buffer(depthImage), nDepthFrameHeight * nDepthFrameWidth * sizeof(UINT16));

	{
		k4abt_tracker_t tracker{NULL};
		k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
		if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			// It should never hit timeout when K4A_WAIT_INFINITE is set.
			printf("Error! Add capture to tracker process queue timeout!\n");
		}
		else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
		{
			printf("Error! Add capture to tracker process queue failed!\n");
		}

		k4abt_frame_t body_frame{ NULL };
		k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			// Successfully popped the body tracking result. Start your processing
			size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
			// std::cout << num_bodies << "bodies are detected!" << std::endl;

			//if (num_bodies <= 0)
			//{
			//	return true;  // 如果检测不到人，则什么也不做
			//}

			k4abt_skeleton_t skeleton;
			k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

			//if (skeleton_result == K4A_RESULT_SUCCEEDED)
			//{
			//	// Successfully get skeleton for the i-th person. Start processing.
			//	std::vector<k4abt_joint_t> skeleton_matrix;
			//	// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
			//	// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
			//	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
			//		skeleton_matrix.push_back(skeleton.joints[i]);
			//	}

			//	// 创建一个临时变量 tmp 用于保存关节点数据
			//	// 把 skeleton_matrix 的节点 3D 位置赋值给 tmp
			//	// 这里把 tmp(i, j) 中的 i 作为了 X、Y、Z，未来可能需要做优化
			//	Eigen::Matrix3Xf tmp;
			//	tmp.resize(3, skeleton_matrix.size());
			//	for (int i = 0; i < skeleton_matrix.size(); i++) {
			//		tmp(0, i) = skeleton_matrix.at(i).position.xyz.x;
			//		tmp(1, i) = skeleton_matrix.at(i).position.xyz.y;
			//		tmp(2, i) = skeleton_matrix.at(i).position.xyz.z;
			//	}

			//	// 使用 TrackerProcessor 成员变量旋转矩阵对 tmp 进行旋转
			//	Eigen::Matrix3f view_rotation;
			//	view_rotation << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
			//	tmp = view_rotation * tmp;

			//	// 计算平移量
			//	// mHasShift 相当于开关，保证计算平移量的逻辑只进行一次。
			//	if (!has_shifted) {
			//		float sumX = 0;  // 用于保存所有节点的 X 值的和
			//		float sumY = 0;  // 用于保存所有节点的 Y 值的和
			//		float minZ = 9999;  // 用于保存所有节点的 Z 值得最小值
			//		for (int i = 0; i < tmp.cols(); i++) {
			//			sumX += tmp(0, i);
			//			sumY += tmp(1, i);

			//			if (minZ > tmp(2, i)) {
			//				minZ = tmp(2, i);
			//			}
			//		}
			//		shift_vector << -sumX / skeleton_matrix.size(), -sumY / skeleton_matrix.size(), -minZ;  // 该平移矩阵是：-X平均值、-Y平均值、-Z最小值
			//		// 关闭开关，该代码块的逻辑不会再调用
			//		has_shifted = true;
			//	}

			//	// 依次给每个列向量加上位移量
			//	// 可优化
			//	for (int i = 0; i < tmp.cols(); i++) {
			//		tmp(0, i) += shift_vector(0);
			//		tmp(1, i) += shift_vector(1);
			//		tmp(2, i) += shift_vector(2);
			//	}

			//	// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
			//	// 仅仅是坐标值，四元数等信息并没有经过调整
			//	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
			//		skeleton.joints[i].position.xyz.x = tmp(0, i);
			//		skeleton.joints[i].position.xyz.y = tmp(1, i);
			//		skeleton.joints[i].position.xyz.z = tmp(2, i);
			//	}

			//	// Am I really need this???
			//	// enhanceWaistTwisted(&skeleton);

			//	// OK, set strings data.
			//	std::stringstream ss;
			//	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
			//	{
			//		k4a_float3_t position = skeleton.joints[i].position;
			//		ss << position.xyz.x << " " << position.xyz.y << " " << position.xyz.z << ", ";
			//	}
			//	ss << "| ";
			//	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
			//	{
			//		k4a_quaternion_t orientation = skeleton.joints[i].orientation;
			//		//k4a_quaternion_t orientation = skeleton_matrix[i].orientation;
			//		//ss << orientation.wxyz.w << " " << orientation.wxyz.x << " " << orientation.wxyz.y << " " << orientation.wxyz.z << ", ";
			//		Eigen::Quaternionf result = Eigen::Quaternionf(view_rotation) * Eigen::Quaternionf(orientation.wxyz.w, orientation.wxyz.x, orientation.wxyz.y, orientation.wxyz.z);
			//		ss << result.coeffs()(3, 0) << " ";
			//		ss << result.coeffs()(0, 0) << " ";
			//		ss << result.coeffs()(1, 0) << " ";
			//		ss << result.coeffs()(2, 0) << ", ";
			//	}
			//	ss << " | 0,";

			//	// Send results with UDP
			//	udp_sender.Send(ss.str());
			//}
			//else if (skeleton_result == K4A_RESULT_FAILED)
			//{
			//	std::cout << "Error! Get body skeleton failed!" << std::endl;
			//}

			k4abt_frame_release(body_frame);
		}
		else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			//  It should never hit timeout when K4A_WAIT_INFINITE is set.
			printf("Error! Pop body frame result timeout!\n");
		}
		else
		{
			printf("Pop body frame result failed!\n");
		}
	}

	k4a_capture_release(capture);

	return true;
}

void AzureKinectCapture::UpdateDepthPointCloud()
{
	if (depthPointCloudImage == NULL)
	{
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nDepthFrameWidth, nDepthFrameHeight,
			nDepthFrameWidth * 3 * (int)sizeof(int16_t),
			&depthPointCloudImage);
	}

	k4a_transformation_depth_image_to_point_cloud(transformation, depthImage, K4A_CALIBRATION_TYPE_DEPTH, depthPointCloudImage);
}

void AzureKinectCapture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	UpdateDepthPointCloud();

	int16_t* pointCloudData = (int16_t*)k4a_image_get_buffer(depthPointCloudImage);

	for (int i = 0; i < nDepthFrameHeight; i++)
	{
		for (int j = 0; j < nDepthFrameWidth; j++)
		{
			pCameraSpacePoints[j + i * nDepthFrameWidth].X = pointCloudData[3 * (j + i * nDepthFrameWidth) + 0] / 1000.0f;
			pCameraSpacePoints[j + i * nDepthFrameWidth].Y = pointCloudData[3 * (j + i * nDepthFrameWidth) + 1] / 1000.0f;
			pCameraSpacePoints[j + i * nDepthFrameWidth].Z = pointCloudData[3 * (j + i * nDepthFrameWidth) + 2] / 1000.0f;
		}
	}
}

// This mapping is much slower then the other ones, use with caution
void AzureKinectCapture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	UpdateDepthPointCloud();

	// Initializing temporary images
	k4a_image_t colorPointCloudImageX, colorPointCloudImageY, colorPointCloudImageZ, dummy_output_image;
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nColorFrameWidth, nColorFrameHeight,
		nColorFrameWidth * (int)sizeof(int16_t),
		&colorPointCloudImageX);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nColorFrameWidth, nColorFrameHeight,
		nColorFrameWidth * (int)sizeof(int16_t),
		&colorPointCloudImageY);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nColorFrameWidth, nColorFrameHeight,
		nColorFrameWidth * (int)sizeof(int16_t),
		&colorPointCloudImageZ);
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, nColorFrameWidth, nColorFrameHeight,
		nColorFrameWidth * (int)sizeof(int16_t),
		&dummy_output_image);

	k4a_image_t depthPointCloudX, depthPointCloudY, depthPointCloudZ;

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nDepthFrameWidth, nDepthFrameHeight,
		nDepthFrameWidth * (int)sizeof(int16_t),
		&depthPointCloudX);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nDepthFrameWidth, nDepthFrameHeight,
		nDepthFrameWidth * (int)sizeof(int16_t),
		&depthPointCloudY);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, nDepthFrameWidth, nDepthFrameHeight,
		nDepthFrameWidth * (int)sizeof(int16_t),
		&depthPointCloudZ);

	int16_t* depthPointCloudImage_buffer = (int16_t*)k4a_image_get_buffer(depthPointCloudImage);
	int16_t* depthPointCloudImageX_buffer = (int16_t*)k4a_image_get_buffer(depthPointCloudX);
	int16_t* depthPointCloudImageY_buffer = (int16_t*)k4a_image_get_buffer(depthPointCloudY);
	int16_t* depthPointCloudImageZ_buffer = (int16_t*)k4a_image_get_buffer(depthPointCloudZ);
	for (int i = 0; i < nDepthFrameWidth * nDepthFrameHeight; i++)
	{
		depthPointCloudImageX_buffer[i] = depthPointCloudImage_buffer[3 * i + 0];
		depthPointCloudImageY_buffer[i] = depthPointCloudImage_buffer[3 * i + 1];
		depthPointCloudImageZ_buffer[i] = depthPointCloudImage_buffer[3 * i + 2];
	}

	int test = 1 << 16;

	// Transforming per-depth-pixel point cloud to per-color-pixel point cloud
	k4a_transformation_depth_image_to_color_camera_custom(transformation, depthImage, depthPointCloudX,
		dummy_output_image, colorPointCloudImageX,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR, 0);
	k4a_transformation_depth_image_to_color_camera_custom(transformation, depthImage, depthPointCloudY,
		dummy_output_image, colorPointCloudImageY,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR, 0);
	k4a_transformation_depth_image_to_color_camera_custom(transformation, depthImage, depthPointCloudZ,
		dummy_output_image, colorPointCloudImageZ,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR, 0);


	int16_t* pointCloudDataX = (int16_t*)k4a_image_get_buffer(colorPointCloudImageX);
	int16_t* pointCloudDataY = (int16_t*)k4a_image_get_buffer(colorPointCloudImageY);
	int16_t* pointCloudDataZ = (int16_t*)k4a_image_get_buffer(colorPointCloudImageZ);
	for (int i = 0; i < nColorFrameHeight; i++)
	{
		for (int j = 0; j < nColorFrameWidth; j++)
		{
			pCameraSpacePoints[j + i * nColorFrameWidth].X = pointCloudDataX[j + i * nColorFrameWidth + 0] / 1000.0f;
			pCameraSpacePoints[j + i * nColorFrameWidth].Y = pointCloudDataY[j + i * nColorFrameWidth + 1] / 1000.0f;
			pCameraSpacePoints[j + i * nColorFrameWidth].Z = pointCloudDataZ[j + i * nColorFrameWidth + 2] / 1000.0f;
		}
	}

	k4a_image_release(colorPointCloudImageX);
	k4a_image_release(colorPointCloudImageY);
	k4a_image_release(colorPointCloudImageZ);

	k4a_image_release(dummy_output_image);
	k4a_image_release(depthPointCloudX);
	k4a_image_release(depthPointCloudY);
	k4a_image_release(depthPointCloudZ);
}

void AzureKinectCapture::MapDepthFrameToColorSpace(UINT16 *pDepthInColorSpace)
{
	if (depthImageInColor == NULL)
	{
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, nColorFrameWidth, nColorFrameHeight,
			nColorFrameWidth * (int)sizeof(uint16_t),
			&depthImageInColor);
	}

	k4a_transformation_depth_image_to_color_camera(transformation, depthImage, depthImageInColor);

	memcpy(pDepthInColorSpace, k4a_image_get_buffer(depthImageInColor), nColorFrameHeight * nColorFrameWidth * (int)sizeof(uint16_t));
}

void AzureKinectCapture::MapColorFrameToDepthSpace(RGB *pColorInDepthSpace)
{
	if (colorImageInDepth == NULL)
	{
		k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, nDepthFrameWidth, nDepthFrameHeight,
			nDepthFrameWidth * 4 * (int)sizeof(uint8_t),
			&colorImageInDepth);
	}

	k4a_transformation_color_image_to_depth_camera(transformation, depthImage, colorImage, colorImageInDepth);

	memcpy(pColorInDepthSpace, k4a_image_get_buffer(colorImageInDepth), nDepthFrameHeight * nDepthFrameWidth * 4 * (int)sizeof(uint8_t));
}