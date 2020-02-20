#include "SingleKinect.h"

// 初始化私有成员
ws_tech::SingleKinect::SingleKinect(py::function process_emotion_val, int device_index_val) :
	process_emotion(process_emotion_val),
	device_index(device_index_val),
	device(nullptr),
	device_config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
	sensor_calibration({}),
	sensor_capture(nullptr),
	color_image(NULL),
	depth_image(NULL),
	tracker(NULL),
	tracker_config(K4ABT_TRACKER_CONFIG_DEFAULT),
	body_frame(NULL),
	udp_sender(ws_tech::UDPSender("127.0.0.1", 8999))
{}

// 打开并启动 Kinect
void ws_tech::SingleKinect::Open()
{
	// 打开 Azure Kinect 设备。
	// --------------------------------------------------
	// k4a_result_t k4a_device_open(uint32_t index, k4a_device_t * device_handle)
	// --------------------------------------------------
	// Parameters
	// index: The index of the device to open, starting with 0. Optionally pass in K4A_DEVICE_DEFAULT.
	// device_handle: Output parameter which on success will return a handle to the device.
	// --------------------------------------------------
	// Returns
	// K4A_RESULT_SUCCEEDED if the device was opened successfully.
	verify(k4a_device_open(0, &device), "Open K4A Device succeed!", "Open K4A Device failed!");

	// depth 相机配置。
	// --------------------------------------------------
	// Depth sensor capture modes.
	// K4A_DEPTH_MODE_OFF
	// Depth sensor will be turned off with this setting.
	// --------------------------------------------------
	// K4A_DEPTH_MODE_NFOV_2X2BINNED
	// Depth captured at 320x288.
	// Passive IR is also captured at 320x288.
	// --------------------------------------------------
	// K4A_DEPTH_MODE_NFOV_UNBINNED
	// Depth captured at 640x576.
	// Passive IR is also captured at 640x576.
	// --------------------------------------------------
	// K4A_DEPTH_MODE_WFOV_2X2BINNED
	// Depth captured at 512x512.
	// Passive IR is also captured at 512x512.
	// --------------------------------------------------
	// K4A_DEPTH_MODE_WFOV_UNBINNED
	// Depth captured at 1024x1024.
	// Passive IR is also captured at 1024x1024.
	// --------------------------------------------------
	// K4A_DEPTH_MODE_PASSIVE_IR
	// Passive IR only, captured at 1024x1024.
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

	// color 相机配置
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_OFF
	// Color camera will be turned off with this setting.
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_720P
	// 1280 * 720 16:9
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_1080P
	// 1920 * 1080 16 : 9
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_1440P
	// 2560 * 1440 16 : 9
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_1536P
	// 2048 * 1536 4 : 3
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_2160P
	// 3840 * 2160 16 : 9
	// --------------------------------------------------
	// K4A_COLOR_RESOLUTION_3072P
	// 4096 * 3072 4 : 3
	device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	// synchronized_images_only
	// Only produce k4a_capture_t objects if they contain synchronized colorand depth images.
	// This setting controls the behavior in which images are dropped when images are produced faster than they can be read, or if there are errors in reading images from the device.
	// If set to true, k4a_capture_t objects will only be produced with both colorand depth images.If set to false, k4a_capture_t objects may be produced only a single image when the corresponding image is dropped.
	// Setting this to false ensures that the caller receives all of the images received from the camera, regardless of whether the corresponding images expected in the capture are available.
	// If either the color or depth camera are disabled, this setting has no effect.
	device_config.synchronized_images_only = true;
	// 其他配置
	device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;

	// 启动 color 和 depth 相机捕获。
	// --------------------------------------------------
	// k4a_result_t k4a_device_start_cameras(k4a_device_t 	device_handle, const k4a_device_configuration_t * config)
	// --------------------------------------------------
	// Parameters
	// device_handle: Handle obtained by k4a_device_open().
	// config: The configuration we want to run the device in. This can be initialized with K4A_DEVICE_CONFIG_INIT_DISABLE_ALL.
	// --------------------------------------------------
	// Returns
	// K4A_RESULT_SUCCEEDED is returned on success.
	verify(k4a_device_start_cameras(device, &device_config), "Start K4A cameras succeed!", "Start K4A cameras failed!");

	// 获取整个 Azure Kinect 设备的标定信息。
	// --------------------------------------------------
	// k4a_result_t k4a_device_get_calibration(k4a_device_t 	device_handle,
	//    const k4a_depth_mode_t 	depth_mode,
	//    const k4a_color_resolution_t 	color_resolution,
	//    k4a_calibration_t * calibration
	// )
	// --------------------------------------------------
	// Parameters
	//    device_handle: Handle obtained by k4a_device_open().
	//    depth_mode: Mode in which depth camera is operated.
	//    color_resolution: Resolution in which color camera is operated.
	//    calibration: Location to write the calibration
	// --------------------------------------------------
	// Returns
	//    K4A_RESULT_SUCCEEDED if calibration was successfully written.K4A_RESULT_FAILED otherwise.
	verify(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration),
		"Get depth camera calibration succeed!",
		"Get depth camera calibration failed!");

	// Start IMU sample stream for gravity vector.
	// 该函数与相机的状态有关，必须在 depth 和 color 相机开启之后使用。
	// --------------------------------------------------
	// k4a_result_t k4a_device_start_imu(k4a_device_t 	device_handle)
	// --------------------------------------------------
	// Parameters
	//    device_handle: Handle obtained by k4a_device_open().
	// --------------------------------------------------
	// Returns
	//    K4A_RESULT_SUCCEEDED is returned on success.K4A_RESULT_FAILED if the sensor is already running or a failure is encountered
	verify(k4a_device_start_imu(device), "Start IMU succeed!", "Start IMU failed!");

	// Create a body tracker handle.
	// 用完之后，需要使用 k4abt_tracker_destroy() 进行关闭。
	// --------------------------------------------------
	// Parameters
	//	sensor_calibration: The sensor calibration that will be used for capture processing.
	//	config: The configuration we want to run the tracker in.This can be initialized with K4ABT_TRACKER_CONFIG_DEFAULT.
	//	tracker_handle: Output parameter which on success will return a handle to the body tracker.
	// --------------------------------------------------
	// Returns
	//	K4A_RESULT_SUCCEEDED if the body tracker handle was created successfully.
	verify(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization succeed.", "Body tracker initialization failed!");
}

// 逐帧计算的入口函数
void ws_tech::SingleKinect::Running(int max_frame)
{
	Samples::PointCloudGenerator point_cloud_generator{ sensor_calibration };

	int frame_count = 0;
	// 不断处理每一帧
	while (frame_count < max_frame || max_frame == -1)
	{
		// 读取 sensor capture。
		// --------------------------------------------------
		// k4a_wait_result_t k4a_device_get_capture(k4a_device_t 	device_handle,
		//    k4a_capture_t * capture_handle,
		//    int32_t 	timeout_in_ms
		// )
		// --------------------------------------------------
		// Parameters
		//    device_handle: Handle obtained by k4a_device_open().
		//    capture_handle: If successful this contains a handle to a capture object.Caller must call k4a_capture_release() when its done using this capture.
		//    timeout_in_ms: Specifies the time in milliseconds the function should block waiting for the capture. If set to 0, 
		//                   the function will return without blocking. Passing a value of K4A_WAIT_INFINITE will block indefinitely
		//                   until data is available, the device is disconnected, or another error occurs.
		// --------------------------------------------------
		// Returns
		//    K4A_WAIT_RESULT_SUCCEEDED if a capture is returned.
		//    If a capture is not available before the timeout elapses, the function will return K4A_WAIT_RESULT_TIMEOUT.
		//    All other failures will return K4A_WAIT_RESULT_FAILED.
		// --------------------------------------------------
		// k4a_capture_t 类型
		// A capture represents a set of images that were captured by a device at approximately the same time.
		// A capture may have a color, IR, and depth image.A capture may have up to one image of each type.
		// A capture may have no image for a given type as well.
		// Captures also store a temperature value which represents the temperature of the device at the time of the capture.
		// The caller must call k4a_capture_release() to release its reference to any k4a_capture_t that it receives from an Azure Kinect API.
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, 3000); // timeout_in_ms is set to 3000

		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			frame_count++;
			std::cout << frame_count << std::endl;

			color_image = k4a_capture_get_color_image(sensor_capture);
			updateEmotionResult(); // 表情识别
			k4a_image_release(color_image);

			depth_image = k4a_capture_get_depth_image(sensor_capture);
			updateFloorResult(point_cloud_generator); // 地面检测
			if (updateBodyTrackingResult() == false) break; // 姿态识别
			k4a_image_release(depth_image);

			k4a_capture_release(sensor_capture);
		}
		else if (get_capture_result != K4A_WAIT_RESULT_TIMEOUT)
		{
			std::cout << "Get depth capture returned error: " << get_capture_result << std::endl;
			break;
		}

		if (is_debug)
		{
			if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q')
			{
				break;
			}
		}

	}

	if (is_debug)
	{
		cv::destroyAllWindows();
	}

}

// 关闭 Kinect
void ws_tech::SingleKinect::Close()
{
	printf("Finished emotion and body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_stop_imu(device);
	k4a_device_close(device);
}

void print_body_information(k4abt_body_t body)
{
	printf("Body ID: %u\n", body.id);
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
	{
		k4a_float3_t position = body.skeleton.joints[i].position;
		k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
		k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[i].confidence_level;
		printf("Joint[%d]: Position[mm] ( %f, %f, %f ); Orientation ( %f, %f, %f, %f); Confidence Level (%d) \n",
			i, position.v[0], position.v[1], position.v[2], orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3], confidence_level);
	}
}

void print_body_index_map_middle_line(k4a_image_t body_index_map)
{
	uint8_t* body_index_map_buffer = k4a_image_get_buffer(body_index_map);

	// Given body_index_map pixel type should be uint8, the stride_byte should be the same as width
	// TODO: Since there is no API to query the byte-per-pixel information, we have to compare the width and stride to
	// know the information. We should replace this assert with proper byte-per-pixel query once the API is provided by
	// K4A SDK.
	assert(k4a_image_get_stride_bytes(body_index_map) == k4a_image_get_width_pixels(body_index_map));

	int middle_line_num = k4a_image_get_height_pixels(body_index_map) / 2;
	body_index_map_buffer = body_index_map_buffer + middle_line_num * k4a_image_get_width_pixels(body_index_map);

	printf("BodyIndexMap at Line %d:\n", middle_line_num);
	for (int i = 0; i < k4a_image_get_width_pixels(body_index_map); i++)
	{
		printf("%u, ", *body_index_map_buffer);
		body_index_map_buffer++;
	}
	printf("\n");
}

// 处理表情的逻辑
// 使用了 color frame
void ws_tech::SingleKinect::updateEmotionResult()
{

	cv::Mat color_frame = cv::Mat(k4a_image_get_height_pixels(color_image), k4a_image_get_width_pixels(color_image), CV_8UC4, k4a_image_get_buffer(color_image));  // 还可以使用 CV_8UC4，4通道数

			// 对彩色图片进行预处理
	cv::cvtColor(color_frame, color_frame, cv::COLOR_BGRA2BGR);
	cv::Mat dst = cv::Mat(720, 1280, color_frame.type());
	cv::resize(color_frame, dst, dst.size(), 0, 0, cv::INTER_LINEAR);
	py::array_t<unsigned char> arr = cv_mat_uint8_3c_to_numpy(dst);
	// 调用外部 Python 环境进行表情识别
	py::list result = process_emotion(arr);
	std::cout << "表情数据：" << result << std::endl;

	//cv::imshow("测试 color 相机 1 ", dst);  // 测试
	//cv::imshow("测试 color 相机 1 ", numpy_uint8_3c_to_cv_mat(arr));  // 测试


}

// 处理姿态的逻辑
// 使用了 sensor_capture 获取 tracker
bool ws_tech::SingleKinect::updateBodyTrackingResult()
{
	k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
	if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		// It should never hit timeout when K4A_WAIT_INFINITE is set.
		printf("Error! Add capture to tracker process queue timeout!\n");
		return false;
	}
	else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
	{
		printf("Error! Add capture to tracker process queue failed!\n");
		return false;
	}

	k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		// Successfully popped the body tracking result. Start your processing
		processBodyFrame();
		return true;
	}
	else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		//  It should never hit timeout when K4A_WAIT_INFINITE is set.
		printf("Error! Pop body frame result timeout!\n");
		return false;
	}
	else
	{
		printf("Pop body frame result failed!\n");
		return false;
	}
}

// Vector3f 转 k4a_float3_t
void setPos(k4a_float3_t& pos, Eigen::Vector3f& v)
{
	pos.xyz.x = v(0);
	pos.xyz.y = v(1);
	pos.xyz.z = v(2);
}

// 通过 k4a_float3_t 构造 Vector3f
Eigen::Vector3f getVector(k4a_float3_t& pos1, k4a_float3_t& pos2)
{
	Eigen::Vector3f v;
	v << (pos2.xyz.x - pos1.xyz.x), (pos2.xyz.y - pos1.xyz.y), (pos2.xyz.z - pos1.xyz.z);
	return v;
}

// 增加扭腰效果
void enhanceWaistTwisted(k4abt_skeleton_t* skeleton)
{
	// 有关的节点位置对象
	k4a_float3_t& head_pos = skeleton->joints[K4ABT_JOINT_HEAD].position;
	k4a_float3_t& spine_chest_pos = skeleton->joints[K4ABT_JOINT_SPINE_CHEST].position;
	k4a_float3_t& spine_navel_pos = skeleton->joints[K4ABT_JOINT_SPINE_NAVEL].position;
	k4a_float3_t& pelvis_pos = skeleton->joints[K4ABT_JOINT_PELVIS].position;
	k4a_float3_t& hip_left_pos = skeleton->joints[K4ABT_JOINT_HIP_LEFT].position;
	k4a_float3_t& hip_right_pos = skeleton->joints[K4ABT_JOINT_HIP_RIGHT].position;

	// 计算一个单位向量和两个距离
	Eigen::Vector3f head_to_chest_identity = getVector(head_pos, spine_chest_pos).Identity();
	int chest_to_naval_norm = getVector(spine_chest_pos, spine_navel_pos).norm();
	int chest_to_pelvis_norm = getVector(spine_chest_pos, pelvis_pos).norm();

	// 计算新的坐标
	Eigen::Vector3f new_naval_v = (Eigen::Vector3f)spine_chest_pos.v + chest_to_naval_norm * head_to_chest_identity;
	Eigen::Vector3f new_pelvis_v = (Eigen::Vector3f)spine_chest_pos.v + chest_to_pelvis_norm * head_to_chest_identity;

	Eigen::Vector3f pelvis_diff_v = new_pelvis_v - (Eigen::Vector3f)pelvis_pos.v;
	Eigen::Vector3f new_hip_left_v = (Eigen::Vector3f)hip_left_pos.v + pelvis_diff_v;
	Eigen::Vector3f new_hip_right_v = (Eigen::Vector3f)hip_right_pos.v + pelvis_diff_v;

	// 覆盖旧坐标
	setPos(spine_navel_pos, new_naval_v);
	setPos(pelvis_pos, new_pelvis_v);
	setPos(hip_left_pos, new_hip_left_v);
	setPos(hip_right_pos, new_hip_right_v);
}

// 姿态识别的核心处理逻辑
void ws_tech::SingleKinect::processBodyFrame()
{
	size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	std::cout << num_bodies << "bodies are detected!" << std::endl;

	if (is_debug)
	{
		for (size_t i = 0; i < num_bodies; i++)
		{
			k4abt_body_t body;
			VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame succeed.", "Get body from body frame failed!");
			body.id = k4abt_frame_get_body_id(body_frame, i);

			print_body_information(body);
		}
	}

	if (num_bodies <= 0)
	{
		return;  // 如果检测不到人，则什么也不做
	}

	k4abt_skeleton_t skeleton;
	k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

	if (skeleton_result == K4A_RESULT_SUCCEEDED)
	{
		// Successfully get skeleton for the i-th person. Start processing.
		std::vector<k4abt_joint_t> skeleton_matrix;
		// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
		// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
		for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
			skeleton_matrix.push_back(skeleton.joints[i]);
		}

		// 创建一个临时变量 tmp 用于保存关节点数据
		// 把 skeleton_matrix 的节点 3D 位置赋值给 tmp
		// 这里把 tmp(i, j) 中的 i 作为了 X、Y、Z，未来可能需要做优化
		Eigen::Matrix3Xf tmp;
		tmp.resize(3, skeleton_matrix.size());
		for (int i = 0; i < skeleton_matrix.size(); i++) {
			tmp(0, i) = skeleton_matrix.at(i).position.xyz.x;
			tmp(1, i) = skeleton_matrix.at(i).position.xyz.y;
			tmp(2, i) = skeleton_matrix.at(i).position.xyz.z;
		}

		// 使用 TrackerProcessor 成员变量旋转矩阵对 tmp 进行旋转
		Eigen::Matrix3f view_rotation;
		view_rotation << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
		tmp = view_rotation * tmp;

		// 计算平移量
		// mHasShift 相当于开关，保证计算平移量的逻辑只进行一次。
		if (!has_shifted) {
			float sumX = 0;  // 用于保存所有节点的 X 值的和
			float sumY = 0;  // 用于保存所有节点的 Y 值的和
			float minZ = 9999;  // 用于保存所有节点的 Z 值得最小值
			for (int i = 0; i < tmp.cols(); i++) {
				sumX += tmp(0, i);
				sumY += tmp(1, i);

				if (minZ > tmp(2, i)) {
					minZ = tmp(2, i);
				}
			}
			shift_vector << -sumX / skeleton_matrix.size(), -sumY / skeleton_matrix.size(), -minZ;  // 该平移矩阵是：-X平均值、-Y平均值、-Z最小值
			// 关闭开关，该代码块的逻辑不会再调用
			has_shifted = true;
		}

		// 依次给每个列向量加上位移量
		// 可优化
		for (int i = 0; i < tmp.cols(); i++) {
			tmp(0, i) += shift_vector(0);
			tmp(1, i) += shift_vector(1);
			tmp(2, i) += shift_vector(2);
		}

		// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
		// 仅仅是坐标值，四元数等信息并没有经过调整
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
			skeleton.joints[i].position.xyz.x = tmp(0, i);
			skeleton.joints[i].position.xyz.y = tmp(1, i);
			skeleton.joints[i].position.xyz.z = tmp(2, i);
		}

		// Am I really need this???
		// enhanceWaistTwisted(&skeleton);

		// OK, set strings data.
		std::stringstream ss;
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		{
			k4a_float3_t position = skeleton.joints[i].position;
			ss << position.xyz.x << " " << position.xyz.y << " " << position.xyz.z << ", ";
		}
		ss << "| ";
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		{
			k4a_quaternion_t orientation = skeleton.joints[i].orientation;
			//k4a_quaternion_t orientation = skeleton_matrix[i].orientation;
			//ss << orientation.wxyz.w << " " << orientation.wxyz.x << " " << orientation.wxyz.y << " " << orientation.wxyz.z << ", ";
			Eigen::Quaternionf result = Eigen::Quaternionf(view_rotation) * Eigen::Quaternionf(orientation.wxyz.w, orientation.wxyz.x, orientation.wxyz.y, orientation.wxyz.z);
			ss << result.coeffs()(3, 0) << " ";
			ss << result.coeffs()(0, 0) << " ";
			ss << result.coeffs()(1, 0) << " ";
			ss << result.coeffs()(2, 0) << ", ";
		}

		// Send results with UDP
		udp_sender.Send(ss.str());
	}
	else if (skeleton_result == K4A_RESULT_FAILED)
	{
		std::cout << "Error! Get body skeleton failed!" << std::endl;
	}

	k4abt_frame_release(body_frame);
}

// 检测地面的逻辑
// 使用了 depth frame
void ws_tech::SingleKinect::updateFloorResult(Samples::PointCloudGenerator& point_cloud_generator)
{
	// Capture an IMU sample for sensor orientation.
	k4a_imu_sample_t imu_sample;
	if (k4a_device_get_imu_sample(device, &imu_sample, 0) == K4A_WAIT_RESULT_SUCCEEDED)
	{
		// Update point cloud.
		point_cloud_generator.Update(depth_image);

		// Get down-sampled cloud points.
		const int downsampleStep = 2;
		const auto& cloudPoints = point_cloud_generator.GetCloudPoints(downsampleStep);

		// Detect floor plane based on latest visual and inertial observations.
		const size_t minimumFloorPointCount = 1024 / (downsampleStep * downsampleStep);
		const auto& maybeFloorPlane = floor_detector.TryDetectFloorPlane(cloudPoints, imu_sample, sensor_calibration, minimumFloorPointCount);

		// Visualize point cloud.
		//window3d.UpdatePointClouds(depthImage);

		// Visualize the floor plane.
		if (maybeFloorPlane.has_value())
		{
			// For visualization purposes, make floor origin the projection of a point 1.5m in front of the camera.
			Samples::Vector cameraOrigin = { 0, 0, 0 };
			Samples::Vector cameraForward = { 0, 0, 1 };

			auto p = maybeFloorPlane->ProjectPoint(cameraOrigin) + maybeFloorPlane->ProjectVector(cameraForward) * 1.5f;
			auto n = maybeFloorPlane->Normal;

			std::cout << "地面检测 n.X ：" << n.X << std::endl;
			//window3d.SetFloorRendering(true, p.X, p.Y, p.Z, n.X, n.Y, n.Z);
		}
		else
		{
			//window3d.SetFloorRendering(false, 0, 0, 0);
		}
	}
}

