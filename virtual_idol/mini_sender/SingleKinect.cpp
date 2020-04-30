#include "SingleKinect.h"

// 初始化私有成员
ws_tech::SingleKinect::SingleKinect(int device_index_val, std::string ip_addr, int udp_port, std::string udp_format) :
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
	udp_sender(ws_tech::UDPSender(ip_addr, udp_port))
{
	// 旋转矩阵初始化
	// SMPL -> Kinect
	rotation_matrix_type1 << 1, 0, 0, 0, -1, 0, 0, 0, -1;
	rotation_matrix_type2 << 1, 0, 0, 0, 0, 1, 0, -1, 0;


	quaternion_type1 = Eigen::Quaterniond(rotation_matrix_type1);
	quaternion_type2 = Eigen::Quaterniond(rotation_matrix_type2);

	
	rotation_matrix_type3 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;
	rotation_matrix_type4 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;
	rotation_matrix_type5 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;
	rotation_matrix_type6 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;

	if (udp_format == "kinect")
	{
		UDP_MAP_LEN = (int)K4ABT_JOINT_COUNT;
		udp_map = new int[UDP_MAP_LEN];
		for (int i = 0; i < UDP_MAP_LEN; i++)
		{
			udp_map[i] = i;
		}
	}
	else if (udp_format == "unity")
	{
		UDP_MAP_LEN = 17;
		udp_map = new int[UDP_MAP_LEN]{
			0,  // Hip
			22, // RHip
			23, // RKnee
			24, // RFoot
			18, // LHip
			19, // LKnee
			20, // LFoot
			1,  // Spine
			2,  // Thorax/Chest
			3,  // Neck/Nose
			26, // Head
			5,  // LShoulder
			6,  // LElbow
			7,  // LWrist
			12, // RShoulder
			13, // RElbow
			14, // RWrist
		};
		std::cout << "udp_format: unity" << std::endl;

	}
}

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
			if (updateBodyTrackingResult() == false) break; // 姿态识别
			k4a_capture_release(sensor_capture);
		}
		else if (get_capture_result != K4A_WAIT_RESULT_TIMEOUT)
		{
			std::cout << "Get depth capture returned error: " << get_capture_result << std::endl;
			break;
		}
	}
	udp_sender.Send("#STOP#");
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

//// Vector3f 转 k4a_float3_t
//void setPos(k4a_float3_t& pos, Eigen::Vector3f& v)
//{
//	pos.xyz.x = v(0);
//	pos.xyz.y = v(1);
//	pos.xyz.z = v(2);
//}
//
//// 通过 k4a_float3_t 构造 Vector3f
//Eigen::Vector3f getVector(k4a_float3_t& pos1, k4a_float3_t& pos2)
//{
//	Eigen::Vector3f v;
//	v << (pos2.xyz.x - pos1.xyz.x), (pos2.xyz.y - pos1.xyz.y), (pos2.xyz.z - pos1.xyz.z);
//	return v;
//}

// 增加扭腰效果
//void enhanceWaistTwisted(k4abt_skeleton_t* skeleton)
//{
//	// 有关的节点位置对象
//	k4a_float3_t& head_pos = skeleton->joints[K4ABT_JOINT_HEAD].position;
//	k4a_float3_t& spine_chest_pos = skeleton->joints[K4ABT_JOINT_SPINE_CHEST].position;
//	k4a_float3_t& spine_navel_pos = skeleton->joints[K4ABT_JOINT_SPINE_NAVEL].position;
//	k4a_float3_t& pelvis_pos = skeleton->joints[K4ABT_JOINT_PELVIS].position;
//	k4a_float3_t& hip_left_pos = skeleton->joints[K4ABT_JOINT_HIP_LEFT].position;
//	k4a_float3_t& hip_right_pos = skeleton->joints[K4ABT_JOINT_HIP_RIGHT].position;
//
//	// 计算一个单位向量和两个距离
//	Eigen::Vector3f head_to_chest_identity = getVector(head_pos, spine_chest_pos).Identity();
//	int chest_to_naval_norm = getVector(spine_chest_pos, spine_navel_pos).norm();
//	int chest_to_pelvis_norm = getVector(spine_chest_pos, pelvis_pos).norm();
//
//	// 计算新的坐标
//	Eigen::Vector3f new_naval_v = (Eigen::Vector3f)spine_chest_pos.v + chest_to_naval_norm * head_to_chest_identity;
//	Eigen::Vector3f new_pelvis_v = (Eigen::Vector3f)spine_chest_pos.v + chest_to_pelvis_norm * head_to_chest_identity;
//
//	Eigen::Vector3f pelvis_diff_v = new_pelvis_v - (Eigen::Vector3f)pelvis_pos.v;
//	Eigen::Vector3f new_hip_left_v = (Eigen::Vector3f)hip_left_pos.v + pelvis_diff_v;
//	Eigen::Vector3f new_hip_right_v = (Eigen::Vector3f)hip_right_pos.v + pelvis_diff_v;
//
//	// 覆盖旧坐标
//	setPos(spine_navel_pos, new_naval_v);
//	setPos(pelvis_pos, new_pelvis_v);
//	setPos(hip_left_pos, new_hip_left_v);
//	setPos(hip_right_pos, new_hip_right_v);
//}

// 四元数转换为旋转向量
void ws_tech::SingleKinect::quaternionToRotationVector(int node_id, Eigen::Quaterniond q, Eigen::Vector3f& result)
{
	using std::cout;
	using std::endl;

	//Eigen::Matrix3d m = q.matrix();
	//switch (node_id)
	//{
	//case K4ABT_JOINT_CLAVICLE_LEFT:
	//case K4ABT_JOINT_SHOULDER_LEFT:
	//case K4ABT_JOINT_ELBOW_LEFT:
	//	m = rotation_matrix_type1 * m * rotation_matrix_type1.inverse();
	//	//m = rotation_matrix_type1.inverse() * m * rotation_matrix_type1;
	//	break;
	//case K4ABT_JOINT_CLAVICLE_RIGHT:
	//case K4ABT_JOINT_SHOULDER_RIGHT:
	//case K4ABT_JOINT_ELBOW_RIGHT:
	//	m = rotation_matrix_type2 * m * rotation_matrix_type2.inverse();
	//	//m = rotation_matrix_type2.inverse() * m * rotation_matrix_type2;
	//	break;
	//default:
	//	break;
	//}
	//Eigen::AngleAxisd rotation_vector(m);
	Eigen::AngleAxisd rotation_vector(q);

	auto angle = rotation_vector.angle();
	auto axis = rotation_vector.axis().transpose();
	auto axis_mul_angle = axis * angle;
	//auto axis_mul_angle = -rotation_vector.axis().transpose();
	result[0] = axis_mul_angle[0];
	result[1] = axis_mul_angle[1];
	result[2] = axis_mul_angle[2];


	cout << "node_id: " << node_id << " xyzw: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << endl;
	cout << "axis: " << axis[0] << ", " << axis[1] << ", " << axis[2] << endl;
	cout << node_id << "'s angle: " << angle << endl;
	cout << "--------------------------------------------------------" << endl;
	//switch (node_id)
	//{
	//case K4ABT_JOINT_CLAVICLE_LEFT:
	//case K4ABT_JOINT_SHOULDER_LEFT:
	//case K4ABT_JOINT_ELBOW_LEFT:
	//	cout << "xyzw: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << endl;
	//	cout << node_id << "'s angle: " << angle << endl;
	//	//m = rotation_matrix_type1.inverse() * m * rotation_matrix_type1;
	//	break;
	//case K4ABT_JOINT_CLAVICLE_RIGHT:
	//case K4ABT_JOINT_SHOULDER_RIGHT:
	//case K4ABT_JOINT_ELBOW_RIGHT:
	//	cout << "xyzw: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << endl;
	//	cout << node_id << "'s angle: " << angle << endl;
	//	//m = rotation_matrix_type2.inverse() * m * rotation_matrix_type2;
	//	break;
	//case K4ABT_JOINT_PELVIS:
	//	cout << "xyzw: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << endl;
	//	cout << node_id << "'s angle: " << angle << endl;
	//	break;
	//default:
	//	break;
	//}


	//std::cout << typeid(axis_mul_angle).name() << std::endl;
	//std::cout << "rotation_vector " << "angle is: " << angle * (180 / M_PI)
	//<< " axis is: " << axis_mul_angle[0] << "," << axis_mul_angle[1] << "," << axis_mul_angle[2] << std::endl;
	//return axis_mul_angle;
}

// 只发送四元数的逻辑
void ws_tech::SingleKinect::sendOrientation()
{
	// OK, set strings data.
	std::stringstream ss;
	Eigen::Vector3f result{};

	auto wxyz = skeleton.joints[0].orientation.wxyz;
	// 注意 eigen Quaterniond 类四元数初始化参数顺序为 w,x,y,z
	Eigen::Quaterniond q_root = Eigen::Quaterniond(wxyz.w, wxyz.x, wxyz.y, wxyz.z);

	for (int i = 0; i < UDP_MAP_LEN; i++)
	{
		auto wxyz = skeleton.joints[udp_map[i]].orientation.wxyz;

		//Eigen::Quaterniond q = Eigen::Quaterniond(wxyz.w, wxyz.x, wxyz.y, wxyz.z);
		////q = q_root.inverse() * q;  // 深度相机坐标系->模型坐标系
		//quaternionToRotationVector(i, q, result);
		//ss << result[0] << " ";
		//ss << result[1] << " ";
		//ss << result[2] << ", ";

		ss << wxyz.x << " ";
		ss << wxyz.y << " ";
		ss << wxyz.z << " ";
		ss << wxyz.w << ", ";
	}

	Sleep(50);
	// Send results with UDP
	udp_sender.Send(ss.str());
	std::cout << "--------------------------------------------------------" << std::endl;
}

// W, X, Y, Z
std::string q0 = "1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0";
std::string q1 = "0.52578104,0.28398842,-0.045088585,-0.8005448,0.5062444,0.2865827,-0.060081635,-0.81115794,0.24238873,0.69895375,-0.22648762, -0.63357294, 0.47243986, 0.57754135, -0.17229918, -0.64308596, 0.52046514, 0.25757676, -0.04867726, -0.8126505, 0.5106523, 0.27476117, -0.06542535, -0.81207144, 0.532599, 0.31141913, -0.07930574, -0.782986, 0.5343164, 0.26835498, -0.04704112, -0.8001741, 0.06989027, 0.2383981, 0.28540525, -0.9256487, 0.38054848, -0.08494649, -0.4517442, 0.8024301, 0.5346426, 0.27560577, -0.049911756, -0.79731274, 0.036649574, 0.15141322, 0.25205135, -0.9550921, 0.40300727, 0.0440083, -0.4857432, 0.7744043, 0.52717006, 0.28184277, -0.051829185, -0.7999814, 0.24497518, 0.025500529, 0.2713471, -0.9304341, 0.13351221, 0.26122046, -0.42089185, 0.8583638";
std::string q2 = "0.62723374,0.6019349,-0.20041622,-0.4517583,0.6212098,0.5947515,-0.20709184,-0.4663497,0.009721586,0.9706215,-0.1542617,-0.18439822, 0.3409674, 0.9107854, -0.22118334, -0.07272677, 0.60376316, 0.61427534, -0.22167361, -0.45716143, 0.5553373, 0.58931535, -0.24513246, -0.5331211, 0.51615715, 0.59046257, -0.25414863, -0.5659896, 0.61219096, 0.61140656, -0.21986736, -0.4506248, 0.6235144, 0.6201091, -0.23345391, -0.41496232, 0.6246734, 0.6239254, -0.23934765, -0.40399635, 0.6251237, 0.5987243, -0.20583078, -0.45649007, 0.30259854, 0.5066047, 0.16483057, -0.79032695, 0.022381756, -0.25165242, -0.5253981, 0.81248194, 0.6266796, 0.60687363, -0.21399985, -0.43952376, 0.503815, 0.26809007, 0.24354699, -0.7842085, 0.26650393, 0.12468674, 0.39770716, -0.86905575";
std::string q3 = "0.69466394,0.34341168,-0.2726511,-0.57023835,0.6940851,0.34524855,-0.24561915,-0.581997,0.33753997,0.712582,-0.41457322,-0.45433763, 0.5235827, 0.5796062, -0.3262748, -0.5324121, 0.7016831, 0.3364952, -0.2618767, -0.57081723, 0.38592944, 0.47714022, -0.04179838, -0.78844696, 0.1477244, -0.48099247, -0.11851301, 0.85602474, 0.69998235, 0.34719247, -0.27117217, -0.5620925, 0.618571, 0.32333276, -0.27056387, -0.66303927, 0.6389377, 0.35745072, -0.2531228, -0.63238955, 0.6874362, 0.3455693, -0.29151696, -0.56835836, 0.6988539, 0.3574183, -0.32645875, -0.52657384, 0.72878385, 0.35058603, -0.36336628, -0.46252403, 0.69250596, 0.34236813, -0.28182796, -0.5690277,0.6867542, 0.36427283, -0.37219083, -0.5070975, 0.7101368, 0.40492564, -0.33196542, -0.47068015";
std::string q4 = "0.54243386,0.54288524,-0.38876468,-0.5098069,0.5282929,0.5700863,-0.38916767,-0.4944259,0.5687773,0.5828117,-0.3760004,-0.44209358, 0.60532665, 0.5150642, -0.36642218, -0.48375952, 0.5555987, 0.534747, -0.37399366, -0.5152519, 0.27683187, 0.6940386, -0.08785321, -0.6587536, 0.28548253, -0.5902222, -0.25389215, 0.7111093, 0.53932047, 0.54168516, -0.38726476, -0.51549643, 0.35222575, 0.5768496, 0.010030345, -0.73694026, 0.0643611, -0.4927622, -0.39079055, 0.774807, 0.5510162, 0.52404857, -0.3870758, -0.52146584, 0.2983025, 0.5217664, -0.019263607, -0.7990021, 0.0070285806, 0.34172133, 0.4258301, -0.8377624, 0.52404237, 0.5444992, -0.4071214, -0.5129838, 0.50012493, 0.51711416, -0.43222663, -0.5437354, 0.50690293, 0.5109519, -0.36370435, -0.59135157";
std::string q5 = "0.61365306,0.58119035,-0.33387318,-0.4173443,0.62565786,0.5560621,-0.326219,-0.439236,0.38080093,0.7585819,-0.37155035,-0.3761577, 0.5175188, 0.65186006, -0.36216328, -0.41963142, 0.6254131, 0.5660417, -0.32961532, -0.4240388, 0.5187623, 0.5657413, -0.3665619, -0.5257897, 0.51648253, 0.58466613, -0.42038578, -0.46334323, 0.6162857, 0.59250313, -0.31541118, -0.4118833, 0.5910981, 0.56006616, -0.37176767, -0.44577748, 0.6270303, 0.58700734, -0.37629429, -0.3473585, 0.6180801, 0.5812291, -0.32182488, -0.42021242, 0.6169684, 0.61067384, -0.3111763, -0.38677734, 0.631353, 0.63225716, -0.2968066, -0.33697197, 0.61032236, 0.5912333, -0.31563696, -0.42228308, 0.6087138, 0.5858875, -0.31928346, -0.42925704, 0.6208151, 0.59941566, -0.2963393, -0.40923396";


// 只发送位置坐标 XYZ 的逻辑
void ws_tech::SingleKinect::sendPosition()
{

	// Successfully get skeleton for the i-th person. Start processing.
	std::vector<k4abt_joint_t> skeleton_matrix;
	// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
	// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
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

	//-------------------------------------
	// 非常不高效，仅仅为了方便理解
	//-------------------------------------
	// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
	// 仅仅是坐标值，四元数等信息并没有经过调整
	for (int i = 0; i < UDP_MAP_LEN; i++) {
		skeleton.joints[udp_map[i]].position.xyz.x = tmp(0, udp_map[i]);
		skeleton.joints[udp_map[i]].position.xyz.y = tmp(1, udp_map[i]);
		skeleton.joints[udp_map[i]].position.xyz.z = tmp(2, udp_map[i]);
	}

	// OK, set strings data.
	std::stringstream ss;
	Eigen::Vector3f result{};

	// 传输顺序没有问题
	for (int i = 0; i < UDP_MAP_LEN; i++)
	{
		auto xyz = skeleton.joints[udp_map[i]].position.xyz;

		ss << xyz.x << ",";
		ss << xyz.y << ",";
		ss << xyz.z;
		if (i != UDP_MAP_LEN - 1)
		{
			ss << ",";
		}
	}

	ss << "," << q1 << "," << q2;

	//Sleep(50);
	// Send results with UDP
	udp_sender.Send(ss.str());
	//std::cout << ss.str() << std::endl;
	std::cout << "--------------------------------------------------------" << std::endl;
}

// 只发送位置坐标 XYZ 的逻辑
void ws_tech::SingleKinect::sendSMPLTheta()
{

	// Successfully get skeleton for the i-th person. Start processing.
	std::vector<k4abt_joint_t> skeleton_matrix;
	// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
	// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
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

	//-------------------------------------
	// 非常不高效，仅仅为了方便理解
	//-------------------------------------
	// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
	// 仅仅是坐标值，四元数等信息并没有经过调整
	for (int i = 0; i < UDP_MAP_LEN; i++) {
		skeleton.joints[udp_map[i]].position.xyz.x = tmp(0, udp_map[i]);
		skeleton.joints[udp_map[i]].position.xyz.y = tmp(1, udp_map[i]);
		skeleton.joints[udp_map[i]].position.xyz.z = tmp(2, udp_map[i]);
	}

	// OK, set strings data.
	std::stringstream ss;
	Eigen::Vector3f result{};

	// 传输顺序没有问题
	for (int i = 0; i < UDP_MAP_LEN; i++)
	{
		auto xyz = skeleton.joints[udp_map[i]].position.xyz;

		ss << xyz.x << ",";
		ss << xyz.y << ",";
		ss << xyz.z;
		if (i != UDP_MAP_LEN - 1)
		{
			ss << ",";
		}
	}

	ss << "," << q1 << "," << q2;

	//Sleep(50);
	// Send results with UDP
	udp_sender.Send(ss.str());
	//std::cout << ss.str() << std::endl;
	std::cout << "--------------------------------------------------------" << std::endl;
}

// 发送提示没有检测到人的信号
void ws_tech::SingleKinect::sendNULL()
{
	// OK, set strings data.
	std::stringstream ss;
	ss << "0";
	// Send results with UDP
	udp_sender.Send(ss.str());
	std::cout << "--------------------------------------------------------" << std::endl;
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
			verify(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame succeed.", "Get body from body frame failed!");
			body.id = k4abt_frame_get_body_id(body_frame, i);

			print_body_information(body);
		}
	}

	if (num_bodies <= 0)
	{
		sendNULL();
		return;  // 如果检测不到人，则什么也不做
	}

	k4a_result_t skeleton_result = k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

	if (skeleton_result == K4A_RESULT_SUCCEEDED)
	{
		//sendOrientation();
		//sendPosition();
		sendSMPLTheta();  // 使用 IK 根据 position --> 计算 theta


		//// Successfully get skeleton for the i-th person. Start processing.
		//std::vector<k4abt_joint_t> skeleton_matrix;
		//// 把 BodyTracking API 算出的关节点数据放到了 skeleton_matrix 中
		//// skeleton_matrix 是 vector<k4abt_joint_t> 类型的成员变量
		//for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		//	skeleton_matrix.push_back(skeleton.joints[i]);
		//}

		//// 创建一个临时变量 tmp 用于保存关节点数据
		//// 把 skeleton_matrix 的节点 3D 位置赋值给 tmp
		//// 这里把 tmp(i, j) 中的 i 作为了 X、Y、Z，未来可能需要做优化
		//Eigen::Matrix3Xf tmp;
		//tmp.resize(3, skeleton_matrix.size());
		//for (int i = 0; i < skeleton_matrix.size(); i++) {
		//	tmp(0, i) = skeleton_matrix.at(i).position.xyz.x;
		//	tmp(1, i) = skeleton_matrix.at(i).position.xyz.y;
		//	tmp(2, i) = skeleton_matrix.at(i).position.xyz.z;
		//}

		//// 使用 TrackerProcessor 成员变量旋转矩阵对 tmp 进行旋转
		//Eigen::Matrix3f view_rotation;
		//view_rotation << 1, 0, 0, 0, -0.1736, 0.9848, 0, -0.9848, -0.1736;
		//tmp = view_rotation * tmp;

		//// 计算平移量
		//// mHasShift 相当于开关，保证计算平移量的逻辑只进行一次。
		//if (!has_shifted) {
		//	float sumX = 0;  // 用于保存所有节点的 X 值的和
		//	float sumY = 0;  // 用于保存所有节点的 Y 值的和
		//	float minZ = 9999;  // 用于保存所有节点的 Z 值得最小值
		//	for (int i = 0; i < tmp.cols(); i++) {
		//		sumX += tmp(0, i);
		//		sumY += tmp(1, i);

		//		if (minZ > tmp(2, i)) {
		//			minZ = tmp(2, i);
		//		}
		//	}
		//	shift_vector << -sumX / skeleton_matrix.size(), -sumY / skeleton_matrix.size(), -minZ;  // 该平移矩阵是：-X平均值、-Y平均值、-Z最小值
		//	// 关闭开关，该代码块的逻辑不会再调用
		//	has_shifted = true;
		//}

		//// 依次给每个列向量加上位移量
		//// 可优化
		//for (int i = 0; i < tmp.cols(); i++) {
		//	tmp(0, i) += shift_vector(0);
		//	tmp(1, i) += shift_vector(1);
		//	tmp(2, i) += shift_vector(2);
		//}

		//// 用已经进行了旋转和平移的临时变量 tmp 对 skeleton_matrix 进行重新赋值
		//// 仅仅是坐标值，四元数等信息并没有经过调整
		//for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
		//	skeleton.joints[i].position.xyz.x = tmp(0, i);
		//	skeleton.joints[i].position.xyz.y = tmp(1, i);
		//	skeleton.joints[i].position.xyz.z = tmp(2, i);
		//}

		//// Am I really need this???
		//// enhanceWaistTwisted(&skeleton);

		// OK, set strings data.
		//std::stringstream ss;
		//for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		//{
		//	k4a_float3_t position = skeleton.joints[i].position;
		//	ss << position.xyz.x << " " << position.xyz.y << " " << position.xyz.z << ", ";
		//}
		//ss << "| ";
		//for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		//{
		//	k4a_quaternion_t orientation = skeleton.joints[i].orientation;
		//	//k4a_quaternion_t orientation = skeleton_matrix[i].orientation;
		//	//ss << orientation.wxyz.w << " " << orientation.wxyz.x << " " << orientation.wxyz.y << " " << orientation.wxyz.z << ", ";
		//	Eigen::Quaternionf result = Eigen::Quaternionf(view_rotation) * Eigen::Quaternionf(orientation.wxyz.w, orientation.wxyz.x, orientation.wxyz.y, orientation.wxyz.z);
		//	ss << result.coeffs()(3, 0) << " ";
		//	ss << result.coeffs()(0, 0) << " ";
		//	ss << result.coeffs()(1, 0) << " ";
		//	ss << result.coeffs()(2, 0) << ", ";
		//}
		//ss << " | 0,";

		//// Send results with UDP
		//udp_sender.Send(ss.str());
	}
	else if (skeleton_result == K4A_RESULT_FAILED)
	{
		std::cout << "Error! Get body skeleton failed!" << std::endl;
	}

	k4abt_frame_release(body_frame);
}
