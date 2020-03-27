#pragma once
#include <iostream>

#include <k4a/k4a.h>
#include <k4abt.h>

#include <vector>
#include <Eigen/Dense>

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "FloorDetector.h"
#include "Utilities.h"
#include "PointCloudGenerator.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include "MatWrapper.h"
#include "UDPSender.h"


namespace ws_tech
{
	class SingleKinect
	{
	public:
		SingleKinect(py::function process_emotion_val, int device_index_val = 0, std::string ip_addr="127.0.0.1", int udp_port=8999);
		void Open();
		void Running(int max_frame = 200);
		void Close();

	private:
		const bool is_debug = false;  // 用于测试
		bool has_shifted = false;  // 用于视角平移
		Eigen::Vector3f shift_vector;  // 用于视角平移
		py::function process_emotion;
		int device_index;
		k4a_device_t device;
		k4a_device_configuration_t device_config;
		k4a_calibration_t sensor_calibration;
		k4a_capture_t sensor_capture;  // capture_handle: If successful this contains a handle to a capture object. Caller must call k4a_capture_release() when its done using this capture.
		k4a_image_t color_image;
		k4a_image_t depth_image;
		Samples::FloorDetector floor_detector;

		// Body Tracking 相关
		k4abt_tracker_t tracker;
		k4abt_tracker_configuration_t tracker_config;
		k4abt_frame_t body_frame;
		k4abt_skeleton_t skeleton;

		// UDP 发送器
		UDPSender udp_sender;

		// 内联成员函数：用于验证
		// 存在 C26812 的小问题
		void verify(const k4a_result_t& result, const std::string& msg, const std::string& error)
		{
			if (result != K4A_RESULT_SUCCEEDED)
			{
				std::cout << error << std::endl;
				std::cout << " - (File: " << __FILE__ << ", Function: " << __FUNCTION__ << ", Line: " << __LINE__ << std::endl;
				exit(1);
			}
			std::cout << msg << std::endl;
		};

		void updateEmotionResult();

		bool updateBodyTrackingResult();

		void processBodyFrame();

		void updateFloorResult(Samples::PointCloudGenerator& point_cloud_generator);

	};
}