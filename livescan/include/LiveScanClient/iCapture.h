//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#pragma once

#include "utils.h"
#include <k4abttypes.h>

typedef k4abt_joint_t Joint;
//struct Joint
//{
//
//};
#ifndef _Joint_
#define _Joint_
typedef struct _Joint
{
	JointType JointType;
	CameraSpacePoint Position;
	TrackingState TrackingState;
} 	Joint;

#endif // _Joint_
typedef enum _JointType JointType;
#ifndef _CameraSpacePoint_
#define _CameraSpacePoint_
typedef struct _CameraSpacePoint
{
	float X;
	float Y;
	float Z;
} 	CameraSpacePoint;

#endif // _CameraSpacePoint_

#ifndef _TrackingState_
#define _TrackingState_
typedef enum _TrackingState TrackingState;


enum _TrackingState
{
	TrackingState_NotTracked = 0,
	TrackingState_Inferred = 1,
	TrackingState_Tracked = 2
};
#endif // _TrackingState_

enum _JointType
{
	JointType_SpineBase = 0,
	JointType_SpineMid = 1,
	JointType_Neck = 2,
	JointType_Head = 3,
	JointType_ShoulderLeft = 4,
	JointType_ElbowLeft = 5,
	JointType_WristLeft = 6,
	JointType_HandLeft = 7,
	JointType_ShoulderRight = 8,
	JointType_ElbowRight = 9,
	JointType_WristRight = 10,
	JointType_HandRight = 11,
	JointType_HipLeft = 12,
	JointType_KneeLeft = 13,
	JointType_AnkleLeft = 14,
	JointType_FootLeft = 15,
	JointType_HipRight = 16,
	JointType_KneeRight = 17,
	JointType_AnkleRight = 18,
	JointType_FootRight = 19,
	JointType_SpineShoulder = 20,
	JointType_HandTipLeft = 21,
	JointType_ThumbLeft = 22,
	JointType_HandTipRight = 23,
	JointType_ThumbRight = 24,
	JointType_Count = (JointType_ThumbRight + 1)
};
#endif // _JointType_

struct Body
{
	bool bTracked;
	std::vector<Joint> vJoints;
	std::vector<Point2f> vJointsInColorSpace;

	Body()
	{
		bTracked = false;
		vJoints.resize(5);
		vJointsInColorSpace.resize(5);
	}
};

class ICapture
{
public:
	ICapture();
	~ICapture();

	virtual bool Initialize() = 0;
	virtual bool AcquireFrame() = 0;
	virtual void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints) = 0;
	virtual void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints) = 0;
	virtual void MapDepthFrameToColorSpace(UINT16 *pColorSpacePoints) = 0;
	virtual void MapColorFrameToDepthSpace(RGB *pDepthSpacePoints) = 0;

	bool bInitialized;

	int nColorFrameHeight, nColorFrameWidth;
	int nDepthFrameHeight, nDepthFrameWidth;

	UINT16 *pDepth;
	BYTE *pBodyIndex;
	RGB *pColorRGBX;
	std::vector<Body> vBodies;

	std::string serialNumber;
};