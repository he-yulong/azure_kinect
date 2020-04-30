#pragma once

// Kinematic model that takes in model parameters and outputs mesh, keypoints, etc.
class KinematicModel
{
public:

	// scale: Scale of the model to make the solving easier, by default 1
	KinematicModel(int scale = 1);
};

