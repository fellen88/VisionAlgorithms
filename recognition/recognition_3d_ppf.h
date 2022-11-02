#ifndef RECOGNITION_3D_PPF_H
#define RECOGNITION_3D_PPF_H

#include "recognition_3d.h"
#include "../camera_data/camera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/grasp_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/grasp_camera_data.lib")
#endif

class Recognition3DPPF : public Recognition3D
{
public:
	ICameraData* p_dataprocess_;

	Recognition3DPPF();
	~Recognition3DPPF();

	bool Compute(int a);

};

#endif

#pragma once
