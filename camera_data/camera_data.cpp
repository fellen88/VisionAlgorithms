// camera_data.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "camera_data.h"

void CameraData::test()
{
	int a = 1;
	int b = 0;
	int c = a / b;
}

ICameraData* GetCameraData()
{
	ICameraData* p_iregistration = new CameraData();
	return p_iregistration;
}
