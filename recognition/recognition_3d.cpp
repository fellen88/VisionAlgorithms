// recognition.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include"recognition_3d.h"

Recognition3D::Recognition3D()
{
}

Recognition3D::~Recognition3D()
{
}

bool Recognition3D::Compute()
{
	return false;
}

IRecognition* GetRecognition3D()
{
	IRecognition* p_irecognition = new Recognition3D();
	return p_irecognition;
}
