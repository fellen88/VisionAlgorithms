#pragma once
class ICameraData
{
	public:
	virtual void test() = 0;
};

extern "C" __declspec(dllexport) ICameraData* APIENTRY GetCameraData();
