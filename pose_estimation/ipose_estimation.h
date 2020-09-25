#ifndef IPOSEESTIMATIONTION_H
#define IPOSEESTIMATIONTION_H

class IPoseEstimation
{
	public:
	virtual bool GetTransformation() = 0;
};

extern "C" __declspec(dllexport) IPoseEstimation* GetPoseEstimation();

#endif