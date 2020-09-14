#ifndef IPOSEESTIMATIONTION_H
#define IPOSEESTIMATIONTION_H

class IPoseEstimation
{
	public:
	virtual std::string GetTransformation(std::string parameters) = 0;
};

extern "C" __declspec(dllexport) IPoseEstimation* GetPoseEstimation();

#endif