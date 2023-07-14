#ifndef I_DETECTION_2D_H_
#define I_DETECTION_2D_H_

#include <opencv2/core/core.hpp>

#ifdef DETECTION2D_EXPORTS
#define DETECTION2D_API __declspec(dllexport)
#else
#define DETECTION2D_API __declspec(dllimport)
#endif


class IDetection2D {

public:
	virtual ~IDetection2D() = default;
	virtual void UpdateParameters(std::string config) = 0;
	virtual void SetInputImage(const cv::Mat &image) = 0;
	virtual void Detect() = 0;
	virtual void GetMask(std::vector<cv::Mat> &mask) = 0;
	virtual void GetLabel(std::vector<int> &label) = 0;

private:

};

DETECTION2D_API IDetection2D * GetDetection2DPtr(std::string config);

#endif // I_DETECTION_2D_H_