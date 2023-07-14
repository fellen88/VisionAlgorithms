#ifndef DETECTION_2D_H_
#define DETECTION_2D_H_

#include "idetection_2d.h"
#include "../camera_data/camera_data.h"

class Detection2D : public IDetection2D {

public:
  Detection2D(std::string config);
	virtual ~Detection2D();

	virtual void UpdateParameters(std::string config) override;
	virtual void SetInputImage(const cv::Mat& image) override;
	virtual void Detect() override;
  virtual void GetMask(std::vector<cv::Mat>& mask) override;
  virtual void GetLabel(std::vector<int>& label) override;

private:
  std::string path_image;
  std::string path_model;
  std::string flag_device;
  std::string path_output;
  cv::Mat image_scene;

  mmdeploy::Detector* detector;
	std::shared_ptr<ICameraData> p_sensor_;
  std::string project_file;
  std::vector<int> dets_label;
  std::vector<cv::Mat> dets_mask;

  bool debug_visualization;
  bool sensor_offline;
  int output_mask_number;
  int output_mask_count;
  float score_threshold;
};
#endif // DETECTION_2D_H_
