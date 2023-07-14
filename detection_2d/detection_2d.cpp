#include "pch.h"
#include "mmdeploy/detector.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "utils/argparse.h"
#include "utils/visualize.h"
#include "detection_2d.h"

Detection2D::Detection2D(std::string config):
	p_sensor_(GetCameraData())
{
	p_sensor_->GetSubPath(config, project_file, 4);
  path_model = project_file + "\\object_instances\\model_dl";
  flag_device = "cuda";
  detector = new mmdeploy::Detector(mmdeploy::Model{ path_model }, mmdeploy::Device{ flag_device });
  UpdateParameters(config);
}

Detection2D::~Detection2D()
{

}

void Detection2D::UpdateParameters(std::string config)
{
	std::string JsonFileName = config;
	JsonOutType json_reader;
	p_sensor_->GetSubPath(config, project_file, 4);
	LOG(INFO) << "********************************************";
	LOG(INFO) << "project name: " << project_file;
	LOG(INFO) << "deep learning: " << "object_instances";
	LOG(INFO) << "********************************************";
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Visualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "CameraOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "OutputMaskNumber", "int");
	if (json_reader.success)
    output_mask_number = json_reader.json_int;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "ScoreThreshold", "float");
	if (json_reader.success)
    score_threshold = json_reader.json_float;
}

void Detection2D::SetInputImage(const cv::Mat& image)
{
  if (sensor_offline)
  {
    path_image = project_file + "\\object_instances\\image\\demo.jpg"; image_scene = cv::imread(path_image);
		if (image_scene.empty()) {
			fprintf(stderr, "failed to load image: %s\n", path_image.c_str());
			return;
		}
  }
  else
  {
    image_scene = image;
  }
 }

void Detection2D::GetMask(std::vector<cv::Mat>& mask)
{
  mask.clear();
  for (int i = 0; i < output_mask_count; i++)
  {
		mask.push_back(dets_mask[i]);
    if (debug_visualization)
    {
			cv::imshow("output_mask", mask[i]);
			cv::waitKey(0);
    }
	}
}

void Detection2D::GetLabel(std::vector<int>& label)
{
  label.clear();
  for (int i = 0; i < output_mask_count; i++)
  {
		label.push_back(dets_label[i]);
	}
}

void Detection2D::Detect()
{
  // apply the detector, the result is an array-like class holding references to
  // `mmdeploy_detection_t`, will be released automatically on destruction
  mmdeploy::Detector::Result dets = detector->Apply(image_scene);
  output_mask_count = 0;
  for (const mmdeploy_detection_t& det : dets)
  {
    if (det.score > score_threshold)
    {
			//fprintf(stdout, "mask %d, height=%d, width=%d\n", index, mask->height, mask->width);
			auto x0 = (int)std::max(std::floor(det.bbox.left) - 1, 0.f);
			auto y0 = (int)std::max(std::floor(det.bbox.top) - 1, 0.f);
			utils::Visualize v;
			auto output = v.get_session(image_scene);
			output.add_instance_mask({ x0, y0 }, 250, det.mask->data, det.mask->height, det.mask->width, 1.0);
			dets_mask.push_back(output.get());
      dets_label.push_back(det.label_id);
		
			output_mask_count++;
		}
    if (output_mask_count >= output_mask_number)
    {
      LOG(INFO)<< "output mask number: " << output_mask_count;
			break;
		}
	}

  if (debug_visualization)
  {
    utils::Visualize v;
    auto sess = v.get_session(image_scene);
    int count = 0;
    for (const mmdeploy_detection_t& det : dets) {
      if (det.score > score_threshold) {  // filter bboxes
        sess.add_det(det.bbox, det.label_id, det.score, det.mask, count++);
      }
    }
    cv::imshow("detection", sess.get());
    cv::waitKey(0);
    // save the result
   /* path_output =project_file + "\\object_instances\\image\\detector_output.jpg";
    if (!path_output.empty()) {
      cv::imwrite(path_output, sess.get());
    }*/
  }
}

IDetection2D* GetDetection2DPtr(std::string config)
{
  return new Detection2D(config);
}
