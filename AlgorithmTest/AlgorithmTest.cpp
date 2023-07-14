#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>

#include "../grasp_pose/grasp_pose.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_grasp_pose.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_grasp_pose.lib")
#endif

#include "../detection_2d/idetection_2d.h"
#pragma comment (lib, "../X64/Release/gpd_detection_2d.lib")

int main()
{
	//3D�Ӿ��㷨�������������ʼ��
	pcl::PointCloud<pcl::PointXYZRGBNormal> object_points;
	std::vector<double> object_pose;
	cv::Mat image;
  std::vector<cv::Mat> object_mask;
	std::vector<int> object_label;

	//std::string project_name  = "GPD";
	//std::string project_name  = "GPD_BYD";
	std::string project_name  = "GPD_DY";

	//3D�Ӿ��㷨:��ȡʵ��ָ��
	std::string object_number = "object_x";
	std::string config_object_x = ".\\" + project_name +"\\"+ object_number + "\\config" + "\\grasp_pose.json";
	std::shared_ptr<gpd::GraspPose> p_object_x_(GetModelBasedPtr(config_object_x));
	//���ز��Ե�������
	pcl::io::loadPLYFile(".\\" + project_name +"\\"+ object_number  + "\\pointcloud\\scene.ply", object_points);
	
	//2D�Ӿ��㷨:��ȡʵ��ָ��
	std::string config_object_instances = ".\\" + project_name +"\\" + "object_instances\\config" + "\\instance_seg.json";
	IDetection2D *p_detection_2d = GetDetection2DPtr(config_object_instances);

	while (true)
	{
		char input;
		std::cout << "input 's' to start /'r' to reset parameters / 'e' to end :" << std::endl;
		std::cin >> input;

		if (input == 's')
		{
			//2D�Ӿ��㷨:���ץȡĿ��
			p_detection_2d->SetInputImage(image);
			p_detection_2d->Detect();
			p_detection_2d->GetMask(object_mask);
			p_detection_2d->GetLabel(object_label);

			//3D�Ӿ��㷨:����Ŀ��λ��
			for (int i = 0; i < object_mask.size(); i++)
			{
				p_object_x_->SetInputMask(object_mask[i]);
				p_object_x_->SetInputPointCloud(object_points);
				p_object_x_->GetGraspPose(&object_pose);
				//���ץȡĿ��λ�˼�����
				for (auto it = object_pose.begin(); it != object_pose.end(); ++it)
				{
					std::cout << *it << " ";
				}
				std::cout << std::endl;
			}
		}
		else if (input == 'r')
			//���²���
			std::shared_ptr<gpd::GraspPose> p_object_x_(GetModelBasedPtr(config_object_x));
		else if (input == 'e')
			//�˳�����
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

