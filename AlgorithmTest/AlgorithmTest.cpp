#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>

#include "../grasp_pose/grasp_pose.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_grasp_pose.lib")
#else
#pragma comment (lib, "../X64/Release/vision_grasp_pose.lib")
#endif

int main()
{
	//3D�Ӿ��㷨�������������ʼ��
	pcl::PointCloud<pcl::PointXYZRGBNormal> object_points;
	std::vector<double> object_pose;

	//3D�Ӿ��㷨:��ȡʵ��ָ��
	std::string project_name = "BinPicking_BYD";
	std::string object_number = "1";

	std::string config_object_x = ".\\" + project_name + "\\Config_" + object_number + "\\bin_picking.json";
	std::shared_ptr<val::GraspPose> p_object_x_(GetModelBasedPtr(config_object_x));

	//���ز��Ե�������
	pcl::io::loadPLYFile(".\\" + project_name + "\\PointCloud\\test_" + object_number + ".ply", object_points);
	char input;

	while (true)
	{
		std::cout << "input 's' to start /'r' to reset parameters / 'e' to end :" << std::endl;
		std::cin >> input;

		if (input == 's')
		{
			//3D�Ӿ��㷨:����Ŀ��λ��
			p_object_x_->SetInputPointCloud(object_points);
			p_object_x_->GetGraspPose(&object_pose);
			//���ץȡĿ��λ�˼�����
			for (auto it = object_pose.begin(); it != object_pose.end(); ++it) {
				std::cout << *it << " ";
			}
			std::cout << std::endl;
		}
		else if (input == 'r')
			//���²���
			std::shared_ptr<val::GraspPose> p_object_x_(GetModelBasedPtr(config_object_x));
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

