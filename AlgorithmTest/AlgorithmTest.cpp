#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>

#include "../pose_estimation/ipose_estimation.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_pose_estimation.lib")
#else
#pragma comment (lib, "../X64/Release/vision_pose_estimation.lib")
#endif

int main()
{
	//3D�Ӿ��㷨�������������ʼ��
	pcl::PointCloud<pcl::PointXYZRGBNormal> object_points;
	std::vector<double> object_pose;
	unsigned char view_point = 0;

	//3D�Ӿ��㷨:��ȡʵ��ָ��
	std::string project_name = "BinPicking_BYD";
	std::string object_number = "1";

	std::string config_object_x = ".\\" + project_name + "\\Config_" + object_number + "\\bin_picking.json";
	std::shared_ptr<val::IPoseEstimation> p_object_x_(GetInstance(val::IPoseEstimation::BinPicking, config_object_x));

	//���ز��Ե�������
	pcl::io::loadPLYFile(".\\" + project_name + "\\PointCloud\\test_1.ply", object_points);
	char input;

	while (true)
	{
		std::cout << "input 's' to start /'r' to reset parameters / 'e' to end :" << std::endl;
		std::cin >> input;

		if (input == 's')
		{
			//�ɼ����յ�1 Ŀ�곡������
			view_point = 1;
			//3D�Ӿ��㷨:����Ŀ��λ��
			p_object_x_->Compute(object_points, view_point, object_pose);
			//���ץȡĿ��λ�˼�����
			for (auto it = object_pose.begin(); it != object_pose.end(); ++it) {
				std::cout << *it << " ";
			}
			std::cout << std::endl;
		}
		else if (input == 'r')
			p_object_x_.reset(GetInstance(val::IPoseEstimation::BinPicking, config_object_x));
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

