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
	//λ�˹����㷨ʹ�ñ�������
	pcl::PointCloud<pcl::PointXYZRGBNormal> object_points;
	std::vector<double> object_pose;
	unsigned char view_point = 0;

	//3D�Ӿ��㷨����:��ʼ��ʵ��
	//std::string project_name = "BinPicking";
	std::string project_name = "BinPicking_BYD";
	//std::string project_name = "BinPicking_MAHLE";

	std::string config_byd_1 = ".\\" + project_name + "\\Config_1\\bin_picking.json";

	std::shared_ptr<val::IPoseEstimation> p_byd_1_(GetInstance(val::IPoseEstimation::BinPicking, config_byd_1));

	//���ز��Ե�������
	pcl::io::loadPLYFile(project_name + "\\PointCloud\\test_1.ply", object_points);
	char input;

	while (true)
	{
		std::cout << "input 's' to start /'r' to reset parameters / 'e' to end :" << std::endl;
		std::cin >> input;
		if (input == 's')
		{
			//�ɼ����յ�1 Ŀ�곡������
			view_point = 1;
			//3D�Ӿ��㷨����:����Ŀ��λ��
			p_byd_1_->Compute(object_points, view_point, object_pose);
		}
		else if (input == 'r')
			p_byd_1_.reset(GetInstance(val::IPoseEstimation::BinPicking, config_byd_1));
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

