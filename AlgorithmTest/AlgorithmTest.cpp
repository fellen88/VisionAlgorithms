// AlgorithmTest.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <vector>

#include "../pose_estimation/ipose_estimation.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/grasp_pose_estimation.lib")
#else
#pragma comment (lib, "../X64/Release/grasp_pose_estimation.lib")
#endif

int main()
{
	//λ�˹����㷨ʹ�ñ�������
	std::vector<double> object_points;
	std::vector<double> object_pose;
	unsigned char view_point = 0;

	for (size_t i = 0; i < 60000; i+=3)
	{
		object_points.push_back(100*std::rand()/RAND_MAX);
		object_points.push_back(100*std::rand()/RAND_MAX);
		object_points.push_back(100*std::rand()/RAND_MAX);
	}

	//3D�Ӿ��㷨����:��ʼ��ʵ��
	IPoseEstimation *pose_estimation_ = GetInstance('B');

	while (true)
	{
		char input;
		std::cout << "input 's' to start / input 'e' to end :" << std::endl;
		std::cin >> input;
		if (input == 's')
		{
			//�ɼ����յ�1 Ŀ�곡������
			view_point = 1;
			//3D�Ӿ��㷨����:����Ŀ��λ��
			pose_estimation_->Algorithm_B(object_points, view_point, object_pose);
		}
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
	delete pose_estimation_;
}

