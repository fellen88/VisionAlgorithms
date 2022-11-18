// AccuracyGrasp.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "pch.h"
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
	//位姿估计算法使用变量定义
	std::vector<double> object_points;
	std::vector<double> object_pose;
	unsigned char view_point = 0;

	for (size_t i = 0; i < 60000; i+=3)
	{
		object_points.push_back(100*std::rand()/RAND_MAX);
		object_points.push_back(100*std::rand()/RAND_MAX);
		object_points.push_back(100*std::rand()/RAND_MAX);
	}

	//3D视觉算法调用:初始化实例
	IPoseEstimation *pose_estimation_ = GetInstance('A');

	while (true)
	{
		char input;
		std::cout << "input 's' to start / input 'e' to end :" << std::endl;
		std::cin >> input;
		if (input == 's')
		{
			//采集拍照点1 目标场景点云
			view_point = 1;
			//3D视觉算法调用:计算目标位姿
			pose_estimation_->Algorithm_A(object_points, view_point, object_pose);
		}
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

