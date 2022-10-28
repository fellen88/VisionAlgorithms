// AlgorithmTest.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>

#include "../pose_estimation/ipose_estimation.h"
#pragma comment (lib, "grasp_pose_estimation.lib")

int main()
{
	//位姿估计算法使用变量定义
	PointCloud::Ptr object_pointcloud(new PointCloud);
	Eigen::Matrix4f object_pose = Eigen::Matrix4f::Zero();
	unsigned char view_point = 0;

	//初始化目标位姿
	object_pose(0, 1) = 1.0;
	object_pose(1, 0) = 1.0;
	object_pose(2, 2) = 1.0;
	object_pose(3, 3) = 1.0;

	//3D视觉算法调用:初始化实例
	IPoseEstimation *pose_estimation_ = GetInstance('A');

	/*采集拍照点1 目标场景点云*/
	view_point = 1;
	//object_pointcloud = 拍照点1点云;

	//3D视觉算法调用:计算目标位姿
	pose_estimation_->Algorithm_A(object_pointcloud, view_point, object_pose);

	/*机器人运动接近*/

	/*采集拍照点2 目标场景点云*/
	view_point = 2;
	//object_pointcloud = 拍照点2点云;

	//3D视觉算法调用:计算目标位姿
	pose_estimation_->Algorithm_A(object_pointcloud, view_point, object_pose);

	/*机器人运动抓取*/

}
// AccuracyGrasp.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>

int main()
{
    std::cout << "Hello World!\n"; 
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
