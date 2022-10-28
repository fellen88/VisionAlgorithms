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
