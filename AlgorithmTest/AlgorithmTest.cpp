// AlgorithmTest.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>

#include "../pose_estimation/ipose_estimation.h"
#pragma comment (lib, "grasp_pose_estimation.lib")

int main()
{
	//λ�˹����㷨ʹ�ñ�������
	PointCloud::Ptr object_pointcloud(new PointCloud);
	Eigen::Matrix4f object_pose = Eigen::Matrix4f::Zero();
	unsigned char view_point = 0;
  
	//��ʼ��Ŀ��λ��
	object_pose(0, 1) = 1.0;
	object_pose(1, 0) = 1.0;
	object_pose(2, 2) = 1.0;
	object_pose(3, 3) = 1.0;

	//3D�Ӿ��㷨����:��ʼ��ʵ��
	IPoseEstimation *pose_estimation_ = GetInstance('A');

	/*�ɼ����յ�1 Ŀ�곡������*/
	view_point = 1;
	//object_pointcloud = ���յ�1����;

	//3D�Ӿ��㷨����:����Ŀ��λ��
	pose_estimation_->Algorithm_A(object_pointcloud, view_point, object_pose);

	/*�������˶��ӽ�*/

	/*�ɼ����յ�2 Ŀ�곡������*/
	view_point = 2;
  //object_pointcloud = ���յ�2����;

	//3D�Ӿ��㷨����:����Ŀ��λ��
	pose_estimation_->Algorithm_A(object_pointcloud, view_point, object_pose);

	/*�������˶�ץȡ*/

}
