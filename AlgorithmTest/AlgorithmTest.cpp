// AlgorithmTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "../pose_estimation/ipose_estimation.h"
#pragma comment (lib, "grasp_pose_estimation.lib")

int main()
{
		IPoseEstimation *pose_estimation_ = GetPoseEstimation();
	   pose_estimation_->GetTransformation("");
		 system("pause");
}
