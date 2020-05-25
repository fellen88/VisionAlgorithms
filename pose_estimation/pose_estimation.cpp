// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string PoseEstimation::Compute()
{
	//if (p_registration_->SetParameters())
	{
		//if (p_registration_->RecieveImage() && p_registration_->Load3DModel(object_model))
		{
			//p_registration_->Load3DModel(object_model, "test.pcd");
			//p_registration_->Show(object_model, "object model");
		}
	}
		return "pose_estimation node test ok!";
}

__DLL_INTERFACE PoseEstimation * GetInstance()
{
	//LOG(INFO) << "GetInstance()";
  PoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  //p_pose_estimation_->Init();
  return p_pose_estimation_;
}