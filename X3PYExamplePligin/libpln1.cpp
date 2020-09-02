#include "stdafx.h"
#include <module/plugininc.h>
#include "libpln1.h"

#include "../pose_estimation/ipose_estimation.h"
#pragma comment (lib, "grasp_pose_estimation.lib")


int CSimpleA::disPatchChar(BYTE ucType, int& iInstanID, std::string strInData, std::string & strOutData, lpCALLBACK pCallback, LPVOID lpPara)   const
{
	std::string strdata;
	iInstanID = 0;

	switch (ucType)
	{
	case 0:
		{
		IPoseEstimation *pose_estimation_ = GetPoseEstimation();
		strOutData = pose_estimation_->GetTransformation(strInData);
		break;
		}
	case 1:
		strOutData = "case 1";
		break;
	case 2:
		strOutData = "case 2";
		break;
	default:
		break;
	}
	return 0;
}

CSimpleA::CSimpleA()
{
}

CSimpleA::~CSimpleA()
{
}

int CSimpleA::add(int a, int b) const
{
    return a + b ;
}

int CSimpleA::subtract(int a, int b) const
{
    return a - b;
}

int CSimpleA::CtrlState(int iState) const
{
	return 0;
}
//int CSimpleA::disPatchInt(int iCnt, const int* piData, int iInCnt, int* piOutData = 0, int iOutData = 0) const
//{
//	return 0;
//}
//int CSimpleA::disPatchFloat(int iCnt, const float* pfData, int iInCnt, float* pfOutData = 0, int iOutData = 0)  const
//{
//	return 0;
//}