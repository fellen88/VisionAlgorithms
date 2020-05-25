#include "stdafx.h"
#include <module/plugininc.h>
#include "libpln1.h"
#include "../pose_estimation/pose_estimation.h"
#pragma comment (lib, "../x64/debug/plugins/grasp_pose_estimation.lib")

PoseEstimation *pose_estimation_ = GetInstance();

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

//int CSimpleA::disPatch(int iCnt) const
//{
//	return 0;
//}
int CSimpleA::disPatchChar(BYTE ucType, int& iInstanID, std::string strInData, std::string & strOutData, lpCALLBACK pCallback, LPVOID lpPara)   const
{
	int a = 0;
	int b = 1;
	std::string strdata;
	iInstanID = 0;
	switch (ucType)
	{
	case 0:
		strOutData = pose_estimation_->Compute();
		(*pCallback)(&strOutData, 12, lpPara);
		break;
	case 1:
		strOutData = "case 1";
		break;
	case 2:
		strOutData = "case 2";
		break;
	case 3:
		strOutData = "case 3";
		break;
	default:
		break;
	}
	return 0;
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