// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "stdafx.h"

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
		LOG(INFO) << "registration_3d process attach ";
		break;
    case DLL_THREAD_ATTACH:
		//LOG(INFO) << "registration_3d thread attach ";
		break;
    case DLL_THREAD_DETACH:
		//LOG(INFO) << "registration_3d thread detach ";
		break;
    case DLL_PROCESS_DETACH:
		//FreeLibrary("ATen_cuda.dll");
		//FreeLibrary("c10_cuda.dll");
		//FreeLibrary("torch_cuda.dll");
		//FreeLibrary("torchvision.dll");
		LOG(INFO) << "registration_3d process detach ";
        break;
    }
    return TRUE;
}

