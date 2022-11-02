#ifndef IRECOGNITION_H
#define IRECOGNITION_H

#include "stdafx.h"

class IRecognition 
{
	public:
		virtual bool Compute() = 0;
};

extern "C" __declspec(dllexport) IRecognition* APIENTRY GetRecognition3D();
extern "C" __declspec(dllexport) IRecognition* APIENTRY GetRecognition3DPPF();

#endif
