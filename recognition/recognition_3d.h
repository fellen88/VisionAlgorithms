#ifndef RECOGNITION_3D_H
#define RECOGNITION_3D_H

#include "irecognition.h"

class Recognition3D : public IRecognition
{
	public:
		Recognition3D();
		~Recognition3D();
		virtual bool Compute();

};

#endif

