#ifndef X3_EXAMPLE_SIMPLE_IMPL_H
#define X3_EXAMPLE_SIMPLE_IMPL_H

#include <module/classmacro.h>
#include "isimple.h"
typedef  void(*lpCALLBACK) (std::string *, int, LPVOID);
const char* const g_clsidSimple = "94071767-ba6b-4769-9eb4-PoseEstimationPlugin";
class CSimpleA : public ISimple
{
	X3BEGIN_CLASS_DECLARE(CSimpleA, g_clsidSimple)
        X3DEFINE_INTERFACE_ENTRY(ISimple)
    X3END_CLASS_DECLARE()
protected:
    CSimpleA();
    virtual ~CSimpleA();

private:
    virtual int add(int a, int b) const;
    virtual int subtract(int a, int b) const;

	//virtual int disPatch(int iCnt) const;
	//virtual int disPatchChar(int iCnt) const;

	virtual int disPatchChar(BYTE ucType, int& iInstanID, std::string strInData, std::string & strOutData, lpCALLBACK pCallback = NULL, LPVOID lpPara = NULL)  const;

	virtual int CtrlState(int iState = 0) const;
	//virtual int disPatchInt(int iCnt, const int* piData, int iInCnt, int* piOutData = 0, int iOutData = 0) const;
	//virtual int disPatchFloat(int iCnt, const float* pfData, int iInCnt, float* pfOutData = 0, int iOutData = 0) const;
};

#endif