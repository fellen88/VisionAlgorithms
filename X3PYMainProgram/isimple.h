#ifndef X3_EXAMPLE_ISIMPLE_H
#define X3_EXAMPLE_ISIMPLE_H

#include <objptr.h>
#include <string>
//const char* const clsidSimple = "94071767-ba6b-4769-9eb4-X3PYExamplePligin";
typedef  void(*lpCALLBACK) (std::string *, int, LPVOID);

class ISimple : public x3::IObject
{
    X3DEFINE_IID(ISimple);
    virtual int add(int a, int b) const = 0;
    virtual int subtract(int a, int b) const = 0;

	//virtual int disPatch(int iCnt) const = 0;
	virtual int disPatchChar(BYTE ucType, int& iInstanID, std::string strInData, std::string & strOutData, lpCALLBACK pCallback = NULL, LPVOID lpPara = NULL)  const = 0;

	virtual int CtrlState(int iState = 0) const = 0;
	//virtual int disPatchInt(int iCnt, const int* piData, int iInCnt, int* piOutData = 0, int iOutData = 0)  const = 0;
	//virtual int disPatchFloat(int iCnt, const float* pfData, int iInCnt, float* pfOutData = 0, int iOutData = 0) const = 0;
};

#endif