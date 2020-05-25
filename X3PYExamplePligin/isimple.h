#ifndef X3_EXAMPLE_ISIMPLE_H
#define X3_EXAMPLE_ISIMPLE_H

#include <objptr.h>
#include <string>
//#define SIA_NO_ERROR 0
//#define SIA_ERR_SNDTIMEOUT -1 ///<����ʧ�ܷ���ֵ������Ӧ��ʱ
//#define SIA_ERR_NETDISCNT -2 ///<����ʧ�ܷ���ֵ����������Ͽ�
//#define SIA_ERR_PARAMINVALID -3 ///<������Ч
//#define SIA_ERR_CHECKCODE	-4  ///< ��������У�������
//#define SIA_ERR_DATAINVALID -5  ///<����������Ч
//#define SIA_ERR_NOTINITION	-6 ///<δ��ʼ��
//#define SIA_ERR_CONNECT_TIMEOUT -7 ///<�������ӳ�ʱ
//#define SIA_ERR_CONNECT_FAILED -8 ///<��������ʧ��
//#define SIA_ERR_STATUSFAULT  -9 ///����״̬����
//#define SIA_ERR_JOBOVERLAP	-10 ///��ҵ����ͬ�����Ѹ���
//#define SIA_ERR_JOBSPACE_SHORTAGE -11 ///< ��ҵ�ռ�����
//#define SIA_ERR_OTHER_CONDITION -12 ///< �ź��쳣
//
typedef  void(*lpCALLBACK) (std::string *, int, LPVOID);
/*<FUNC+>*******************************************************
* ��������: disPatchChar
* ��������: �����õĽӿں���
* �������:
iCnt=�����ķ�֧
iInstanID = ʵ����ID
strInData����������
* �������:
strOutData����ַ���
* �� �� ֵ:��������
* ��������:
* ����˵��: ��
* �޸ļ�¼:
* -------------------------------------------------------------
*    2019/3/13        1.0                     ��������
*<FUNC->*******************************************************
*/
class ISimple : public x3::IObject
{
    X3DEFINE_IID(ISimple);
    virtual int add(int a, int b) const = 0;
    virtual int subtract(int a, int b) const = 0;

	//virtual int disPatch(int iCnt) const = 0;

	virtual int disPatchChar(BYTE ucType, int& iInstanID, std::string strInData, std::string & strOutData, lpCALLBACK pCallback = NULL, LPVOID lpPara = NULL) const = 0;

	virtual int CtrlState(int iState = 0) const = 0;
	//virtual int disPatchFloat(int iCnt, const float* pfData, int iInCnt, float* pfOutData = 0, int iOutData = 0) const = 0;
};

#endif