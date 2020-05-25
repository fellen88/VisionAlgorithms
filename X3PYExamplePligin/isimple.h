#ifndef X3_EXAMPLE_ISIMPLE_H
#define X3_EXAMPLE_ISIMPLE_H

#include <objptr.h>
#include <string>
//#define SIA_NO_ERROR 0
//#define SIA_ERR_SNDTIMEOUT -1 ///<发送失败返回值，代表应答超时
//#define SIA_ERR_NETDISCNT -2 ///<发送失败返回值，代表网络断开
//#define SIA_ERR_PARAMINVALID -3 ///<参数无效
//#define SIA_ERR_CHECKCODE	-4  ///< 返回数据校验码错误
//#define SIA_ERR_DATAINVALID -5  ///<返回数据无效
//#define SIA_ERR_NOTINITION	-6 ///<未初始化
//#define SIA_ERR_CONNECT_TIMEOUT -7 ///<网络连接超时
//#define SIA_ERR_CONNECT_FAILED -8 ///<网络连接失败
//#define SIA_ERR_STATUSFAULT  -9 ///制器状态错误
//#define SIA_ERR_JOBOVERLAP	-10 ///作业下载同名，已覆盖
//#define SIA_ERR_JOBSPACE_SHORTAGE -11 ///< 作业空间已满
//#define SIA_ERR_OTHER_CONDITION -12 ///< 信号异常
//
typedef  void(*lpCALLBACK) (std::string *, int, LPVOID);
/*<FUNC+>*******************************************************
* 函数名称: disPatchChar
* 功能描述: 外界调用的接口函数
* 输入参数:
iCnt=函数的分支
iInstanID = 实例化ID
strInData输入数据流
* 输出参数:
strOutData输出字符串
* 返 回 值:意义如上
* 操作流程:
* 其它说明: 无
* 修改记录:
* -------------------------------------------------------------
*    2019/3/13        1.0                     创建函数
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