// X3PYMainProgram.cpp : �������̨Ӧ�ó������ڵ㡣

#include "stdafx.h"
#include <portability/x3port.h>     // �൱��#include<windows.h>
#include <nonplugin/useplugins.h>   // �������������࣬һ��������ֻ�ܰ���һ��
#include "isimple.h"
//#include "isimple2.h"
//#include <objptr.h>
void DisCallBack(std::string* strData, int iCmd, LPVOID pThis)
{

}
int test()
{
  x3::Object<ISimple> obj1("94071767-ba6b-4769-9eb4-PoseEstimation");     // ������ID��������
	std::string strOut;
	int iTID;
	obj1->disPatchChar(0, iTID, "12", strOut, DisCallBack, NULL);
	printf(" %s\r\n", strOut.c_str());
	//x3::Object<ISimple> obj(clsidSimple);     // ������ID��������
	//while (true)
	//{
	//	if (obj1)                  // ����Ƿ񴴽��ɹ�
	//	{
	//		int res1 = obj1->add(1, 2);       // ���ýӿں�����������ָͨ��һ��
	//		printf("ww1+2 = %d\r\n", res1);
	//		Sleep(1000);
	//	}
	return 0;
}

void LoadDLL()
{
	// �������ļ�������NULL��β
	const char* plugins[] = {
		"PoseEstimation.pln", NULL
	};
	const char* strID[] = {
	  "94071767-ba6b-4769-9eb4-PoseEstimation", NULL
	};
	const char* path[] = {
		"plugins\\PoseEstimation"
	};
	// �Զ����غ�ж�ز��������ڳ����ļ���plugins��Ŀ¼��
	x3::AutoLoadPlugins autoload(plugins, strID, path);

	// ����ʹ�ýӿ��ˡ�����
	test();
}

void _tmain()
{
	LoadDLL();
	getchar();
}




