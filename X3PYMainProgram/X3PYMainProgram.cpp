// X3PYMainProgram.cpp : 定义控制台应用程序的入口点。

#include "stdafx.h"
#include <portability/x3port.h>     // 相当于#include<windows.h>
#include <nonplugin/useplugins.h>   // 包含辅助加载类，一个工程内只能包含一次
#include "isimple.h"
//#include "isimple2.h"
//#include <objptr.h>
void DisCallBack(std::string* strData, int iCmd, LPVOID pThis)
{

}
int test()
{
  x3::Object<ISimple> obj1("94071767-ba6b-4769-9eb4-PoseEstimation");     // 给定类ID创建对象
	std::string strOut;
	int iTID;
	obj1->disPatchChar(0, iTID, "12", strOut, DisCallBack, NULL);
	printf(" %s\r\n", strOut.c_str());
	//x3::Object<ISimple> obj(clsidSimple);     // 给定类ID创建对象
	//while (true)
	//{
	//	if (obj1)                  // 检查是否创建成功
	//	{
	//		int res1 = obj1->add(1, 2);       // 调用接口函数，就像普通指针一样
	//		printf("ww1+2 = %d\r\n", res1);
	//		Sleep(1000);
	//	}
	return 0;
}

void LoadDLL()
{
	// 多个插件文件名，以NULL结尾
	const char* plugins[] = {
		"PoseEstimation.pln", NULL
	};
	const char* strID[] = {
	  "94071767-ba6b-4769-9eb4-PoseEstimation", NULL
	};
	const char* path[] = {
		"plugins\\PoseEstimation"
	};
	// 自动加载和卸载插件，插件在程序文件的plugins子目录下
	x3::AutoLoadPlugins autoload(plugins, strID, path);

	// 可以使用接口了。。。
	test();
}

void _tmain()
{
	LoadDLL();
	getchar();
}




