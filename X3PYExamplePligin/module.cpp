#include "stdafx.h"
#include <module/plugininc.h>
#include <module/pluginimpl.h>      // ʵ�ֲ���ĵ�������
#include <module/modulemacro.h>     // ע��ʵ����ĺ궨��

#include "libpln1.h"               // ����ʵ����

XBEGIN_DEFINE_MODULE()
XDEFINE_CLASSMAP_ENTRY(CSimpleA) // ע����ͨʵ�����ʵ����
//XDEFINE_CLASSMAP_ENTRY_Singleton(libpln1)
XEND_DEFINE_MODULE()            // �����̬��

OUTAPI bool x3InitializePlugin()    // �������ʱִ�У����ڶ����ʼ��
{
	return true;
}

OUTAPI void x3UninitializePlugin()  // ���ж��ʱִ�У������ͷŶ�������
{
}

