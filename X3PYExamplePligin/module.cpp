#include "stdafx.h"
#include <module/plugininc.h>
#include <module/pluginimpl.h>      // 实现插件的导出函数
#include <module/modulemacro.h>     // 注册实现类的宏定义

#include "libpln1.h"               // 包含实现类

XBEGIN_DEFINE_MODULE()
XDEFINE_CLASSMAP_ENTRY(CSimpleA) // 注册普通实现类或单实例类
//XDEFINE_CLASSMAP_ENTRY_Singleton(libpln1)
XEND_DEFINE_MODULE()            // 插件动态库

OUTAPI bool x3InitializePlugin()    // 插件加载时执行，用于额外初始化
{
	return true;
}

OUTAPI void x3UninitializePlugin()  // 插件卸载时执行，用于释放额外数据
{
}

