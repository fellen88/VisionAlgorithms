// x3py framework: https://github.com/rhcad/x3py
#ifndef X3_NONPLUGIN_USE_PLUGINS_H
#define X3_NONPLUGIN_USE_PLUGINS_H

#include <utilfunc/loadmodule.h>
#include "swigext.h"

// PLUGIN_PATH:         the internal plugin (*.pln) 's relative folder.
// SELF_MODULE_NAME:    file name of the caller module which will load the internal plugins.
// SELF_MODULE:         used to make SELF_MODULE_NAME.

#ifndef SELF_MODULE_NAME
#ifndef SELF_MODULE
#define SELF_MODULE_NAME  NULL      // application file name
#else
#define SELF_MODULE_NAME  SELF_PRE SELF_MODULE SELF_EXT
#endif
#endif // SELF_MODULE_NAME

#if defined(X3_CORE_PORTABILITY_H) && !defined(X3_EXCLUDE_CREATEOBJECT)
#include "../portability/portimpl.h"
#endif

namespace x3 {

static LoadModuleHelper* s_plugins[40] = { NULL };
static int s_nplugin = 0;
static const char* g_strID[40] = { NULL/*, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL*/ };

#if !defined(X3_EXCLUDE_CREATEOBJECT) && !defined(CREATEOBJECTIMPL)
#define CREATEOBJECTIMPL

HMODULE getManagerModule()
{
    return s_plugins[0] ? s_plugins[0]->getModule() : NULL;
}

class IObject;
bool createObject(const char* clsid, long iid, IObject** p)
{
    typedef bool (*F)(const char*, long, IObject**);
	int i = 0;
	for (i = 0; i < 40 - 1 && clsid[i]; i++)
	{
		if (0 == strcmp(g_strID[i], clsid))
		{
			F f = !s_plugins[i] ? NULL : (F)s_plugins[i]->getFunc("x3CreateObject");
			return f && f(clsid, iid, p);
		}
	}

	return false;
}
#endif // CREATEOBJECTIMPL

int loadPlugins(const char* const* plugins, const char* const*  folder)
{
    HMODULE basemod = GetModuleHandleA(SELF_MODULE_NAME);

    for (int i = 0; s_nplugin < 40 - 1 && plugins[i]; ++i)
    {
        LoadModuleHelper* p = new LoadModuleHelper();
        if (p->load(plugins[i], basemod, folder[i]))
        {
            s_plugins[s_nplugin++] = p;
        }
        else
        {
            delete p;
        }
    }

    return s_nplugin;
}

void unloadPlugins()
{
    for (int i = s_nplugin - 1; i >= 0; i--)
    {
        delete s_plugins[i];
        s_plugins[i] = NULL;
    }
    s_nplugin = 0;
}

/** \code
 *   const char* plugins[] = { "x3manager.pln", "myplugin.pln", NULL };
 *   x3::AutoLoadPlugins autoload(plugins, "plugins");
 * \endcode
 */
struct AutoLoadPlugins
{
	AutoLoadPlugins(const char* const* plugins, const char* const* strID, const char* const* folder )
    {
		//for (int i = 0; i < iCnt; i++)
		//{
		//	s_nAdd[i] = clsids[i];
		//}
		for (int i = 0; i < 40 - 1 && strID[i]; ++i)
		{
			g_strID[i] = strID[i];
		}
        loadPlugins(plugins, folder);
    }
    ~AutoLoadPlugins()
    {
        unloadPlugins();
    }
};

} // x3
#endif
