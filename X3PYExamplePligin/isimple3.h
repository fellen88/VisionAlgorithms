#ifndef X3_EXAMPLE_ISIMPLE3_H
#define X3_EXAMPLE_ISIMPLE3_H

#include <objptr.h>

class ISimple3 : public x3::IObject
{
    X3DEFINE_IID(ISimple3);

    virtual x3::AnyObject createSimple() = 0;
};

#endif