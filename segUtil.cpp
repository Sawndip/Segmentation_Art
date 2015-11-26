#include "segMisc.h"

namespace Seg_Three
{
    bool isXContainedBy(const TDLine & small, const TDLine & large)
    {
        return small.a.x >= large.a.x && small.b.x <= large.b.x;
    }    
    bool isYContainedBy(const TDLine & small, const TDLine & large)
    {
        return small.a.y >= large.a.y && small.b.y <= large.b.y;
    }
} // namespace
