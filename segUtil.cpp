#include "segUtil.h"

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
    // Enum to String
    static const char *MovingDirections[] =
    {
        "TOP", "BOTTOM", "LEFT", "RIGHT", "CENTER", "UNKNOWN"
    };
    static const char *MovingStatus[] =
    {
        "CrossIn", "CrossOut", "Inside", "Stop", "Unknown"
    };
    
    char * getMovingDirectionStr(const MOVING_DIRECTION direction)
    {
        return (char *)MovingDirections[(int)direction];
    }
    char * getMovingStatusStr(const MOVING_STATUS status)
    {
        return (char *)MovingStatus[(int)status];
    }

} // namespace
