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

    int loopIndex(const int index, const int maxIdx)
    {
        const int nextIdx = index + 1;
        if (nextIdx >= maxIdx)
            return 0;
        return nextIdx;
    }

    MOVING_DIRECTION getPossibleMovingInDirection(const int lux, const int luy,
                                                  const int rectWidth,
                                                  const int rectHeight,
                                                  const int imgWidth, const int imgHeight)
    {
        const int rightGap = imgWidth - lux - rectWidth;        
        const int bottomGap = imgHeight - luy - rectHeight;
        if (lux <= 2 && luy <= 2)
            return TOP_LEFT;
        else if (lux <= 2 && bottomGap <= 2)
            return BOTTOM_LEFT;
        else if (luy <= 2 && rightGap <= 2)
            return TOP_RIGHT;
        else if (rightGap <= 2 && bottomGap <= 2)
            return BOTTOM_RIGHT;
        else if (lux <= 2)
            return LEFT;
        else if (luy <= 2)
            return TOP;
        else if (rightGap <= 2)
            return RIGHT;
        else if (bottomGap <= 2)
            return BOTTOM;
        return DIRECTION_UNKNOWN;
    }
} // namespace
