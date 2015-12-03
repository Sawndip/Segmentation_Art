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

    TDLine rectToBoundaryLine(const int bdNum, const cv::Rect & rect)
    {             
        TDLine boundaryLine;
        switch(bdNum)
        {
        case 0:
            boundaryLine = TDLine(TDPoint(rect.x, 0), TDPoint(rect.x + rect.width, 0));
            boundaryLine.movingAngle = - M_PI / 2;
            break;
        case 1:
            boundaryLine = TDLine(TDPoint(rect.x, 0), TDPoint(rect.x + rect.width, 0));
            boundaryLine.movingAngle = M_PI / 2;
            break;
        case 2:
            boundaryLine = TDLine(TDPoint(rect.y, 0), TDPoint(rect.y + rect.height, 0));
            boundaryLine.movingAngle = 0;
        case 3:
            boundaryLine = TDLine(TDPoint(rect.y, 0), TDPoint(rect.y + rect.height, 0));
            boundaryLine.movingAngle = M_PI;
            break;            
        }
        return boundaryLine;
    }

    // check two lines' possibility to be consecutive.
    // 1. they have similar angle (50 points)
    // 2. if not, start/end point is close (inside 16 pixels, 50points)
    double consecutivityOfTwoLines(const TDLine & l1, const TDLine & l2)
    {
       double score = 0.0;
       // 1. moving angle: y = (-100/PI)x + 50, y = (100/PI)x - 150
       const double diffAngle = fabs(l1.movingAngle - l2.movingAngle);
       if (diffAngle <= M_PI) // aproaching 0, then score 50points
           score += (-100.0 / M_PI) * diffAngle + 50;
       else // aproaching 2pi, then score 50points.
           score += (100.0 / M_PI) * diffAngle - 150;

       const int startShift = l1.a.x - l2.a.x;
       const int endShift = l1.b.x - l2.b.x;
       // 2. whether shift the same direction takes 10       
       const int lineShift = abs(startShift - endShift);
       if (lineShift <= 4)
           score += 20;
       else if (lineShift <= 8)
           score += 15;
       else if (lineShift <= 16)
           score += 10;
       else if (lineShift <= 32)
           score += 5;
       else
           score += 0;

       const int lineClose = abs(startShift) + abs(endShift);
       if (lineClose < 4)
           score += 15;
       else if (lineClose < 8)
           score += 20;
       else if (lineClose < 16)
           score += 15;
       else if (lineClose < 32)
           score += 10;
       else if (lineClose < 64)
           score += 5;
       else
           score += 0;
       return score;
    }
    
} // namespace
