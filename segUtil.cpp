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
        "TOP", "BOTTOM", "LEFT", "RIGHT", "TOP_LEFT",
        "BOTTOM_LEFT", "TOP_RIGHT", "BOTTOM_RIGHT", "UNKNOWN"
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

    MOVING_DIRECTION getPossibleMovingInDirection(const cv::Rect & rect,
                                                  const int imgWidth, const int imgHeight)
    {
        return getPossibleMovingInDirection(rect.x, rect.y,
                                            rect.width, rect.height, imgWidth, imgHeight);
    }

    MOVING_DIRECTION getPossibleMovingInDirection(const int lux, const int luy,
                                                  const int rectWidth,
                                                  const int rectHeight,
                                                  const int imgWidth, const int imgHeight)
    {   // TODO: magic number 2 here.
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

    TDLine rectToBoundaryLine(const int bdNum, const cv::Rect & rect, const bool bCrossIn,
                              const int skipTB = 0, const int skipLR = 0)
    {
        assert(skipTB > 0 && skipLR > 0);
        TDLine boundaryLine;
        switch(bdNum)
        {
        case 0:
            boundaryLine = TDLine(TDPoint(rect.x - skipLR, 0),
                                  TDPoint(rect.x + rect.width - skipLR, 0));
            boundaryLine.movingAngle = bCrossIn ? -M_PI/2 : M_PI/2;
            break;
        case 1:
            boundaryLine = TDLine(TDPoint(rect.x - skipLR, 0),
                                  TDPoint(rect.x + rect.width - skipLR, 0));
            boundaryLine.movingAngle = bCrossIn ? M_PI/2: -M_PI/2;
            break;
        case 2:
            boundaryLine = TDLine(TDPoint(rect.y - skipTB, 0),
                                  TDPoint(rect.y + rect.height - skipTB, 0));
            boundaryLine.movingAngle = bCrossIn ? 0 : M_PI;
        case 3:
            boundaryLine = TDLine(TDPoint(rect.y - skipTB, 0),
                                  TDPoint(rect.y + rect.height - skipTB, 0));
            boundaryLine.movingAngle = bCrossIn ? M_PI : 0;
            break;            
        }
        if (boundaryLine.a.x < 0)
            boundaryLine.a.x = 0;
        if (boundaryLine.b.x < 0)
            boundaryLine.b.x = 0;
        return boundaryLine;
    }

    // the input argument angle: [0, 2*Pi]
    inline double diffAngleToScore(const double angle, const int maxScore)
    {  // aproaching 0, then score maxScore points, approaching pi, then 0.           
        if (angle <= M_PI) 
            return (-1.0 * maxScore / M_PI) * angle + maxScore;
        else // aproaching 2pi, then score maxScore points.
            return (maxScore / M_PI) * angle - maxScore;
    }
    inline double vertexShiftToScore(const int shift, const int maxScore)
    {
       if (shift > 64)
           return 0;
       else if (shift > 32)
           return 0.1 * maxScore;
       else if (shift > 16)
           return 0.4 * maxScore;
       else if (shift > 8)
           return 0.7 * maxScore;
       else if (shift > 4)
           return (double)maxScore;
       else
           return (0.9 * maxScore);
    }
    // check two lines' possibility to be consecutive.
    // 1. they have similar angle (50 points)
    // 2. if not, start/end point is close (inside 16 pixels, 50points)
    double leftConsecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                       const int angleMaxScore, const bool bStart)
    {       
       int shift = l1.a.x - l2.a.x;
       if (bStart == false)
           shift = l1.b.x - l2.b.x;
       if (shift < 0)
           return 0.0;
       const double diffAngle = fabs(l1.movingAngle - l2.movingAngle);
       double score = diffAngleToScore(diffAngle, angleMaxScore);
       score += vertexShiftToScore(shift, 100 - angleMaxScore);
       return score;
    }
    double rightConsecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                        const int angleMaxScore, const bool bStartPoint)
    {
        return leftConsecutivityOfTwoLines(l2, l1, angleMaxScore, bStartPoint); 
    }
    // balanced consecutivity: namely left & right all similar
    double consecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                   const int angleMaxScore)
    {
        // two of those calls' return value will be 0.0
        double score = leftConsecutivityOfTwoLines(l1, l2, angleMaxScore, true);
        score += leftConsecutivityOfTwoLines(l1, l2, angleMaxScore, false);
        score += rightConsecutivityOfTwoLines(l1, l2, angleMaxScore, true);
        score += rightConsecutivityOfTwoLines(l1, l2, angleMaxScore, false);
        return score / 2; 
    }

    // prerequisite: width/height of the two rects are the same.
    double calcOverlapRate(const cv::Rect & a, const cv::Rect & b)
    {
        assert(a.width == b.width && a.height == b.height);
        if (a.width == 0 || a.height == 0)
            return 0.0;
        cv::Rect overlapBox = calcOverlapRect(a, b);
        return (overlapBox.width * overlapBox.height * 1.0 / (a.width * a.height));
    }
    // percentage = (a&b's overlap area) / (a's area)
    double percentContainedBy(const cv::Rect & a, const cv::Rect & b)
    {
        if (a.width == 0 || a.height == 0)
            return 1.0;
        if (b.width == 0 || b.height == 0)
            return 0.0;
        cv::Rect overlapBox = calcOverlapRect(a, b);
        return (overlapBox.width * overlapBox.height * 1.0 / (a.width * a.height));
    }

    cv::Rect calcOverlapRect(const cv::Rect & a, const cv::Rect & b)
    {
        if (a.x + a.width < b.x  || a.x > b.x + b.width ||
            a.y + a.height < b.y || a.y > b.y + b.height  )
            return cv::Rect(0, 0, 0, 0);
        const int x = std::max(a.x, b.x);
        const int y = std::max(a.y, b.y);
        const int width = a.width + b.width -
                          (std::max(a.x+a.width, b.x+b.width) - std::min(a.x, b.x));
        const int height = a.height + b.height -
                           (std::max(a.y + a.height, b.y + b.height) - std::min(a.y, b.y));
        return cv::Rect(x, y, width, height);
    }

    void enlargeBoxByMinBox(cv::Rect & box, const cv::Rect & minBox)
    {
        if (box.x > minBox.x)
            box.x = minBox.x;
        if (box.y > minBox.y)
            box.y = minBox.y;
        if (box.x + box.width < minBox.x + minBox.width)
            box.width = minBox.x + minBox.width - box.x;
        if (box.y + box.height < minBox.y + minBox.height)
            box.height = minBox.y + minBox.height - box.y;    
        return;
    }

    void boundBoxByMaxBox(cv::Rect & box, const cv::Rect & maxBox)
    {
        if (box.x < maxBox.x)
            box.x = maxBox.x;
        if (box.y < maxBox.y)
            box.y = maxBox.y;
        if (box.x + box.width > maxBox.x + maxBox.width)
            box.width = maxBox.x + maxBox.width - box.x;
        if (box.y + box.height > maxBox.y + maxBox.height)
            box.height = maxBox.y + maxBox.height - box.y;    
        return;
    }

    int overlapXLenOfTwolines(const TDLine & a, const TDLine & b)
    {
        if (a.a.x >= b.b.x)
            return b.b.x - a.a.x; // nagtive number
        else if (b.a.x >= a.b.x)
            return a.b.x - b.a.x; // nagtive number
        else
            return std::min(abs(a.b.x - b.a.x), abs(b.b.x - a.a.x));
    }
    
} // namespace
