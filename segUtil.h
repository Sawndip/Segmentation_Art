#ifndef _THREE_SEGMENT_UTIL_H_
#define _THREE_SEGMENT_UTIL_H_

#include <tuple>
#include <vector>
#include <math.h>
// tools - just using Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using :: std :: vector;
using :: std :: tuple;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////    
//// Log Macros: Minimal Log Facility
#define LogD(format, ...)  printf("[%-8s:%4d] [DEBUG] " format, \
                                   __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define LogI(format, ...)  printf("[%-8s:%4d] [INFO] " format, \
                                   __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define LogW(format, ...)  printf("[%-8s:%4d] [WARN] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogE(format, ...)  printf("[%-8s:%4d] [ERROR] " format, \
                                   __FILE__, __LINE__,  ##__VA_ARGS__)
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// Enumeraions
enum {BORDER_NUM = 4};
enum MOVING_DIRECTION
{
    TOP = 0, BOTTOM, LEFT, RIGHT = 3, 
    TOP_LEFT = 4, BOTTOM_LEFT, TOP_RIGHT, BOTTOM_RIGHT = 7,
    DIRECTION_NUM = 8, DIRECTION_UNKNOWN = 8
};
enum MOVING_STATUS
{   //moving stop is an assist status, along with other three status.
    MOVING_CROSS_IN = 0, MOVING_CROSS_OUT, MOVING_INSIDE, MOVING_STOP,
    MOVING_FINISH = 4, MOVING_UNKNOWN = 5
};
enum CONSUME_LINE_RESULT
{
    CONSUME_NOTHING = 0x0,
    CONSUME_IN_LINE = 0x2,
    CONSUME_OUT_LINE = 0x4
};
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// Basic Structures    
struct SegResults
{
    SegResults()
    : m_objIdx(-1)
    , m_inDirection(DIRECTION_UNKNOWN)
    , m_outDirection(DIRECTION_UNKNOWN)        
    , m_bOutForRecognize(false)
    , m_bTerminate(false)
    , m_curBox(0,0,0,0)
    , m_colorBox(0,0,0,0)
    {
        
    }
    int m_objIdx;
    MOVING_DIRECTION m_inDirection;
    MOVING_DIRECTION m_outDirection;    
    bool m_bOutForRecognize;
    bool m_bTerminate;
    cv::Rect m_curBox;
    cv::Rect m_colorBox;
};

struct TDPoint
{
    TDPoint() = default;
    TDPoint(const int a, const int b) : x(a), y(b) {}
    int x;
    int y;
};

// Not a normal Line,but with moving status
struct TDLine
{
    TDLine()
        : a(-1, -1) // indicate an empty line (invalid)
        , b(-1, -1)
        , movingDirection(DIRECTION_UNKNOWN)
        , movingStatus(MOVING_UNKNOWN)
        , bValid(false)
        , bUsed(false)        
        , mayPreviousLineStart(-1, -1)
        , mayPreviousLineEnd(-1, -1)        
    {
    }
    TDLine(const TDPoint & _a, const TDPoint & _b)
        : a(_a), b(_b)
        , movingAngle(0.0)      
        , movingDirection(DIRECTION_UNKNOWN)
        , movingStatus(MOVING_UNKNOWN)
        , bValid(false)
        , bUsed(false)        
        , mayPreviousLineStart(-1, -1)
        , mayPreviousLineEnd(-1, -1)        
    {
           
    }
    TDPoint a;
    TDPoint b;
    double movingAngle;
    MOVING_DIRECTION movingDirection;
    MOVING_STATUS movingStatus;
    // whether a valid line: size bigger than 32pixels & it has predecessors
    bool bValid; 
    bool bUsed; // whether used by contourTracker to do update
    // previous line's points.
    TDPoint mayPreviousLineStart;
    TDPoint mayPreviousLineEnd;    
    inline int getXLength() {return abs(a.x - b.x);}
    inline int getYLength() {return abs(a.y - b.y);}
    inline double getLength()
    {
        return sqrt(getXLength()*getXLength() + getYLength()*getYLength());
    }
};

// BgResult composes with two parts:
// 1. optical flow will fill binaryData & angles(mv);
// 2. boundary scan will fill lines(object cross the lines)
struct BgResult
{   // TODO: PXT: the four corner share the same mv? how do we deal with that?
    BgResult()
    {
        xMvs.resize(BORDER_NUM);
        yMvs.resize(BORDER_NUM);
        resultLines.resize(BORDER_NUM);
    }
    // copy assignment
    BgResult & operator=(const BgResult & another)
    {
        another.binaryData.copyTo(binaryData);
        for (int k=0; k < BORDER_NUM; k++)
        {
            xMvs[k] = another.xMvs[k];
            yMvs[k] = another.yMvs[k];
            resultLines[k] = another.resultLines[k];
        }
        return *this;
    }
    void reset()
    {
        xMvs.clear();
        yMvs.clear();
        resultLines.clear();
        xMvs.resize(BORDER_NUM);
        yMvs.resize(BORDER_NUM);
        resultLines.resize(BORDER_NUM);
    }
    // members
    cv::Mat binaryData;
    // 1. top, bottom, left, right. each border's size should be initialized properly by user.
    // 2. its size should be exactly the same as FourBorder's m_lines
    vector<vector<double> > xMvs; // actually the same as lines, its size is BORDER_NUM=4.
    vector<vector<double> > yMvs; 
    vector<vector<TDLine> > resultLines;
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//// Util Functions
extern char * getMovingDirectionStr(const MOVING_DIRECTION direction);
extern char * getMovingStatusStr(const MOVING_STATUS status);
extern bool isYContainedBy(const TDLine & small, const TDLine & large);
extern bool isXContainedBy(const TDLine & small, const TDLine & large);
extern int loopIndex(const int index, const int maxIdx);
extern MOVING_DIRECTION getPossibleMovingInDirection(const int lux, const int luy,
                                                     const int rectWidth, const int rectHeight,
                                                     const int imgWidth, const int imgHeight);
extern MOVING_DIRECTION getPossibleMovingInDirection(const cv::Rect & rect,
                                                     const int imgWidth, const int imgHeight);

// ignore the distance to the boundary
extern TDLine rectToBoundaryLine(const int bdNum, const cv::Rect & rect, const bool bCrossIn,
                                 const int skipTB, const int skipLR);
// consecutivity
extern double leftConsecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                          const int takeInterval,
                                          const int angleMaxScore, const bool bStart);
extern double rightConsecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                           const int takeInterval,
                                           const int angleMaxScore, const bool bStart);
extern double consecutivityOfTwoLines(const TDLine & l1, const TDLine & l2,
                                      const int takeInterval,
                                      const int angleMaxScore);
extern inline double diffAngleToScore(const double angle, const int maxScore);
extern inline double vertexShiftToScore(const int shift,
                                        const int takeInterval, const int maxScore);

// Rect Overlap Bound part
extern void boundBoxByMaxBox(cv::Rect & box, const cv::Rect & maxBox);
extern void enlargeBoxByMinBox(cv::Rect & box, const cv::Rect & minBox);
extern cv::Rect calcOverlapRect(const cv::Rect & a, const cv::Rect & b);
extern double calcOverlapRate(const cv::Rect & a, const cv::Rect & b);
extern double overlapPercentContainedBySmall(const cv::Rect & a, const cv::Rect & b);
extern int overlapXLenOfTwolines(const TDLine & a, const TDLine & b);

// inline log trivials
inline void dumpRect(const cv::Rect & rect)
{
    LogD("x:%d, y:%d, width:%d, height:%d.\n", rect.x, rect.y, rect.width, rect.height);
    return;
}
inline void dumpVectorInt(const vector<int> & vi)
{
    for (int k=0; k <(int)vi.size(); k++)
        LogD("Mo.%d: %d.\n", k, vi[k]);
    return;
}
inline int sumVectorInt(const vector<int> & vi)
{
    int sum = 0;
    for (int k=0; k <(int)vi.size(); k++)
        sum += vi[k];
    return sum;
}

}

#endif ////
