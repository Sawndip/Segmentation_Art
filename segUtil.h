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
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogI(format, ...)  printf("[%-8s:%4d] [INFO] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
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
    TOP = 0, BOTTOM, RIGHT, LEFT, CENTER = 4,
    DIRECTION_NUM = 5, DIRECTION_UNKNOWN = 5
};
enum MOVING_STATUS
{
    MOVING_IN = 0, MOVING_OUT, MOVING_INSIDE, MOVING_STOP, 
    MOVING_UNKNOWN = 4
};
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// Basic Structures    
struct SegResults
{
    int m_objIdx;
    bool m_bOutForRecognize;
    cv::Rect m_curBox;
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
    {
        movingDirection = DIRECTION_UNKNOWN;
        movingStatus = MOVING_UNKNOWN;
    }
    TDLine(const TDPoint & _a, const TDPoint & _b)
        : a(_a), b(_b)
        , movingAngle(0.0)      
        , movingDirection(DIRECTION_UNKNOWN)
        , movingStatus(MOVING_UNKNOWN)
    {
           
    }
    TDPoint a;
    TDPoint b;
    double movingAngle;
    MOVING_DIRECTION movingDirection;
    MOVING_STATUS movingStatus;
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
    }
    // copy assignment
    BgResult & operator=(const BgResult & result)
    {
        result.binaryData.copyTo(binaryData);
        for (int k=0; k < BORDER_NUM; k++)
        {
            xMvs[k] = result.xMvs[k];
            yMvs[k] = result.yMvs[k];
            lines[k] = result.lines[k];            
        }
        return *this;
    }
    // members
    cv::Mat binaryData;
    // 1. top, bottom, left, right. each border's size should be initialized properly by user.
    // 2. its size should be exactly the same as FourBorder's m_lines
    vector<vector<double> > xMvs; // actually the same as lines, its size is BORDER_NUM=4.
    vector<vector<double> > yMvs; 
    vector<TDLine> lines[BORDER_NUM];
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//// Util Functions
extern bool isYContainedBy(const TDLine & small, const TDLine & large);
extern bool isXContainedBy(const TDLine & small, const TDLine & large);
}

#endif ////
