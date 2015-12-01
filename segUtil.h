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
    TOP = 0, BOTTOM, RIGHT, LEFT, CENTER = 4,
    TOPLEFT, BOTTOMLEFT, TOPRIGHT, BOTTOMRIGHT = 8,
    DIRECTION_NUM = 9, DIRECTION_UNKNOWN = 9
};
enum MOVING_STATUS
{
    MOVING_CROSS_IN = 0, MOVING_CROSS_OUT, MOVING_INSIDE, MOVING_STOP, 
    MOVING_UNKNOWN = 4
};
char * getMovingDirectionStr(const MOVING_DIRECTION direction);
char * getMovingStatusStr(const MOVING_STATUS status);

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// Basic Structures    
struct SegResults
{
    int m_objIdx;
    MOVING_DIRECTION inDirection;
    MOVING_DIRECTION outDirection;    
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
        bTraced = false;
        mayPreviousLine = NULL;
    }
    TDLine(const TDPoint & _a, const TDPoint & _b)
        : a(_a), b(_b)
        , movingAngle(0.0)      
        , movingDirection(DIRECTION_UNKNOWN)
        , movingStatus(MOVING_UNKNOWN)
        , bTraced(false)
        , mayPreviousLine(NULL)
    {
           
    }
    TDPoint a;
    TDPoint b;
    double movingAngle;
    MOVING_DIRECTION movingDirection;
    MOVING_STATUS movingStatus;
    bool bTraced;
    // BoundaryScan help ThreeDiff to mark the previous boundary line.
    // ThreeDiff uses this info to do MovingIn/Out Rect updating.
    // Namely: ((previous->a.x - current->a.x) + (p->b.x - c->b.x)) / 2 
    TDLine *mayPreviousLine; 
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
extern bool isYContainedBy(const TDLine & small, const TDLine & large);
extern bool isXContainedBy(const TDLine & small, const TDLine & large);
extern int loopIndex(const int index, const int maxIdx);
}

#endif ////
