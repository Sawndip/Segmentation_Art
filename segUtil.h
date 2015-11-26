#ifndef _THREE_SEGMENT_UTIL_H_
#define _THREE_SEGMENT_UTIL_H_

#include <tuple>
#include <vector>
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
enum DIRECTION
{
    TOP = 0, BOTTOM, RIGHT, LEFT, CENTER,
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

struct BgResult
{   // TODO: PXT: the four corner share the same mv? how do we deal with that?
    cv::Mat binaryData;
    vector<double> angles[BORDER_NUM];
    // copy assignment
    BgResult & operator=(const BgResult & result)
    {
        result.binaryData.copyTo(binaryData);
        for (int k=0; k < BORDER_NUM; k++)
            angles[k] = result.angles[k];
        return *this;
    }
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
        , movingDirection(DIRECTION_UNKNOWN)
        , movingStatus(MOVING_UNKNOWN)
    {
           
    }
    TDPoint a;
    TDPoint b;
    DIRECTION movingDirection;
    MOVING_STATUS movingStatus;
};

//////////////////////////////////////////////////////////////////////////////////////////
class FourBorders
{
public:
    FourBorders() = default;
    FourBorders(const int widthTB, const int widthLR)
    {
        init(widthTB, 2, widthLR, 2);
    }
    FourBorders(const int widthTB, const int heightTB, const int widthLR, const int heightLR)
    {
        init(widthTB, heightTB, widthLR, heightLR);
    }

private:
    int init(const int widthTB, const int heightTB, const int widthLR, const int heightLR)
    {
        m_widthTB = widthTB;
        m_heightTB = heightTB;
        m_widthLR = widthLR;
        m_heightLR = heightLR;
        return 0;
    }
    
public:
    int m_widthTB;
    int m_heightTB;
    int m_widthLR;
    int m_heightLR;    
    vector<TDLine> m_lines[4];
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//// Util Functions
extern bool isYContainedBy(const TDLine & small, const TDLine & large);
extern bool isXContainedBy(const TDLine & small, const TDLine & large);
}

#endif ////
