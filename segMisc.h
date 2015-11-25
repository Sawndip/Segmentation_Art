#ifndef _THREE_SEGMENT_MISC_H_
#define _THREE_SEGMENT_MISC_H_

#include <tuple>
#include <vector>
// tools - just using Mat
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using :: std :: vector;
using :: std :: tuple;

namespace Seg_Three
{

//// Log Macros: Minimal Log Facility
#define LogD(format, ...)  printf("[%-8s:%4d] [DEBUG] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogI(format, ...)  printf("[%-8s:%4d] [INFO] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogW(format, ...)  printf("[%-8s:%4d] [WARN] " format, \
                                   __FILE__, __LINE__, ##__VA_ARGS__)
#define LogE(format, ...)  printf("[%-8s:%4d] [ERROR] " format, \
                                   __FILE__, __LINE__,  ##__VA_ARGS__)
    
//// Misc Basic Structures    
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

struct TDLine
{
    TDLine() = default;
    TDLine(const TDPoint & _a, const TDPoint & _b) : a(_a), b(_b) {}
    TDPoint a;
    TDPoint b;
};

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
class FourBorders
{
public:
    FourBorders() = default;
    FourBorders(const int imgWidth, const int imgHeight)
    {
        init(imgWidth, 2, imgHeight, 2, 0, 0, 0, 0);
    }
    FourBorders(const int widthTB, const int heightTB, const int widthLR, const int heightLR,
                const int skipT, const int skipB, const int skipL, const int skipR)
    {
        init(widthTB, heightTB, widthLR, heightLR, skipT, skipB, skipL, skipR);
    }
    int init(const int widthTB, const int heightTB, const int widthLR, const int heightLR,
             const int skipT, const int skipB, const int skipL, const int skipR)
    {
        m_skipT = skipT;
        m_skipB = skipB;
        m_skipL = skipL;
        m_skipR = skipR;        
        m_widthTB = widthTB;
        m_heightTB = heightTB;
        m_widthLR = widthLR;
        m_heightLR = heightLR;
        return 0;
    }
    
public:
    int m_skipT; // Top Bottome Left Right skip pixels  
    int m_skipB;
    int m_skipL;
    int m_skipR;    
    int m_widthTB;
    int m_heightTB;
    int m_widthLR;
    int m_heightLR;
    vector<TDLine> m_lines[4];
};

enum DIRECTION {TOP = 0, BOTTOM, RIGHT, LEFT, DIRECTION_NUM = 4, DIRECTION_UNKNOWN = 4};
enum MOVING_STATUS {MOVING_IN = 0, MOVING_INSIDE, MOVING_STOP, MOVING_OUT, MOVING_UNKNOWN};

extern bool isYContainedBy(const TDLine & small, const TDLine & large);
extern bool isXContainedBy(const TDLine & small, const TDLine & large);
}

#endif ////
