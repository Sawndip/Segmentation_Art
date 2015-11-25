#ifndef _BOUNDARY_SCAN_H_
#define _BOUNDARY_SCAN_H_

// sys
#include <vector>
#include <tuple>
#include <string>
// tools 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segMisc.h"
#include "vectorSpace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using :: std :: tuple;
using namespace Vector_Space;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////    
class BoundaryScan
{
public:
    BoundaryScan();
    ~BoundaryScan();
    int init(const int width, const int height);
    int processFrame(const cv::Mat & bgResult, FourBorders & lines);

private: // inner classes
    class BordersMem
    {
    public:
        BordersMem() : bInit(false), top(NULL), bottom(NULL), left(NULL),right(NULL) {};
        ~BordersMem()
        {
            if (top) delete [] top;
            if (bottom) delete [] bottom;
            if (left) delete [] left;
            if (right) delete [] right;            
        }
        void init(const int _widthTB, const int _heightTB,
                  const int _widthLR, const int _heightLR)
        {
            if (bInit == false)
            {
                widthTB = _widthTB;
                heightTB = _heightTB;                
                widthLR = _widthLR;
                heightLR = _heightLR;                
                top = new unsigned char[widthTB * heightTB];
                bottom = new unsigned char[widthTB * heightTB];
                left = new unsigned char[widthLR * heightLR];
                right = new unsigned char[widthLR * heightLR];
                bInit = true;
            }
        }        
    public:
        bool bInit;
        int widthTB;
        int widthLR;
        int heightTB;
        int heightLR;        
        unsigned char * top;
        unsigned char * bottom;
        unsigned char * left;        
        unsigned char * right;
    };

private: // inner members
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    unsigned char *m_directions[DIRECTION_NUM];
    BordersMem m_bordersMem;
    static const int M_ELEMENT_WIDTH = 2;    
    static const int M_ELEMENT_HEIGHT = 2;

private: // inner helpers
    int doErode();
    int doDilate();
    int scanBorders(FourBorders & lines);
};

} // namespace Seg_Three

#endif // _BOUNDARY_SCAN_H_
