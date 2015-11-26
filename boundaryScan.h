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
    int init(const int width, const int height
             const int skipTB, const int skipLR,
             const int scanSizeTB, const int scanSizeLR);
    int processFrame(const BgResult & bgResult, FourBorders & lines);

private: // inner classes
    class BordersMem
    {
    public:
        BordersMem()
        {
            bzero(directions, sizeof(directions));
        };
        ~BordersMem()
        {
            for (int k = 0; k < BORDER_NUM; k++)
                if (directions[k])
                    delete [] directions[k];
        }
        void init(const int _widthTB, const int _heightTB,
                  const int _widthLR, const int _heightLR)
        {
            widthTB = _widthTB;
            heightTB = _heightTB;                
            widthLR = _widthLR;
            heightLR = _heightLR;                
            m_directions[0] = new unsigned char[widthTB * heightTB];
            m_directions[1] = new unsigned char[widthTB * heightTB];
            m_directions[2] = new unsigned char[widthLR * heightLR];
            m_directions[3] = new unsigned char[widthLR * heightLR];
        }        
    public:
        int widthTB;
        int widthLR;
        int heightTB;
        int heightLR;
        // top bottom left right
        unsigned char *m_directions[BORDER_NUM];        
    };

private: // inner members
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    int m_skipTB;
    int m_skipLR;
    int m_scanSizeTB;
    int m_scanSizeLR;
    
    BordersMem m_bordersMem;
    int m_objIdx;
    static const int M_ELEMENT_WIDTH = 2;    
    static const int M_ELEMENT_HEIGHT = 2;

private: // inner helpers
    int doErode();
    int doDilate();
    int scanBorders(FourBorders & lines);
};

} // namespace Seg_Three

#endif // _BOUNDARY_SCAN_H_
