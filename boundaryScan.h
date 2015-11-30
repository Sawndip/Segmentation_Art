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
#include "segUtil.h"
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
    int init(const int width, const int height,
             const int skipTB, const int skipLR,
             const int scanSizeTB, const int scanSizeLR);
    int processFrame(BgResult & bgResult);

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
            directions[0] = new unsigned char[widthTB * heightTB];
            directions[1] = new unsigned char[widthTB * heightTB];
            directions[2] = new unsigned char[widthLR * heightLR];
            directions[3] = new unsigned char[widthLR * heightLR];
        }        
    public:
        int widthTB;
        int widthLR;
        int heightTB;
        int heightLR;
        // top bottom left right
        unsigned char *directions[BORDER_NUM];        
    };

private: // inner members
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    int m_skipTB;
    int m_skipLR;
    int m_scanSizeTB;
    int m_scanSizeLR;
    // for cross boundary analyse
    BordersMem m_bordersMem;
    vector<TDLine> m_lastLines[BORDER_NUM];
    
    // for simple erode/dilate
    static const int M_ELEMENT_WIDTH = 2;    
    static const int M_ELEMENT_HEIGHT = 2;

private: // important inner helpers
    int scanBoundaryLines(BgResult & bgResult);
    int premergeLines(BgResult & bgResult);
    int updateLineMovingStatus(BgResult & bgResult, const int index);
    int canLinesBeMerged(const TDLine & l1, const TDLine & l2, const TDLine & l3);
    int calcLineMovingStatus(const double angle, const int index, TDLine & line);

private: // trival inner helpers
    int doErode(const int times = 1);
    int doDilate(const int times = 1);    
    double getLineMoveAngle(const TDLine & l1,
                            const vector<double> & xMvs, const vector<double> & yMvs);    
    inline bool isLineCloseEnough(const double diffAngle)
    {   // TODO: between 0 to 100? degree is taken as the similar
        static const double arcThreshold = M_PI * 1.0 / 180 * 100;    
        if (diffAngle < arcThreshold || (fabs(2*M_PI - diffAngle) < arcThreshold))
            return true;
        return false;
    }
        
};

} // namespace Seg_Three

#endif // _BOUNDARY_SCAN_H_
