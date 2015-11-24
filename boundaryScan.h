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
class BoundaryScan // San Fen
{
public:
    BoundaryScan();
    ~BoundaryScan();
    int init(const int width, const int height);
    int processFrame(const cv::Mat & in, FourBorders & lines);

private: // inner classes
    class Borders
    {
    public:
        Borders() : top(NULL), bottom(NULL), left(NULL),right(NULL) {};
        ~Borders()
        {
            if (top) delete [] top;
            if (bottom) delete [] bottom;
            if (left) delete [] left;
            if (right) delete [] right;            
        }
        void init(const int r, const int w)
        {
            rows = w;
            columns = rows;
            top = new unsigned char[columns * rows];
            bottom = new unsigned char[columns * rows];
            left = new unsigned char[columns * rows];
            right = new unsigned char[columns * rows];
        }
    public:
        int columns;
        int rows;
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
    Borders m_borders;
    static const int M_BORDER_ROWS = 2;
    static const int M_ELEMENT_WIDTH = 2;    
    static const int M_ELEMENT_HEIGHT = 2;
private: // inner helpers
    int doErode();
    int doDilate();
    int scanBorders(FourBorders & lines);
};

} // namespace Seg_Three

#endif // _BOUNDARY_SCAN_H_
