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
//// BoundaryScan only deal with line level process.
// 1. to get boundary line info, 'scanLineBoundary' is called, and it won't do any analyse.
// 2. to merge close and similar lines, 'premergeLines' is called, it will do one frame level
//    analyse, to merge proper lines that belongs to one bigger line.
// 3. to further merge lines and get stabel result (avoid unnormal lines got from OpticalFlow)
//        
class BoundaryScan
{
public:
    BoundaryScan();
    ~BoundaryScan();
    int init(const int width, const int height,
             const int skipTB, const int skipLR,
             const int scanSizeTB, const int scanSizeLR,
             const int takeFrameInterval);
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
    #define M_ARC_THRESHOLD (M_PI / 2.0)    
    static const int M_BOUNDARY_SCAN_CACHE_LINES = 3;
    static const int M_ELEMENT_WIDTH = 2; // for simple erode/dilate
    static const int M_ELEMENT_HEIGHT = 2;

    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    int m_skipTB;
    int m_skipLR;
    int m_scanSizeTB;
    int m_scanSizeLR;
    int m_takeFrameInterval;    
    // for cross boundary analyse
    BordersMem m_bordersMem;
    int m_curFrontIdx;
    vector<vector<TDLine> > m_cacheLines[M_BOUNDARY_SCAN_CACHE_LINES];    

private: // important inner helpers
    int scanBoundaryLines(const BgResult & bgResult);
    int premergeLines(const BgResult & bgResult);
    int canLinesBeMerged(const TDLine & l1, const TDLine & l2, const TDLine & l3);
    int stableAnalyseAndMarkLineStatus(BgResult & bgResult);
    int outputLineAnalyseResultAndUpdate(BgResult & bgResult);
    int markPredecessorsRecursively(const int curIdx, const int bdNum, BgResult & bgResult,
                                    vector<TDLine> & curLines, vector<TDLine> & middleLines,
                                    vector<TDLine> & oldLines);
    int goMarking(const int bdNum, BgResult & bgResult,
                  TDLine & curLine, vector<TDLine> & middleLines, vector<TDLine> & oldLines);
                  

private: // trival inner helpers
    int doErode(const int times = 1);
    int doDilate(const int times = 1);
    int mergeOverlapOfOnePositionLines(vector<TDLine> & lines, const int curIdx);
    double getLineMoveAngle(const TDLine & l1,
                            const vector<double> & xMvs, const vector<double> & yMvs);
    int calcLineMovingStatus(const int bdBum, TDLine & line);
    inline bool isLineCloseEnough(const double diffAngle)
    {
        assert(diffAngle >= 0);
        if (diffAngle < M_ARC_THRESHOLD || (fabs(2*M_PI - diffAngle) < M_ARC_THRESHOLD))
            return true;
        return false;
    }
};

} // namespace Seg_Three

#endif // _BOUNDARY_SCAN_H_
