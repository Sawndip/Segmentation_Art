#ifndef _CONTOUR_TRACK_H_
#define _CONTOUR_TRACK_H_

// sys
#include <string>
#include <vector>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segMisc.h"
#include "vectorSpace.h"
#include "CompressiveTracker.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace Vector_Space;
using namespace Compressive_Tracker;

namespace Seg_Three
{
    
class ContourTrack
{    
public:
    ContourTrack(const int idx, const cv::Mat & in,
                 const int width, const int height, // image width/height
                 const int directionIn,        
                 const int lux, const int luy, // first appear coordinate
                 const int possibleWidth, const int possibleHeight);
    ~ContourTrack();
    // 1. important ones
    int processFrame(const cv::Mat & in);
    int updateTrackerUsingDiff(const cv::Mat & in, const cv::Mat & bgResult,
                               const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int flushFrame();
    
    // 2. trival ones
    int getIdx() const {return m_idx;}
    cv::Rect & getCurBox() {return m_curBox;}
    
private:
    const int m_idx;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;

    // 0. using CompressiveTrack as tracker
    cv::Rect m_curBox; // changing every time with diffResults' influence
    CompressiveTracker m_ctTracker;
    
    // 1. When create new: give the basic infos
    // 2. When do tracking, using Simplified Optical Flow & update inner status.
    bool m_bAllIn; // some objects may not always in.
    bool m_bAllOut;
    int m_lux;
    int m_luy;
    int m_xCenter;
    int m_yCenter;
    int m_curWidth;
    int m_curHeight;
    int m_largestWidth;
    int m_largestHeight;
    
    DIRECTION m_inDirection;
    DIRECTION m_outDirection;

    // 4. size changing function
    double m_aw; 
    double m_bw;
    double m_ah; 
    double m_bh;
    const static int m_c = -1; // y = ax^2 + bx + c
private: // inner helpers
    int curMaxChangeSize(int & x, int & y);
};

}//namespace

#endif // _CONTOUR_TRACK_H_
