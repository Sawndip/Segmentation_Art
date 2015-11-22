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
    ContourTrack(const int idx,
                 const int width, const int height, // image width/height
                 const int directionIn,        
                 const int lux, const int luy, // first appear coordinate
                 const int possibleWidth, const int possibleHeight);
    ~ContourTrack();
    // read frame in and deliver to proper members
    int processFrame();    
    // when no new frames, we flush out cached frames
    int flushFrame(cv::Mat & out);

public:
    struct TrackFeatures
    {
        int m_xCentroid;
        int m_yCentroid;
        int m_width;
        int m_height;
        int m_r;
        int m_g;
        int m_b;
    };

private:
    const int m_idx;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
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
    // 3. using ?? as tracker


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
