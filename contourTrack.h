#ifndef _CONTOUR_TRACK_H_
#define _CONTOUR_TRACK_H_

// sys
#include <string>
#include <vector>
// project
#include "segMisc.h"
#include "vectorSpace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using namespace Vector_Space;

namespace Seg_Three
{
    
class ContourTrack
{    
public:
    ContourTrack(const int width, const int height,
                 const int possibleWidth, const int possibleHeight,
                 const );
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
    // 1. When create new: give the basic infos
    // 2. When do tracking, using Simplified Optical Flow & update inner status.
    bool m_bAllIn; // some objects may not always in.
    bool m_bOutputWholePic; 
    int m_xCenter;
    int m_yCenter;
    int m_curWidth;
    int m_curHeight;
    int m_largestWidth;
    int m_largestHeight;
    
    DIRECTION m_inDirection;
    DIRECTION m_OutDirection;
    int m_xmv;
    int m_ymv
    // 3. using KLT as tracker
            
};

}//namespace

#endif // _CONTOUR_TRACK_H_
