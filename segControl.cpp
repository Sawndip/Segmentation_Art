#include "segControl.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
SegControl :: SegControl()
{
    return;
}

SegControl :: ~SegControl()
{
    return;        
}

int SegControl :: init(const int width, const int height)
{
    // 1. do all members' initialization
    int ret = -1;
    m_imgWidth = width;
    m_imgHeight = height;
    // 2. key members
    ret = m_segBg.init(width, height);
    assert(ret >= 0);
    ret = m_boundaryScan.init(width, height);
    assert(ret >= 0);
    // put all complexities inside ThreeDiff
    ret = m_threeDiff.init(width, height);
    assert(ret >= 0);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
int SegControl :: processFrame(const cv::Mat & in,
                               vector<SegResults> & segResults, cv::Mat & out)
{   // we get psoBook's opinion
    int ret = -1;
    ret = m_segBg.processFrame(in, out);
    assert(ret >= 0);
    // four directions
    FourBorders possibleBoundaries;
    m_boundaryScan.processFrame(out, possibleBoundaries);
    // update boundary or add new contourTrack
    vector<Rect> rects;
    ret = m_threeDiff.processFrame(in, out, possibleBoundaries, segResults);
    return ret;
}

int SegControl :: flushFrame(vector<SegResults> & segResults, cv::Mat & out)
{
    return m_threeDiff.flushFrame(segResults, out);
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    
 
} // namespace Seg_Three
