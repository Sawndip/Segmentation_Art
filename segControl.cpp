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
    bookResult.create(width, height, CV_8UC1);
    // 2. key members
    ret = psoBook.init(width, height);
    assert(ret >= 0);
    ret = boundaryScan(width, height);
    assert(ret >= 0);
    // put all complexities inside ThreeDiff
    ret = threeDiff.init(width, height);
    assert(ret >= 0);
    return 0;    
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
int SegControl :: processFrame(const cv::Mat & in, cv::Mat & out)
{   // we get psoBook's opinion
    int ret = -1;
    ret = psoBook.processFrame(in, out);
    assert(ret >= 0);
    // four directions
    vector<<tuple<TDPoint, TDPoint> > > possibleBoundaries = boundaryScan(out);
    // update boundary or add new contourTrack
    vector<Rect> rects;
    ret = threeDiff.processFrame(in, out, possibleBoundaries, rects);
    return ret;
}

int flushFrame(cv::Mat & out);
{
    return threeDiff.flushFrame(out);
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    
 
} // namespace Seg_Three
