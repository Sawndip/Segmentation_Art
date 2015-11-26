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

int SegControl :: init(const int width, const int height,
                       const int skipTB, const int skipLR,
                       const int scanSizeTB, const int scanSizeLR)
{
    // 1. do all members' initialization
    int ret = -1;
    m_imgWidth = width;
    m_imgHeight = height;
    m_skipTB = skipTB;
    m_skipLR = skipLR;
    m_scanBordSizeTB = scanSizeTB;
    m_scanBordSizeLR = scanSizeLR;
    // 2. key members
    ret = m_segBg.init(width, height, skipTB, skipLR, scanSizeTB, scanSizeLR);
    assert(ret >= 0);
    ret = m_boundaryScan.init(width, height, skipTB, skipLR, scanSizeTB, scanSizeLR);
    assert(ret >= 0);
    // put all complexities inside ThreeDiff
    ret = m_threeDiff.init(width, height);
    assert(ret >= 0);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
int SegControl :: processFrame(const cv::Mat & in,
                               vector<SegResults> & segResults,
                               cv::Mat & bgResult)
{   // we get psoBook's opinion
    int ret = -1;
    ret = m_segBg.processFrame(in, bgResult);
    assert(ret >= 0);
    if (ret > 0) // got a frame
    {    
        // four directions
        FourBorders possibleBoundaries(m_imgWidth, m_imgHeight);
        m_boundaryScan.processFrame(bgResult, possibleBoundaries);
        // update boundary or add new contourTrack
        ret = m_threeDiff.processFrame(in, bgResult, possibleBoundaries, segResults);
    }
    return ret;
}

int SegControl :: flushFrame(vector<SegResults> & segResults)
{
    return m_threeDiff.flushFrame(segResults);
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    
 
} // namespace Seg_Three
