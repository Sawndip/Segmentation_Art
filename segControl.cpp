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
    m_scanSizeTB = scanSizeTB;
    m_scanSizeLR = scanSizeLR;
    // 2. key members
    ret = m_segBg.init(width, height, skipTB, skipLR, scanSizeTB, scanSizeLR);
    assert(ret >= 0);
    // boundaryScan play an important role
    ret = m_boundaryScan.init(width, height, skipTB, skipLR, scanSizeTB, scanSizeLR);
    assert(ret >= 0);
    // put all complexities inside ThreeDiff
    ret = m_threeDiff.init(width, height);
    assert(ret >= 0);
    // 3. bgResults
    m_bgResults.binaryData.create(height, width, CV_8UC1);
    // top bottom left right
    m_bgResults.angles[0].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResults.angles[1].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResults.angles[0].resize((height - 2*skipTB)*scanSizeLR);
    m_bgResults.angles[1].resize((height - 2*skipTB)*scanSizeLR);    
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
int SegControl :: processFrame(const cv::Mat & in,
                               vector<SegResults> & segResults)
{   
    int ret = -1;
    ret = m_segBg.processFrame(in, m_bgResult.binaryData, m_bgResult.angles);
    assert(ret >= 0);
    if (ret > 0) // got a frame
    {    
        // four directions
        FourBorders possibleBorders(m_imgWidth-2*m_skipLR, m_scanSizeTB,
                                       m_imgHeight-2*m_skipTB, m_scanSizeLR);
        m_boundaryScan.processFrame(bgResult, possibleBorders);
        // update boundary or add new contourTrack
        ret = m_threeDiff.processFrame(in, bgResult, possibleBorders, segResults);
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
