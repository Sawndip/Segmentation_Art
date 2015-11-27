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
    m_bgResult.binaryData.create(height, width, CV_8UC1);
    // top bottom left right
    m_bgResult.xMvs.resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.xMvs.resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.xMvs.resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.xMvs.resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.yMvs.resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.yMvs.resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.yMvs.resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.yMvs.resize((height - 2*skipTB)*scanSizeLR);    
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
int SegControl :: processFrame(const cv::Mat & in,
                               vector<SegResults> & segResults)
{   
    int ret = -1;
    // 1. fill the m_bgResult's binaryData/mvs by opticalFlow detection.
    ret = m_segBg.processFrame(in, m_bgResult.binaryData,
                               m_bgResult.xMvs, m_bgResult.yMvs);
    assert(ret >= 0);
    if (ret > 0) // got a frame
    {   // 2. Fill the m_bgResult's four lines info by do simple erode & dilate on binaryData.
        //    Do pre-merge short-lines that we are sure they are the same objects.
        m_boundaryScan.processFrame(m_bgResult);
        // 3. all other stuff are doing by this call. Details are described in ThreeDiff class.
        ret = m_threeDiff.processFrame(in, m_bgResult, segResults);
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
