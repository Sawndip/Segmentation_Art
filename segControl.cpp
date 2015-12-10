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
                       const int scanSizeTB, const int scanSizeLR,
                       const int takeFrameInterval)
{
    // 1. do all members' initialization
    int ret = -1;
    m_imgWidth = width;
    m_imgHeight = height;
    m_inputFrames = 0;
    m_skipTB = skipTB;
    m_skipLR = skipLR;
    m_scanSizeTB = scanSizeTB;
    m_scanSizeLR = scanSizeLR;
    // 2. key members
    ret = m_segBg.init(width, height, skipTB, skipLR, scanSizeTB, scanSizeLR, takeFrameInterval);
    assert(ret >= 0);
    // boundaryScan play an important role
    ret = m_boundaryScan.init(width, height, skipTB, skipLR,
                              scanSizeTB, scanSizeLR, takeFrameInterval);
    assert(ret >= 0);
    // put all complexities inside ThreeDiff
    ret = m_threeDiff.init(width, height, skipTB, skipLR,
                           scanSizeTB, scanSizeLR, takeFrameInterval);
    assert(ret >= 0);
    // 3. bgResults
    m_bgResult.binaryData.create(height, width, CV_8UC1);
    // top bottom left right
    m_bgResult.xMvs[0].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.xMvs[1].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.xMvs[2].resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.xMvs[3].resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.yMvs[0].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.yMvs[1].resize((width - 2*skipLR)*scanSizeTB);
    m_bgResult.yMvs[2].resize((height - 2*skipTB)*scanSizeLR);
    m_bgResult.yMvs[3].resize((height - 2*skipTB)*scanSizeLR);    
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Lib's APIs
int SegControl :: processFrame(const cv::Mat & in, vector<SegResults> & segResults)
{   
    int ret = -1;
    m_inputFrames++;
    // 1. fill the m_bgResult's binaryData/mvs by opticalFlow detection.
    ret = m_segBg.processFrame(in, m_bgResult.binaryData,
                               m_bgResult.xMvs, m_bgResult.yMvs, 0.6);
    //// 2. do erode/dilate on the whole image
    //const cv::Mat ker = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    //cv::Mat dst;
    //for(int k = 0; k < 2; k++)
    //{
    //    cv::dilate(m_bgResult.binaryData, dst, ker);
    //    cv::erode(m_bgResult.binaryData, dst, ker);
    //}
    //dst.copyTo(m_bgResult.binaryData);;
    // 3. if optical flow output frames, we further analyse them.
    if (ret > 0) // got a frame
    {   // 4. Fill the m_bgResult's four lines info by do simple erode & dilate on binaryData.
        //    Do pre-merge short-lines that we are sure they are the same objects.
        m_boundaryScan.processFrame(m_bgResult);
        // 5. all other stuff are doing by this call. Details are described in ThreeDiff class.
        ret = m_threeDiff.processFrame(in, m_bgResult, segResults);
        LogI("Frame %d: SegResults size: %d.\n", m_inputFrames, (int)segResults.size());
    }
    
    m_bgResult.reset(); // reset lines.
    return ret;
}

int SegControl :: flushFrame(vector<SegResults> & segResults)
{
    return m_threeDiff.flushFrame(segResults);
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    
 
} // namespace Seg_Three
