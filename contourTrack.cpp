#include "math.h"
#include "contourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int idx, const cv::Mat & in,
                             const int width, const int height,
                             const int directionIn,
                             const int lux, const int luy,                         
                             const int possibleWidth, const int possibleHeight)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_inputFrames(0)
    , m_bAllIn(false)
    , m_bAllOut(false)
    , m_lux(lux)
    , m_luy(luy)
    , m_xCenter(0)
    , m_yCenter(0)
    , m_curWidth(possibleWidth)
    , m_curHeight(possibleHeight)
    , m_largestWidth(possibleWidth)
    , m_largestHeight(possibleHeight)
    , m_inDirection((DIRECTION)directionIn)
    , m_outDirection(DIRECTION_UNKNOWN)      
{
    assert(m_curWidth > 0 && m_curHeight > 0);
    // 1. may not used, but keep it right now
    m_xCenter = m_lux + m_curWidth / 2;
    m_yCenter = m_luy + m_curHeight / 2;
    // 2. calculate the size changing function.
    // take as function: y = ax^2 + bx + c
    // We know: x=0, y=1; x=20, y=0.5; x=m_imgWidth, y=0;
    // so: 400a + 20b = 1.5
    m_aw = (1.5 * m_imgWidth - 20) / (400.0 * m_imgWidth - 20 * m_imgWidth * m_imgWidth);
    m_bw = (1.5 - 400 * m_aw) / 20.0;
    m_ah = (1.5 * m_imgHeight - 20) / (400.0 * m_imgHeight - 20 * m_imgHeight * m_imgHeight);
    m_bw = (1.5 - 400 * m_ah) / 20.0;
    // 2. compressive tracker part.
    m_curBox = cv::Rect(m_lux, m_luy, m_curWidth, m_curHeight);
    m_ctTracker.init(in, m_curBox); //ct.init(grayImg, box);

    LogI("Create New ContourTrack %d: InDirection: %d, lux:%d, luy:%d, possibleWidth:%d, "
         "possibleHeight:%d, centerX:%d, centerY:%d\n", m_idx, 
         directionIn, lux, luy, possibleWidth, possibleHeight, m_xCenter, m_yCenter);
    return;
}

ContourTrack :: ~ContourTrack()
{    
    return;        
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs    
int ContourTrack :: processFrame(const cv::Mat & in, const cv::Mat & bgResult,
                                 const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // Process frame using compressive tracker.
    int ret = m_ctTracker.processFrame(in, m_curBox);
    if (ret  < 0)
        printf("Tracker warning \n.");
    return 0;
}

int ContourTrack :: flushFrame(cv::Mat & out)
{
    
    return 0;
}; 
    
//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers
int ContourTrack :: curMaxChangeSize(int & x, int & y)
{
    const double xRate = m_aw * m_curWidth * m_curWidth + m_bw * m_curWidth + m_c;
    const double yRate = m_aw * m_curHeight * m_curHeight + m_bw * m_curHeight + m_c;
    x = (int)round(m_curWidth * xRate);
    y = (int)round(m_curHeight * yRate);    
    return 0;
}
    
} // namespace Seg_Three
