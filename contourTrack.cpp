#include "ContourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int width, const int height,
                             const int directionIn,
                             const int lux, const int luy,                         
                             const int possibleWidth, const int possibleHeight)
    : m_imgWidth(width)
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
    , m_inDirection((DERECTION)directionIn)
    , m_outDirection(DERECTION_UNKNOWN)      
{
    assert(m_curWidth > 0 && m_curHeight > 0);
    m_xCenter = m_lux + m_curWidth / 2;
    m_yCenter = m_luy + m_curHeight / 2;    
    LogI("Create New ContourTrack: InDirection: %d, lux:%d, luy:%d, possibleWidth:%d, "
         "possibleHeight:%d, centerX:%d, centerY:%d\n",
         directionIn, lux, luy, possibleWidth, possibleHeight, m_xCenter, m_yCenter);  
    return;
}

ContourTrack :: ~ContourTrack()
{
    
    return;        
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs    
int ContourTrack :: processFrame()
{    
    return 0;
}

int flushFrame(cv::Mat & out);
{
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers    
    
} // namespace Seg_Three
