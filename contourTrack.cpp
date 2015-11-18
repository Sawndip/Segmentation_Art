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
    , m_lux(0)
    , m_luy(0)
    , m_xCenter(0)
    , m_yCenter(0)
    , m_curWidth(possibleWidth)
    , m_curHeight(possibleHeight)
    , m_largestWidth(possibleWidth)
    , m_largestHeight(possibleHeight)
    , m_inDirection((DERECTION)derectionIn)
    , m_outDirection(DERECTION_UNKNOWN)      
{
    assert(m_curWidth > 0 && m_curHeight > 0);
        
        
  
    return;
}

ContourTrack :: ~ContourTrack()
{
    return;        
}

int ContourTrack :: init(const int idx, const int width, const int height)
{
    // do all members initialization
    m_idx = idx;
    m_imgWidth = width;
    m_imgHeight = height;
    m_inputFrames = 0;
        
    return 0;    
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
