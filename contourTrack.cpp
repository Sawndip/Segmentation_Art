#include "ContourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack()
{
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
