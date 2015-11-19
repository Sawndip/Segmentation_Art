#ifndef _THREE_DIFF_H_
#define _THREE_DIFF_H_

// sys
#include <string>
#include <vector>
#include <queue>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segMisc.h"
#include "vectorSpace.h"

// namespace
using :: std :: string;
using :: std :: vector;
using :: std :: queue;
using namespace Vector_Space;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
class ThreeDiff // San Fen
{
public:
    ThreeDiff();
    ~ThreeDiff();    

private:
    // We try to know the whole lifespan of the moving object, even it is stopped for a
    // while, or it is disguised by something;
    //
    // psosegment is just for codebook & collectiveWiddom
    // three diff will do a lot more:
    // 1. accurately locate the object
    // 2. boundary detect;
    // 3. simple optical flow; direction detect;
    // 4. three frames do diff, do '&', do simple erode/dilate;
private:
    // 1. general 
    bool m_bInit;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    // 2. cache related
    const int M_THREE_DIFF_CACHE_FRAMES = 2;
    int m_curFrontIdx;
    cv::Mat m_cacheFrames[M_THREE_DIFF_CACHE_FRAMES];
    cv::Mat m_bookResults[M_THREE_DIFF_CACHE_FRAMES];
    cv::Mat m_diffResults[M_THREE_DIFF_CACHE_FRAMES];
    vector<std:tuple<TDPoint, TDPoint> > m_crossLines[M_THREE_DIFF_CACHE_FRAMES];    
    // 3. contourTrack part (simple optical flow & feature extraction)
    int m_objIdx;
    vector<ContourTrack *> m_tracks;


private: // inner helpers
    
    
};

} // namespace Seg_Three

#endif // _THREE_DIFF_H_
