#ifndef _THREE_DIFF_H_
#define _THREE_DIFF_H_

// sys
#include <string>
#include <vector>
#include <tuple>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segMisc.h"
#include "vectorSpace.h"
#include "contourTrack.h"

// namespace
using :: std :: string;
using :: std :: vector;
using :: std :: tuple;
using namespace Vector_Space;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
// We try to know the whole lifespan of the moving object, even it is stopped for a
// while, or it is disguised by something;
//
// psosegment is just for codebook & collectiveWiddom
// three diff will do a lot more:
// 1. accurately locate the object
// 2. boundary detect;
// 3. simple optical flow; direction detect;
// 4. three frames do diff, do '&', do simple erode/dilate;

class ThreeDiff // San Fen
{
public:
    ThreeDiff();
    ~ThreeDiff();    
    int init(const int width, const int height);
    int processFrame(const cv::Mat & in,
                     cv::Mat & bgResult, // also, it is the out binary frame.
                     vector<vector<tuple<TDPoint, TDPoint> > > & curFourLines,
                     vector<SegResults> & segResults);
    
    int flushFrame(vector<SegResults> & segResults, cv::Mat & bgResult);
    
private:
    // 1. general 
    bool m_bInit;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    
    // 2. cache related
    static const int M_THREE_DIFF_CACHE_FRAMES = 2;
    int m_curFrontIdx;
    cv::Mat m_diffAndResults[M_THREE_DIFF_CACHE_FRAMES];
    cv::Mat m_diffOrResults[M_THREE_DIFF_CACHE_FRAMES];    
    cv::Mat m_bgResults[M_THREE_DIFF_CACHE_FRAMES];
    vector<vector<tuple<TDPoint, TDPoint> > > m_crossLines[M_THREE_DIFF_CACHE_FRAMES];    

    // 3. contourTrack part (using compressiveTracker, then do postprocess with diffResults)
    int m_objIdx;
    vector<ContourTrack *> m_trackers;

private: // inner helpers
    // 1. important ones
    int doUpdateContourTracking(const cv::Mat in, cv::Mat & out,
                                FourBorders & curFourLines,
                                vector<SegResults> & segResults);
    int doCreateNewContourTrack(const cv::Mat & in, cv::Mat & out,
                                FourBorders & lines3,
                                vector<SegResults> & segResults);
    int updateAfterOneFrameProcess(const cv::Mat in, const cv::Mat & bgResult,
                                   const FourBorders & lines3);
    // 2. trival ones
    int doBgDiff(const cv::Mat & first, const cv::Mat & second);
};

} // namespace Seg_Three

#endif // _THREE_DIFF_H_
