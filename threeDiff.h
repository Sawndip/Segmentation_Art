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
#include "segUtil.h"
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
    int init(const int width, const int height,
             const int skipTB, const int skipLR,
             const int scanSizeTB, const int scanSizeLR, const int takeFrameInterval);

    int processFrame(const cv::Mat & in, BgResult & bgResult,
                     vector<SegResults> & segResults);    
    int flushFrame(vector<SegResults> & segResults);
    
private:
    // 1. general 
    bool m_bInit;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    int m_skipTB;
    int m_skipLR;
    int m_scanSizeTB;
    int m_scanSizeLR;
    int m_takeFrameInterval;
    
    // 2. cache related
    int m_curFrontIdx;
    static const int M_THREE_DIFF_CACHE_FRAMES = 2;
    cv::Mat m_cacheIn[M_THREE_DIFF_CACHE_FRAMES];
    // background binary data & border mv angle
    BgResult m_bgResults[M_THREE_DIFF_CACHE_FRAMES];
    
    // 3. contourTrack part (using compressiveTracker, then do postprocess with diffResults)
    int m_objIdx;
    vector<ContourTrack *> m_trackers;

private: // inner helpers
    // 1. important ones
    int contourTrackingProcessFrame(const cv::Mat & in, const cv::Mat & lastIn,
                                    BgResult & bgResult, vector<SegResults> & segResults);
    int doCreateNewContourTrack(const cv::Mat & in, BgResult & bgResult,
                                vector<SegResults> & segResults);
    // 2. trival ones
    int updateAfterOneFrameProcess(const cv::Mat in, const BgResult & bgResult);    
    int doBgDiff(const cv::Mat & first, const cv::Mat & second);
    int isGoodTimeToUpdateTrackerBoxes(vector<bool> & bGoodTime);
    double calcDistanceOfTwoRect(cv::Rect & box1, cv::Rect & box2);
};

} // namespace Seg_Three

#endif // _THREE_DIFF_H_
