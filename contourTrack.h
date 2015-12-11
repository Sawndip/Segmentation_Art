#ifndef _CONTOUR_TRACK_H_
#define _CONTOUR_TRACK_H_

// sys
#include <string>
#include <vector>
#include <queue>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// project
#include "segUtil.h"
#include "vectorSpace.h"
#include "CompressiveTracker.h"

// namespace
using :: std :: string;
using :: std :: vector;
using :: std :: queue;
using namespace Vector_Space;
using namespace Compressive_Tracker;

namespace Seg_Three
{
    
class ContourTrack
{   
public:
    ContourTrack(const int idx, const int width, const int height, // image width/height
                 const int skipTB, const int skipLR, const int takeFrameInterval, 
                 const int directionIn, const TDLine & theLine, const cv::Rect & firstBox, 
                 const int firstAppearFrameCount, const cv::Mat & in, BgResult & bgResult);
    
    ~ContourTrack();
    // 1. APIs
    int processFrame(const cv::Mat & in, const cv::Mat & lastIn, BgResult & bgResult,
                     const bool bGoodTimeToUpdate);
    int flushFrame();
    
    // 2. trival ones
    int getIdx() const {return m_idx;}
    cv::Rect & getCurBox() {return m_curBox;}
    cv::Rect & getLastBox() {return m_lastBox;}    
    bool canOutputRegion() {return m_bOutputRegion;}
    int getFirstAppearFrameCount() {return m_firstAppearFrameCount;}    
    MOVING_DIRECTION getInDirection(){return m_inDirection;}
    void setInDirection(const MOVING_DIRECTION d){m_inDirection = d;}    
    MOVING_DIRECTION getOutDirection(){return m_outDirection;}
    MOVING_STATUS getMovingStatus() {return m_movingStatus;}
    void setMovingStatus(MOVING_STATUS newMS) {m_movingStatus = newMS;}
    void setLastBoundary(const int bdNum, const TDLine & line) {
        m_lastBoundaryLines[bdNum] = line;
    }
    
private: // members
    const int m_idx;
    const int m_imgWidth;
    const int m_imgHeight;
    const int m_skipTB;
    const int m_skipLR;
    const int m_takeFrameInterval;    
    int m_inputFrames;
    const int m_firstAppearFrameCount;
    bool m_bOutputRegion;
    int m_maxEnlargeDx;
    int m_maxEnlargeDy;
    int m_maxShrinkDx;
    int m_maxShrinkDy;
    
    // 1. using CompressiveTrack as tracker
    cv::Rect m_lastBox;
    cv::Rect m_curBox;
    CompressiveTracker *m_ctTracker;    
    // some internal status
    MOVING_DIRECTION m_inDirection;
    MOVING_DIRECTION m_outDirection;
    MOVING_STATUS m_movingStatus;
    bool m_bMovingStop; // MOVING_STOP is an assist status, along with other three.
    int m_allInCount;
    int m_allOutCount;
    int m_crossOutCount;
    queue<int> m_lastConsumeLinesResults; // (CONSUME_LINE_RESULT)
    queue<double> m_lastBoxesFitness; // the fitness of the last boxes
    TDLine m_lastBoundaryLines[BORDER_NUM]; // may have one or two boundary lines simultaneously
    int m_movingInStatusChangingThreshold;
    int m_movingOutStatusChangingThreshold;    

private: // inner important helpers
    int processOneBoundaryLine(const int bdNum, TDLine & theLine, BgResult & bgResult);
    double doTrackUpdate(const cv::Mat & in, const cv::Mat & lastIn,
                         BgResult & bgResult, const bool bGoodTimeToUpdate);    
    int doStatusChanging(const int statusResult, const double fitness);
    int doEnlargeBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                               const int maxEnlargeDx, const int maxEnlargeDy);
    int doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                              const int maxShrinkDx, const int maxShrinkDy);
    int doShrinkBoxUsingImage2(const cv::Mat & image, cv::Rect & box,
                              const int maxShrinkDx, const int maxShrinkDy);
    
private: // inner trival ones
    vector<MOVING_DIRECTION> checkBoxApproachingBoundary(const cv::Rect & rect);
    cv::Rect estimateMinBoxByTwoConsecutiveLine (const int bdNum, const TDLine & lastLine,
        const TDLine & updateLine, const bool bCrossIn);
    int getConsumeResult(const vector<int> & results);
    int adjustBoxByBgResult(BgResult & bgResult, cv::Rect & baseBox,
                            const int maxEnlargeDx = 48, const int maxEnlargeDy = 48,
                            const int maxShrinkDx = 48, const int maxShrinkDy = 48);
    double getFitnessOfBox(const cv::Mat & image, const cv::Rect & box);
    int doBoxProtectionCalibrate(cv::Rect & box);    
};

}//namespace

#endif // _CONTOUR_TRACK_H_
