#ifndef _CONTOUR_TRACK_H_
#define _CONTOUR_TRACK_H_

// sys
#include <string>
#include <vector>
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
using namespace Vector_Space;
using namespace Compressive_Tracker;

namespace Seg_Three
{
    
class ContourTrack
{   
public:
    ContourTrack(const int idx, const cv::Mat & in,
                 const int width, const int height, // image width/height
                 const int skipTB, const int skipLR,
                 const int directionIn, const TDLine & theLine, 
                 const cv::Rect & firstBox, 
                 const int firstAppearFrameCount);
    
    ~ContourTrack();
    // 1. APIs
    int processFrame(const cv::Mat & in, BgResult & bgResult,
                     const cv::Mat & diffAnd, const cv::Mat & diffOr);
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
    int m_imgWidth;
    int m_imgHeight;
    int m_skipTB;
    int m_skipLR;
    int m_inputFrames;    
    const int m_firstAppearFrameCount;
    bool m_bOutputRegion;
    
    // 1. using CompressiveTrack as tracker
    cv::Rect m_lastBox;
    cv::Rect m_curBox; // changing every time with diffResults' influence
    CompressiveTracker *m_ctTracker;    
    // some internal status
    int m_largestWidth;
    int m_largestHeight;    
    MOVING_DIRECTION m_inDirection;
    MOVING_DIRECTION m_outDirection;
    MOVING_STATUS m_movingStatus;
    bool m_bMovingStop; // MOVING_STOP is an assist status, along with other three.
    int m_allInCount;
    int m_allOutCount;
    int m_crossOutCount;    
    TDLine m_lastBoundaryLines[BORDER_NUM]; // may have one or two boundary lines simultaneously
    static const int M_MOVING_STATUS_CHANGING_THRESHOLD = 2;    

private: // inner important helpers
    int processOneBoundaryLine(const int bdNum, TDLine & theLine,
                        BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int doEnlargeBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                               const int maxEnlargeDx, const int maxEnlargeDy);
    int doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                              const int maxShrinkDx, const int maxShrinkDy);
    
private: // inner trival ones 
    vector<MOVING_DIRECTION> checkBoxApproachingBoundary(const cv::Rect & rect);
    cv::Rect estimateMinBoxByTwoConsecutiveLine (const int bdNum, const TDLine & lastLine,
        const TDLine & updateLine, const bool bCrossIn);
    int getConsumeResult(const vector<int> & results);
    int doStatusChanging(const int statusResult);
    int adjustCurBoxForCT(BgResult & bgResult);
};

}//namespace

#endif // _CONTOUR_TRACK_H_

/*
  Cross In Box Size Calculate
  1. When create, make it realy nice.
  2. When do cross in update, using diffOr do enlarge, then using BgResult do shrink.
     Both the above operations are with restrictions, namely each time can at most changing
     32 * tan(theta)
*/
