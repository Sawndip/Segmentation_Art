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
    // three phrases of the contour tracker.
    // 1. the cross in phrase: compressive tracker is not crated until bAllIn is set.
    //                         using BoundaryCheck to track the objects.
    // 2. the normal track phrase: using compressive tracker to track the object.
    // 3. the cross out phrase: using BoundaryCheck to track the objects.
    // 4. different phrases can be told by MOVING_STATUS: CrossIn, Inside, CrossOut
    ContourTrack(const int idx, const cv::Mat & in,
                 const int width, const int height, // image width/height
                 const int skipTB, const int skipLR,
                 const int directionIn, const TDLine & theLine, 
                 const int lux, const int luy, // first appear coordinate
                 const int possibleWidth, const int possibleHeight,
                 const int firstAppearFrameCount);
    
    ~ContourTrack();
    // 1. important ones
    int processFrame(const cv::Mat & in, BgResult & bgResult, // may be modified 
                     const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int flushFrame();
    
    // 2. trival ones
    int getIdx() const {return m_idx;}
    cv::Rect & getCurBox() {return m_curBox;}
    cv::Rect & getLastBox() {return m_lastBox;}    
    bool canOutputRegion() {return m_bOutputRegion;}
    int getFirstAppearFrameCount() {return m_firstAppearFrameCount;}
    MOVING_DIRECTION getInDirection(){return m_inDirection;}
    MOVING_DIRECTION getOutDirection(){return m_outDirection;}
    MOVING_STATUS getMovingStatus() {return m_movingStatus;}
    void setMovingStatus(MOVING_STATUS newMS) {m_movingStatus = newMS;}
    
private:
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

private: // inner helpers
    int markAcrossIn(const vector<MOVING_DIRECTION> & directions,
                     BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int markAcrossOut(const vector<MOVING_DIRECTION> & directions,
                     BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int updateCrossInBox(const int bdNum, TDLine & updateLine, BgResult & bgResult,
                         const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int updateCrossOutBox(const int bdNum, TDLine & updateLine, BgResult & bgResult,
                          const cv::Mat & diffAnd, const cv::Mat & diffOr);
    // helper to kick out untraced line but actually belong to exist tracker
    int updateUntracedIfNeeded(const int bdNum, TDLine & updateLine);
    // important ones    
    cv::Rect getMaxCrossBoxUsingDiff(const BgResult & bgResult,
                                     const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int doEnlargeBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                               const int maxEnlargeDx, const int maxEnlargeDy);
    int doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                              const int maxShrinkDx, const int maxShrinkDy);
    
private:    
    // trival ones
    int curMaxChangeSize(int & x, int & y);
    double calcOverlapRate(const cv::Rect & a, const cv::Rect & b);
    cv::Rect calcOverlapArea(const cv::Rect & a, const cv::Rect & b);
    void enlargeBoxByMinBox(cv::Rect & box, const cv::Rect & minBox);    
    void boundBoxByMaxBox(cv::Rect & box, const cv::Rect & maxBox);    
    vector<MOVING_DIRECTION> checkBoxApproachingBoundary(const cv::Rect & rect);
    int estimateShiftByTwoConsecutiveLine(int & xShift, int & yShift, 
         int & widthShift, int & heightShift, const int bdNum,
         const TDLine & lastLine, const TDLine & updateLine);
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
