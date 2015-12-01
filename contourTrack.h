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
                 const int directionIn, const TDLine & theLine, 
                 const int lux, const int luy, // first appear coordinate
                 const int possibleWidth, const int possibleHeight);
    ~ContourTrack();
    // 1. important ones
    int processFrame(const cv::Mat & in, const BgResult & bgResult,
                     const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int flushFrame();
    
    // 2. trival ones
    int getIdx() const {return m_idx;}
    int isAllIn() const {return m_bAllIn;}    
    cv::Rect & getCurBox() {return m_curBox;}
    cv::Rect & getLastBox() {return m_lastBox;}    
    bool canOutputRegion() {return m_bOutputRegion;}
    MOVING_DIRECTION getInDirection(){return m_inDirection;}
    TDLine & getLastBoudanryLine() {return m_lastBoundaryLine;}
    void setLastBoudanryLine(const TDLine & theLine) {m_lastBoundaryLine = theLine;}
    MOVING_STATUS getMovingStatus() {return m_movingStatus;}
    void setMovingStatus(MOVING_STATUS newMS) {m_movingStatus = newMS;}
        
private:
    const int m_idx;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    
    bool m_bAllIn; // some objects may not always in.
    bool m_bAllOut;
    bool m_bOutputRegion;
    
    // 1. using CompressiveTrack as tracker
    cv::Rect m_lastBox;
    cv::Rect m_curBox; // changing every time with diffResults' influence
    CompressiveTracker *m_ctTracker;    
    int m_largestWidth;
    int m_largestHeight;
    
    MOVING_DIRECTION m_inDirection;
    MOVING_DIRECTION m_outDirection;
    MOVING_STATUS m_movingStatus;
    TDLine m_lastBoundaryLine;
    // 4. size changing function
    double m_a1; 
    double m_b1;
    double m_a2w; 
    double m_b2w;
    double m_a2h; 
    double m_b2h;        
    static const int m_halfChangingValue = 20;

private: // inner helpers
    int updateTrackerUsingDiff(const cv::Mat & in, const BgResult & bgResult,
                               const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box);
    // trival ones
    int curMaxChangeSize(int & x, int & y);
    double calcOverlapRate(cv::Rect & a, cv::Rect & b);
    cv::Rect calcOverlapArea(cv::Rect & a, cv::Rect & b);
    void boundBoxByMinBox(cv::Rect & maxBox, const cv::Rect & minBox);
};

}//namespace

#endif // _CONTOUR_TRACK_H_
