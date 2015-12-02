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
    int isAllIn() const {return m_bAllIn;}    
    cv::Rect & getCurBox() {return m_curBox;}
    cv::Rect & getLastBox() {return m_lastBox;}    
    bool canOutputRegion() {return m_bOutputRegion;}
    int getFirstAppearFrameCount() {return m_firstAppearFrameCount;}
    MOVING_DIRECTION getInDirection(){return m_inDirection;}
    vector<TDLine> & getLastBoudanryLines() {return m_lastBoundaryLines;}
    MOVING_STATUS getMovingStatus() {return m_movingStatus;}
    void setMovingStatus(MOVING_STATUS newMS) {m_movingStatus = newMS;}
    
private:
    const int m_idx;
    int m_imgWidth;
    int m_imgHeight;
    int m_inputFrames;
    
    const int m_firstAppearFrameCount;
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
    bool m_bMovingStop; // MOVING_STOP is an assist status, along with other three.
    vector<TDLine> m_lastBoundaryLines; // may have one or two boundary lines simultaneously
    
    // 4. size changing function
    double m_a1; 
    double m_b1;
    double m_a2w; 
    double m_b2w;
    double m_a2h; 
    double m_b2h;        
    static const int m_halfChangingValue = 20;

private: // inner helpers
    // important ones
    int updateTrackerUsingDiff(const cv::Mat & in, const BgResult & bgResult,
                               const cv::Mat & diffAnd, const cv::Mat & diffOr);
    int doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box);
    int markAcrossIn(const vector<MOVING_DIRECTION> & directions,
                     vector<vector<TDLine> > & resultLines);    
    int markAcrossOut(const vector<MOVING_DIRECTION> & directions,
                      vector<vector<TDLine> >& resultLines);
    
    // trival ones
    int curMaxChangeSize(int & x, int & y);
    double calcOverlapRate(cv::Rect & a, cv::Rect & b);
    cv::Rect calcOverlapArea(cv::Rect & a, cv::Rect & b);
    void boundBoxByMinBox(cv::Rect & maxBox, const cv::Rect & minBox);
    vector<MOVING_DIRECTION> checkBoxApproachingBoundary(const cv::Rect & rect);
};

}//namespace

#endif // _CONTOUR_TRACK_H_
