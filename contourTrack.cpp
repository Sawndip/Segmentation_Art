#include "math.h"
#include "contourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int idx, const cv::Mat & in,
                             const int width, const int height,
                             const int directionIn, const TDLine & theLine,
                             const int lux, const int luy,                         
                             const int possibleWidth, const int possibleHeight,
                             const int firstAppearFrameCount)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_inputFrames(0)
    , m_firstAppearFrameCount(firstAppearFrameCount)
    , m_bAllIn(false)
    , m_bAllOut(false)
    , m_bOutputRegion(false)
    , m_lastBox(lux, luy, possibleWidth, possibleHeight)
    , m_curBox(m_lastBox)
    , m_ctTracker(NULL)
    , m_largestWidth(possibleWidth)
    , m_largestHeight(possibleHeight)
    , m_inDirection((MOVING_DIRECTION)directionIn)
    , m_outDirection(DIRECTION_UNKNOWN)
    , m_movingStatus(MOVING_CROSS_IN)
    , m_bMovingStop(false)
{
    // TODO: may need further polishing.
    m_lastBoundaryLines.clear();
    m_lastBoundaryLines.push_back(theLine);
    
    assert(width > 0 && height > 0);
    // 1. calculate the size changing function.
    // take as function: y = a1x + b1 & y = a2x + b2,
    // with x=0, y=1; x=20, y=0.5, x=imgWidth, y=0;
    m_a1 = -0.5 / m_halfChangingValue;
    m_b1 = 1.0;
    m_a2w = 0.5 / (m_halfChangingValue - width);
    m_a2w = -m_a2w * width;
    m_a2h = 0.5 / (m_halfChangingValue - height);
    m_a2h = -m_a2h * height;
    
    // 2. compressive tracker part.
    // won't init until bAllIn is set, namely MOVING_STATUS changes from CROSS_IN to INSIDE.
    // m_ctTracker = new CompressiveTracker();
    // m_ctTracker->init(in, m_curBox); //ct.init(grayImg, box);
    
    LogI("Create New ContourTrack %d: InDirection: %d, lux:%d, luy:%d, possibleWidth:%d, "
         "possibleHeight:%d. \n", m_idx, 
         directionIn, m_curBox.x, m_curBox.y, m_curBox.width, m_curBox.height);
    return;
}

ContourTrack :: ~ContourTrack()
{    
    return;        
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs    
int ContourTrack :: processFrame(const cv::Mat & in, BgResult & bgResult,
                                 const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // Process frame using compressive tracker.
    m_inputFrames++;
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;
    // if the box is close to boudary, the tracker may moving out.
    vector<MOVING_DIRECTION> directions = checkBoxApproachingBoundary(m_curBox);
    
    // BE AWARE: 1) tracer has moving status(in out inside).
    //           2) each boundary line also has moving status.
    bool bCTTracking = false;
    switch(m_movingStatus)
    {
    // 1. MOVING_INSIDE trackers just do untraced & MOVING_CROSS_OUT checking            
    case MOVING_INSIDE: 
        // TODO: magic number subsitution needed. 10frames * 10pixel = 100 pixles
        if (m_inputFrames >= 10 && directions.size() > 0)
        {   // line's movingDirection should be MOVING_CROSS_OUT
            markAcrossOut(directions, resultLines);
            // change moving status of the tracker
        }
        else // do normal tracking.
            bCTTracking = true;   
        break;
    case MOVING_CROSS_OUT:
    //  use boundary info to track, won't use compressive tracker
        if (markAcrossOut(directions, resultLines) < 0)
        {  // if all out, we tell the called to terminal the whole tracking.
            return 1; // 1 means terminal the tracking.
        }
        break;
    case MOVING_CROSS_IN:
        if (markAcrossIn(directions, resultLines) < 0)
        {  // if all in, use normal tracking.
            bCTTracking = true;
        }
        break;
    // case MOVING_STOP: this is not a valid status for track's moving status.
    default:
        LogE("Not valid moving status.\n");
        break;
    }

    //// normal tracking
    if (bCTTracking == true)
    {
        if (m_ctTracker == NULL)
        {
            m_ctTracker = new CompressiveTracker();
            m_ctTracker->init(in, m_curBox);
        }
        assert(m_ctTracker != NULL);
        if (m_ctTracker->processFrame(in, m_curBox) < 0)
        {
            LogW("Compressive Tracker do warning a failing track.\n.");
            // TODO: how to do update ? just terminate the tracking right now.
            return 1;
        }
    }
    
    return 0;
}
    
int ContourTrack :: flushFrame()
{
    
    return 0;
}; 
    
//////////////////////////////////////////////////////////////////////////////////////////
//// Internal Helpers: important ones

// 1. when do re-calc the curBox, we tend to get it a little bigger.
// 2. then we use diffOrResult & curMaxChangeSize to limit the expand of the box size.
int ContourTrack :: updateTrackerUsingDiff(const cv::Mat & in, const BgResult & bgResult,
                                           const cv::Mat & diffAnd, const cv::Mat & diffOr)
{ 
    int ret = 0;
    // 1. we use this dx dy and diffOr to get the possible maxium box
    int dx = 0, dy = 0;
    curMaxChangeSize(dx, dy);
    int lux = m_curBox.x - dx;
    int luy = m_curBox.y - dy;
    int rbx = m_curBox.x + m_curBox.width + dx;
    int rby = m_curBox.y + m_curBox.height + dy;
    if (lux < 0) lux = 0;
    if (luy < 0) luy = 0;
    if (rbx > m_imgWidth) rbx = m_imgWidth-1;
    if (rby > m_imgHeight) rby = m_imgHeight-1;
    if (rbx <= lux || rby < luy)
    LogW("%d-%d-%d-%d, %d-%d-%d-%d. dx-%d,dy-%d\n",rbx,lux,rby,luy,m_curBox.x,m_curBox.y,
          m_curBox.width, m_curBox.height, dx, dy);
    //assert(rbx > lux && rby > luy);
    cv::Rect maxBox(lux, luy, rbx - lux, rby - luy);
    if (maxBox.width % 2 != 0) maxBox.width--;
    if (maxBox.height % 2 != 0) maxBox.height--;
    // a). shrink the max box using 'diffOr' to get the possible maxium box.
    doShrinkBoxUsingImage(diffOr, maxBox);
    // b). then shrink the max box using new bgResult
    doShrinkBoxUsingImage(bgResult.binaryData, maxBox);    
    // c). calculate the minimal area that needed.
    cv::Rect minBox = calcOverlapArea(m_lastBox, m_curBox);
    // make the max box at least contain the min box.
    boundBoxByMinBox(maxBox, minBox);

    /* Until Now, we get the next box */
    m_curBox = maxBox;
    if (m_largestWidth < m_curBox.width)
        m_largestWidth = m_curBox.width;
    if (m_largestHeight < m_curBox.height)
        m_largestHeight = m_curBox.height;
    
    // 2. check the bAllIn (enter border)
    if (m_bAllIn == false)
    {
        if (m_curBox.x >= 2 && m_curBox.x + m_curBox.width < m_imgWidth &&
            m_curBox.y >= 2 && m_curBox.y + m_curBox.height < m_imgHeight )
            m_bAllIn = true;
    }

    // TODO: PXT: Bug here, could leave from topleft or topright, namely the corner, but
    // we cannot deal with this situation, may fix it later after do some tests.
    if (m_curBox.width <= 4 || m_curBox.height <= 4)
    {
        m_bAllOut = true;
        ret = 1;
        if (m_curBox.x < 4) m_outDirection = LEFT;
        if (m_imgWidth - m_curBox.x - m_curBox.width < 4) m_outDirection = RIGHT;
        if (m_curBox.y < 4) m_outDirection = TOP;
        if (m_imgHeight - m_curBox.y - m_curBox.height < 4) m_outDirection = BOTTOM;
    }
        
    return ret;
}

// box's width & height must be an even number.
int ContourTrack :: doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box)
{   // using a 2x2 window do scaning the image from the border of the box
    // 1. top
    int k = 0;
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   // find a 2x2 area with all '255' (foreground).
            if (image.at<uchar>(box.x + k, box.y + j)     &
                image.at<uchar>(box.x + k, box.y + j+1)   &
                image.at<uchar>(box.x + k+1, box.y + j)   &
                image.at<uchar>(box.x + k+1, box.y + j+1) )
                break; // find the boundary.
        }
        if (j < box.height)
            break;
    }
    // do update: 
    box.y += k;
    box.height -= k;
    
    // 2. bottom
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + k, box.y + box.height - j)     &
                image.at<uchar>(box.x + k, box.y + box.height - j-1)   &
                image.at<uchar>(box.x + k+1, box.y + box.height - j)   &
                image.at<uchar>(box.x + k+1, box.y + box.height - j -1))
                break; // find the boundary.
        }
        if (j > 0)
            break;
    }
    // do update: 
    box.height -= k;

    // 3. left
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + k, box.y + j)     &
                image.at<uchar>(box.x + k, box.y + j+1)   &
                image.at<uchar>(box.x + k+1, box.y + j)   &
                image.at<uchar>(box.x + k+1, box.y + j+1) )
                break; // find the boundary.
        }
        if (j < box.height)
            break;
    }
    box.x += k;
    box.width -= k;

    // 4. right
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + box.width - k, box.y + box.height - j)   &
                image.at<uchar>(box.x + box.width - k, box.y + box.height - j-1) &
                image.at<uchar>(box.x + box.width - k-1, box.y + box.height -j)  &
                image.at<uchar>(box.x + box.width - k-1, box.y + box.height -j-1))
                break; // find the boundary.
        }
        if (j < box.height)
            break;
    }
    box.width -= k;

    return 0;
}

int ContourTrack ::  markAcrossIn(const vector<MOVING_DIRECTION> & directions,
                                  vector<vector<TDLine> > & resultLines)
{

    return 0;
}

int ContourTrack ::  markAcrossOut(const vector<MOVING_DIRECTION> & directions,
                                   vector<vector<TDLine> > & resultLines)
{

    return 0;
}
    
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones

// shrink or dilate
int ContourTrack :: curMaxChangeSize(int & x, int & y)
{
    double xRate;
    double yRate;
    if (m_lastBox.width < m_halfChangingValue)
        xRate = m_a1*m_lastBox.width + m_b1;
    else
        xRate = m_a2w*m_lastBox.width + m_b2w;
    if (m_lastBox.height < m_halfChangingValue)
        yRate = m_a1*m_lastBox.height + m_b1;
    else
        yRate = m_a2h*m_lastBox.height + m_b2h;

    x = (int)round(m_lastBox.width * xRate);
    y = (int)round(m_lastBox.height * yRate);
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    return 0;
}

// prerequisite: width/height of the two rects are the same.    
double ContourTrack :: calcOverlapRate(cv::Rect & a, cv::Rect & b)
{
    assert(a.width == b.width && a.height == b.height);
    if (a.width == 0 || a.height == 0)
        return 0.0;
    cv::Rect overlapBox = calcOverlapArea(a, b);
    return (overlapBox.width * overlapBox.height * 1.0 / (a.width * a.height));
}

cv::Rect ContourTrack :: calcOverlapArea(cv::Rect & a, cv::Rect & b)
{
    if (a.x + a.width < b.x  || a.x > b.x + b.width ||
        a.y + a.height < b.y || a.y > b.y + b.height)
        return cv::Rect(0, 0, 0, 0);
    const int x = std::max(a.x, b.x);
    const int y = std::max(a.y, b.y);
    const int width = a.width + b.width -
                      (std::max(a.x+a.width, b.x+b.width) - std::min(a.x, b.x));
    const int height = a.height + b.height -
                       (std::max(a.y + a.height, b.y + b.height) - std::min(a.y, b.y));
    return cv::Rect(x, y, width, height);
}

void ContourTrack :: boundBoxByMinBox(cv::Rect & maxBox, const cv::Rect & minBox)
{
    if (maxBox.x > minBox.x)
        maxBox.x = minBox.x;
    if (maxBox.y > minBox.y)
        maxBox.y = minBox.y;
    if (maxBox.x + maxBox.width < minBox.x + minBox.width)
        maxBox.width = minBox.x + minBox.width - maxBox.x;
    if (maxBox.y + maxBox.height < minBox.y + minBox.height)
        maxBox.height = minBox.y + minBox.height - maxBox.y;    
    return;
}

// check box close to which boundary (should take skipTB,LR into account)    
vector<MOVING_DIRECTION> ContourTrack :: checkBoxApproachingBoundary(const cv::Rect & rect)
{
    vector<MOVING_DIRECTION> directions;
    
    return directions;
}
    
} // namespace Seg_Three
