#include "math.h"
#include "contourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int idx, const cv::Mat & in,
                             const int width, const int height,
                             const int directionIn,
                             const int lux, const int luy,                         
                             const int possibleWidth, const int possibleHeight)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_inputFrames(0)
    , m_bAllIn(false)
    , m_bAllOut(false)
    , m_bOutputRegion(false)
    , m_lastBox(lux, luy, possibleWidth, possibleHeight)
    , m_curBox(m_lastBox)
    , m_largestWidth(possibleWidth)
    , m_largestHeight(possibleHeight)
    , m_inDirection((DIRECTION)directionIn)
    , m_outDirection(DIRECTION_UNKNOWN)      
{
    assert(width > 0 && height > 0);
    // 1. calculate the size changing function.
    // take as function: y = ax^2 + bx + c
    // We know: x=0, y=1; x=20, y=0.5; x=m_imgWidth, y=0;
    // so: 400a + 20b = -0.5
    const int halfChangingValue = 20; // 20 pixel, then can change 10 pixels.
    m_aw = (halfChangingValue- 0.5 * m_imgWidth) /
                  (halfChangingValue * halfChangingValue * m_imgWidth -
                   halfChangingValue * m_imgWidth * m_imgWidth);
    m_bw = (-0.5 - halfChangingValue * halfChangingValue * m_aw) / halfChangingValue;
    m_ah = (halfChangingValue - 0.5 * m_imgHeight) /
            (halfChangingValue * halfChangingValue * m_imgHeight -
             halfChangingValue * m_imgHeight * m_imgHeight);
    m_bh = (-0.5 - halfChangingValue * halfChangingValue * m_ah) / halfChangingValue;
    
    // 2. compressive tracker part.
    m_ctTracker = new CompressiveTracker();
    m_ctTracker->init(in, m_curBox); //ct.init(grayImg, box);
    
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
int ContourTrack :: processFrame(const cv::Mat & in, const cv::Mat & bgResult,
                                 const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // Process frame using compressive tracker.
    m_lastBox = m_curBox;
    int ret = m_ctTracker->processFrame(in, m_curBox);
    if (ret < 0)
        LogW("Compressive Tracker do warning a failing track.\n.");
    else
    {
        // re-calculate the m_curBox area,
        // also update the inner status of the object, such as bAllIn, bAllOut
        updateTrackerUsingDiff(in, bgResult, diffAnd, diffOr);
        // TODO: PXT: fix following reallocateiong.
        // prepare for the next processFrame call.
        delete m_ctTracker;
        m_ctTracker = new CompressiveTracker();
        m_ctTracker->init(in, m_curBox);
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
int ContourTrack :: updateTrackerUsingDiff(const cv::Mat & in, const cv::Mat & bgResult,
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
    assert(rbx > lux && rby > luy);
    cv::Rect maxBox(lux, luy, rbx - lux, rby - luy);
    if (maxBox.width % 2 != 0) maxBox.width--;
    if (maxBox.height % 2 != 0) maxBox.height--;
    // a). shrink the max box using 'diffOr' to get the possible maxium box.
    doShrinkBoxUsingImage(diffOr, maxBox);
    // b). then shrink the max box using new bgResult
    doShrinkBoxUsingImage(bgResult, maxBox);    
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
        {
            m_bAllIn = true;
            m_bOutputRegion = true;
        }
    }
    else
    {
        m_bOutputRegion = false; // we have already output it at least once.
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
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones

// shrink or dilate
int ContourTrack :: curMaxChangeSize(int & x, int & y)
{
    const double xRate = m_aw*m_lastBox.width*m_lastBox.width + m_bw*m_lastBox.width + m_c;
    const double yRate = m_ah*m_lastBox.height*m_lastBox.height + m_bh*m_lastBox.height + m_c;
    x = (int)round(m_lastBox.width * xRate);
    y = (int)round(m_lastBox.height * yRate);
    assert(x >= 0 && y >= 0);
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
    
} // namespace Seg_Three
