#include <algorithm>
#include <math.h>
#include "contourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int idx, const cv::Mat & in,
                             const int width, const int height,
                             const int skipTB, const int skipLR,
                             const int directionIn, const TDLine & theLine,
                             const int lux, const int luy,                         
                             const int possibleWidth, const int possibleHeight,
                             const int firstAppearFrameCount)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_skipTB(skipTB)
    , m_skipLR(skipLR)      
    , m_inputFrames(0)
    , m_firstAppearFrameCount(firstAppearFrameCount)
    , m_bAllIn(false) // may not be used
    , m_bAllOut(false) // may not be used
    , m_bOutputRegion(false) // may not be used
    , m_lastBox(lux, luy, possibleWidth, possibleHeight)
    , m_curBox(m_lastBox)
    , m_ctTracker(NULL)
    , m_largestWidth(possibleWidth) // may not be used
    , m_largestHeight(possibleHeight) // may not be used
    , m_inDirection((MOVING_DIRECTION)directionIn)
    , m_outDirection(DIRECTION_UNKNOWN)
    , m_movingStatus(MOVING_CROSS_IN)
    , m_bMovingStop(false) // may not be used
    , m_allInCount(0)
{
    // line's direction is not quit the same as the tracker's moving direction
    assert(theLine.movingDirection < 4);
    m_lastBoundaryLines[(int)theLine.movingDirection] = theLine;
    
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
        
    LogI("Create New ContourTrack %d: InDirection: %d, lux:%d, luy:%d, initWidth:%d, "
         "initHeight:%d. \n", m_idx, 
         directionIn, m_curBox.x, m_curBox.y, m_curBox.width, m_curBox.height);
    return;
}

ContourTrack :: ~ContourTrack()
{   
    return;        
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs
    
//// Process frame using compressive tracker or Boundary Info    
int ContourTrack :: processFrame(const cv::Mat & in, BgResult & bgResult,
                                 const cv::Mat & diffAnd, const cv::Mat & diffOr)
{   
    m_inputFrames++;
    m_lastBox = m_curBox;    
    // if the box is close to boudary, the tracker may moving out.
    vector<MOVING_DIRECTION> directions = checkBoxApproachingBoundary(m_curBox);
    // BE AWARE: 1) tracer has moving status(in out inside).
    //           2) each boundary line also has moving status.
    
    bool bCTTracking = false; // use Compressive Tracking or BoundaryInfo tracking.
    switch(m_movingStatus)
    {
    // 1. MOVING_INSIDE trackers just do untraced & MOVING_CROSS_OUT checking            
    case MOVING_INSIDE: 
        // TODO: magic number subsitution needed. 10frames * 10pixel = 100 pixles
        if (m_inputFrames >= 10 && directions.size() > 0)
            // line's movingDirection should be MOVING_CROSS_OUT            
            markAcrossOut(directions, bgResult, diffAnd, diffOr); 
        else // do normal tracking.
            bCTTracking = true;   
        break;
    case MOVING_CROSS_OUT:
    //  use boundary info to track, won't use compressive tracker
        if (markAcrossOut(directions, bgResult, diffAnd, diffOr) < 0)
            // if all out, we tell the called to terminal the whole tracking.
            return 1; // 1 means terminal the tracking.
        break;
    case MOVING_CROSS_IN:
        if (markAcrossIn(directions, bgResult, diffAnd, diffOr) < 0)
            // if all in, use normal tracking.
            bCTTracking = true;
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
int ContourTrack ::  markAcrossIn(const vector<MOVING_DIRECTION> & directions,
                                  BgResult & bgResult,
                                  const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    assert(directions.size() <= 2);
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;    
    bool bStillCrossing = false;
    // how to make the object moving in & we update the rectBox gradully?    
    for (int bdNum = 0; bdNum < BORDER_NUM; bdNum++)
    {
        auto it = std::find(directions.begin(), directions.end(), bdNum);
        if (it != directions.end())
        {
            for (int k = 0; k < (int)resultLines[bdNum].size(); k++)
            {
                if (resultLines[bdNum][k].movingStatus == MOVING_CROSS_IN)
                {   
                    if (resultLines[bdNum][k].mayPreviousLineStart.x != -1 &&
                        resultLines[bdNum][k].mayPreviousLineEnd.x != -1)
                    {   // have previous line
                        if (resultLines[bdNum][k].mayPreviousLineStart.x ==
                                m_lastBoundaryLines[bdNum].a.x &&
                            resultLines[bdNum][k].mayPreviousLineEnd.x ==
                                m_lastBoundaryLines[bdNum].b.x)
                        {   // the predecessor is found, so update curBox using the line
                            bStillCrossing = true;
                            updateCrossInBox(bdNum, resultLines[bdNum][k],
                                             bgResult, diffAnd, diffOr);
                            m_lastBoundaryLines[bdNum] = resultLines[bdNum][k];
                        }
                        // TODO: do we need a else here ??????
                    }
                    else 
                    {   // this untraced line may part of our object, let's check it.
                        if (updateUntracedIfNeeded(bdNum, resultLines[bdNum][k]) > 0)
                        {
                            bStillCrossing = true;
                            updateCrossInBox(bdNum, resultLines[bdNum][k],
                                             bgResult, diffAnd, diffOr);
                            m_lastBoundaryLines[bdNum] = resultLines[bdNum][k];
                        }
                    }
                }
            }
        }
    }
    // all in
    if (bStillCrossing == false)
    {
        m_allInCount++;
        if (m_allInCount >= 2) // TODO: magic number: all in threshold
        {// reset lastBoundaryLines
            for (int k = 0; k < BORDER_NUM; k++)
                m_lastBoundaryLines[k] = TDLine();
            // mark we are inside!
            m_movingStatus = MOVING_INSIDE;
            return -1;
        }
    }
    return 0;
}

/*************************
We have Three Steps to get a new proper cross in box(m_curBox).
1. using DiffOr DiffAnd, m_lastBox to get the max & min box.
2. using m_lastBoundaryLines[bdNum]'s angle to estimate the x, y shift.
3. using current new boundary line(updateLine) to further update the estimated line.
*************************/
int ContourTrack :: updateCrossInBox(const int bdNum, TDLine & updateLine,
                                     BgResult & bgResult,
                                     const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;    
    // 1. first we use lastBox & diffResult to get the max box
    cv::Rect maxBox = getMaxCrossBoxUsingDiff(bgResult, diffAnd, diffOr);
    // 2. calculate the minimal area that needed(get min box)
    cv::Rect minBox = calcOverlapArea(m_lastBox, maxBox);
    
    // 3. then use lastBoundaryLine[bdNum] (must have, for we are in updateTracker)
    //    angle to get the possible x, y shift
    TDLine & lastLine = m_lastBoundaryLines[bdNum];
    int xShift = 0, yShift = 0;
    getShiftByTwoConsecutiveLine(xShift, yShift, bdNum, lastLine, updateLine);

    maxBox.x = (maxBox.x + m_lastBox.x + xShift) / 2;
    maxBox.y = (maxBox.y + m_lastBox.y + xShift) / 2;
    maxBox.width = maxBox.width + m_lastBox.width / 2; // take it as no width changing
    maxBox.height = maxBox.height + m_lastBox.height / 2; // take it as no height changing
    
    // 4. make the max box at least contain the min box.
    boundBoxByMinBox(maxBox, minBox);
    
    /* 5. Until Now, we get the next box */
    m_curBox = maxBox;
    if (m_largestWidth < m_curBox.width)
        m_largestWidth = m_curBox.width;
    if (m_largestHeight < m_curBox.height)
        m_largestHeight = m_curBox.height;
    
    return 0;
}    

// return > 0: need update this line & mark this line's previousLine
int ContourTrack :: updateUntracedIfNeeded(const int bdNum, TDLine & updateLine)
{
    TDLine boundaryLine = m_lastBoundaryLines[bdNum];
    if (boundaryLine.a.x == -1 && boundaryLine.b.x == -1)
        boundaryLine = rectToBoundaryLine(bdNum, m_lastBox);
    const double consecutivityScore = consecutivityOfTwoLines(boundaryLine, updateLine);

    // TODO: magic number 75.0 here.
    if (consecutivityScore > 75.0)
    {
        updateLine.mayPreviousLineStart = boundaryLine.a;
        updateLine.mayPreviousLineEnd = boundaryLine.b;        
        return 1;
    }
    return 0;
}

/*
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
*/
    
int ContourTrack :: markAcrossOut(const vector<MOVING_DIRECTION> & directions,
                        BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr)
    
{
    
    return 0;
}

int ContourTrack :: getShiftByTwoConsecutiveLine(int & xShift, int & yShift, const int bdNum,
                                 const TDLine & lastLine, const TDLine & updateLine)
{
    if (lastLine.a.x == -1 && lastLine.b.x == -1)
        return 0;
    const int startDistance = updateLine.a.x - lastLine.a.x;
    const int endDistance = updateLine.b.x - lastLine.b.x;
    const double averageAngle = (lastLine.movingAngle + updateLine.movingAngle) / 2;
    switch(bdNum)
    {
    case 0:
        xShift = (startDistance + endDistance) / 2;
        yShift = (int)(round(fabs(tan(averageAngle) * startDistance)));
        break;
    case 1:
        xShift = (startDistance + endDistance) / 2;
        yShift = (int)((-1) * round(fabs(tan(averageAngle) * startDistance)));
        break;
    case 2:
        xShift = (int)(round(fabs(startDistance / tan(averageAngle))));
        yShift = (startDistance + endDistance) / 2;        
        break;
    case 3:
        xShift = (int)((-1) * (round(fabs(startDistance / tan(averageAngle)))));
        yShift = (startDistance + endDistance) / 2;        
        break;
    }
    
    return 0;
}
    
// 1. when do re-calc the curBox, we tend to get it a little bigger.
// 2. then we use diffOrResult & curMaxChangeSize to limit the expand of the box size.
cv::Rect ContourTrack :: getMaxCrossBoxUsingDiff(const BgResult & bgResult,
                                           const cv::Mat & diffAnd, const cv::Mat & diffOr)
{ 
    // 1. we use this dx dy and diffOr to get the possible maxium box
    int dx = 0, dy = 0;
    curMaxChangeSize(dx, dy);
    int lux = m_lastBox.x - dx;
    int luy = m_lastBox.y - dy;
    int rbx = m_lastBox.x + m_lastBox.width + dx;
    int rby = m_lastBox.y + m_lastBox.height + dy;
    if (lux < 0) lux = 0;
    if (luy < 0) luy = 0;
    if (rbx > m_imgWidth) rbx = m_imgWidth-1;
    if (rby > m_imgHeight) rby = m_imgHeight-1;
    if (rbx <= lux || rby < luy)
    LogD("%d-%d-%d-%d, %d-%d-%d-%d. dx-%d,dy-%d\n",rbx,lux,rby,luy,m_curBox.x,m_curBox.y,
          m_curBox.width, m_curBox.height, dx, dy);
    //assert(rbx > lux && rby > luy);
    cv::Rect maxBox(lux, luy, rbx - lux, rby - luy);
    if (maxBox.width % 2 != 0) maxBox.width--;
    if (maxBox.height % 2 != 0) maxBox.height--;
    // a). shrink the max box using 'diffOr' to get the possible maxium box.
    doShrinkBoxUsingImage(diffOr, maxBox);
    // b). then shrink the max box using new bgResult
    doShrinkBoxUsingImage(bgResult.binaryData, maxBox);    

    return maxBox;
}

// box's width & height must be an even number.
int ContourTrack :: doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box)
{   // using a 2x2 window do scaning the image from the border of the box
    // 1. top
    int k = 0;
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   // find a 2x2 area with all '255' (foreground).
            if (image.at<uchar>(box.x + k, box.y + j)     &
                image.at<uchar>(box.x + k, box.y + j+1)   &
                image.at<uchar>(box.x + k+1, box.y + j)   &
                image.at<uchar>(box.x + k+1, box.y + j+1) )
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    // do update: 
    box.y += k;
    box.height -= k;
    
    // 2. bottom
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + k, box.y + box.height - j)     &
                image.at<uchar>(box.x + k, box.y + box.height - j-1)   &
                image.at<uchar>(box.x + k+1, box.y + box.height - j)   &
                image.at<uchar>(box.x + k+1, box.y + box.height - j -1))
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    // do update: 
    box.height -= k;

    // 3. left
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + k, box.y + j)     &
                image.at<uchar>(box.x + k, box.y + j+1)   &
                image.at<uchar>(box.x + k+1, box.y + j)   &
                image.at<uchar>(box.x + k+1, box.y + j+1) )
                score++; // find the boundary.
        }
        if (score >= 4 || score * 2.0 / box.height > 0.1)
            break;
    }
    box.x += k;
    box.width -= k;

    // 4. right
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.x + box.width - k, box.y + box.height - j)   &
                image.at<uchar>(box.x + box.width - k, box.y + box.height - j-1) &
                image.at<uchar>(box.x + box.width - k-1, box.y + box.height -j)  &
                image.at<uchar>(box.x + box.width - k-1, box.y + box.height -j-1))
                score++; // find the boundary.
        }
        if (score >= 4 || score * 2.0 / box.height > 0.1)
            break;
    }
    box.width -= k;
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones
// shrink or dilate, just estimation, may not be used.
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
double ContourTrack :: calcOverlapRate(const cv::Rect & a, const cv::Rect & b)
{
    assert(a.width == b.width && a.height == b.height);
    if (a.width == 0 || a.height == 0)
        return 0.0;
    cv::Rect overlapBox = calcOverlapArea(a, b);
    return (overlapBox.width * overlapBox.height * 1.0 / (a.width * a.height));
}

cv::Rect ContourTrack :: calcOverlapArea(const cv::Rect & a, const cv::Rect & b)
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
    if (maxBox.y < minBox.y)
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
    // TODO: magic number should be eliminated
    if (rect.x <= m_skipLR + 16)
        directions.push_back(LEFT);
    if (rect.x + rect.width >= m_imgWidth - m_skipLR - 16)
        directions.push_back(RIGHT);
    if (rect.y <= m_skipTB + 16)
        directions.push_back(TOP);
    if (rect.y + rect.height >= m_imgHeight - m_skipTB - 16)
        directions.push_back(RIGHT);

    assert(directions.size() <= 2);
    return directions;
}
    
} // namespace Seg_Three
