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
    , m_allOutCount(0)
    , m_crossOutCount(0)  
{
    // line's direction is not quit the same as the tracker's moving direction
    assert(theLine.movingDirection < 4);
    assert(width > 0 && height > 0);
    m_lastBoundaryLines[(int)theLine.movingDirection] = theLine;   
    // compressive tracker part.
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
    bool bCTTracking = false; // use Compressive Tracking or BoundaryInfo tracking.
    switch(m_movingStatus)
    {
    case MOVING_CROSS_IN:
        markAcrossIn(directions, bgResult, diffAnd, diffOr);
        if (m_allInCount >= M_MOVING_STATUS_CHANGING_THRESHOLD) 
        {   
            for (int k = 0; k < BORDER_NUM; k++) // reset lastBoundaryLines
                m_lastBoundaryLines[k] = TDLine();
            m_movingStatus = MOVING_INSIDE; // mark we are inside!
            m_allInCount = 0;
            bCTTracking = true; // if all in, use normal tracking.
        }
        break;
    // MOVING_INSIDE trackers just do untraced & MOVING_CROSS_OUT checking
    // BE AWARE: 1) tracker has moving status(cross_in/cross_out/inside).
    //           2) each boundary line also has moving status(cross_in/cross_out).            
    case MOVING_INSIDE:
        // TODO: magic number subsitution needed. 10frames * 10pixel = 100 pixles
        if (m_inputFrames >= 5 && directions.size() > 0)
        {   // line's movingDirection should be MOVING_CROSS_OUT
            // update lastBoundaryLines inside this call if needed
            markAcrossOut(directions, bgResult, diffAnd, diffOr);
            if (m_crossOutCount >= M_MOVING_STATUS_CHANGING_THRESHOLD)
            {
                m_crossOutCount = 0;
                bCTTracking = false;
                m_movingStatus = MOVING_CROSS_OUT;
            }
            else
                bCTTracking = true;
        }
        else // do normal tracking.
            bCTTracking = true;
        break;
    case MOVING_CROSS_OUT:
        //  use boundary info to track, won't use compressive tracker
        markAcrossOut(directions, bgResult, diffAnd, diffOr);
        if (m_allOutCount >= M_MOVING_STATUS_CHANGING_THRESHOLD)
        {   // !! // 1 means terminal the tracking.
            // NOTE: if all out, we tell the caller to terminal the whole tracking.
            return 1; 
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
int ContourTrack ::  markAcrossIn(const vector<MOVING_DIRECTION> & directions,
                                  BgResult & bgResult,
                                  const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    LogI("Across In frame: %d, directions size: %d.\n", m_inputFrames, (int)directions.size());
    dumpVectorInt(directions);
    // 
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
                            LogD("???? Get Previous Line & updateCrossInBox. "
                                 "Last %d-%d-%d-%d,  %d-%d-%d-%d.\n",
                                 m_lastBox.x, m_lastBox.y, m_lastBox.width, m_lastBox.height,
                                 m_curBox.x, m_curBox.y, m_curBox.width, m_curBox.height);
                        }
                        else // for the 
                        {
                            LogD("????????? Realy here.\n");
                        }
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
                        else
                            LogD("????????? How to Deal ??.\n");
                    }
                }
            }
        }
    }
    // all in
    if (bStillCrossing == false)
        m_allInCount++;

    return 0;
}

// if cannot find the moving out lines, we return < 0 to indicate it is still moving inside
// or the whole object is moving out.
// Two consecutive frames are used to do the measure.
int ContourTrack :: markAcrossOut(const vector<MOVING_DIRECTION> & directions,
                        BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // for across out, it is relatively simple then in,
    // for we don't need accurately change our box any more.
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;
    bool bCrossing = false;
    for (int bdNum = 0; bdNum < BORDER_NUM; bdNum++)
    {
        auto it = std::find(directions.begin(), directions.end(), bdNum);
        if (it != directions.end())
        {
            for (int k = 0; k < (int)resultLines[bdNum].size(); k++)
            {
                if (resultLines[bdNum][k].movingStatus == MOVING_CROSS_OUT)
                {   
                    if (resultLines[bdNum][k].mayPreviousLineStart.x != -1 &&
                        resultLines[bdNum][k].mayPreviousLineEnd.x != -1)
                    {   // have previous line
                        if (resultLines[bdNum][k].mayPreviousLineStart.x ==
                                m_lastBoundaryLines[bdNum].a.x &&
                            resultLines[bdNum][k].mayPreviousLineEnd.x ==
                                m_lastBoundaryLines[bdNum].b.x)
                        {   // the predecessor is found, so update curBox using the line
                            bCrossing = true;
                            updateCrossOutBox(bdNum, resultLines[bdNum][k],
                                              bgResult, diffAnd, diffOr);
                            m_lastBoundaryLines[bdNum] = resultLines[bdNum][k];
                        }
                        // TODO: do we need a else here ??????
                        // if BoundaryScan do a great job, we don't have to have a 'else'.
                    }
                    else 
                    {   // this untraced line may part of our object, let's check it.
                        if (updateUntracedIfNeeded(bdNum, resultLines[bdNum][k]) > 0)
                        {
                            bCrossing = true;
                            updateCrossOutBox(bdNum, resultLines[bdNum][k],
                                              bgResult, diffAnd, diffOr);
                            m_lastBoundaryLines[bdNum] = resultLines[bdNum][k];
                        }
                    }
                }
            }
        }        
    }

    // change status.
    if (bCrossing == true && m_movingStatus == MOVING_INSIDE)
        m_crossOutCount++;
    else
        m_crossOutCount = 0;
    
    if (bCrossing == false && m_movingStatus == MOVING_CROSS_OUT)
        m_allOutCount++;
    else 
        m_allOutCount = 0;
    return 0;
}
    
// 1. when do re-calc the curBox, we tend to get it a little bigger.
// 2. then we use diffOrResult & curMaxChangeSize to limit the expand of the box size.
cv::Rect ContourTrack :: getMaxCrossBoxUsingDiff(const BgResult & bgResult,
                                           const cv::Mat & diffAnd, const cv::Mat & diffOr)
{ 
    // 1. we use this dx dy and diffOr to get the possible maxium box
    // restrictions: each time we at most enlarge 32 pixels, shrink 16pixels
    // so enlarge/shrink gradually.
    static const int maxEnlargeDx = 32, maxEnlargeDy = 32;
    static const int maxShrinkDx = 16, maxShrinkDy = 16;    
    // now we do enlarge & shrink.
    cv::Rect boxThisRound = m_lastBox;
    // a). enlarge the max box using 'diffOr' to get the possible maxium box.
    doEnlargeBoxUsingImage(diffOr, boxThisRound, maxEnlargeDx, maxEnlargeDy);
    //// b). shrink the max box using new bgResult
    doShrinkBoxUsingImage(bgResult.binaryData, box, maxShrinkDx, maxShrinkDy);    

    boundBoxByMaxBox(box, theMaxBox);
    return box;
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
    // 1. first we use lastBox & diffResult to get the max box
    cv::Rect maxBox = getMaxCrossBoxUsingDiff(bgResult, diffAnd, diffOr);
    LogD("111111:\n");
    dumpRect(maxBox);
    // 2. calculate the minimal area that needed(get min box)
    cv::Rect minBox = calcOverlapArea(m_lastBox, maxBox);
    LogD("2222:\n");
    dumpRect(minBox);
    
    // 3. then use lastBoundaryLine[bdNum] (must have, for we are in updateTracker)
    //    angle to get the possible x, y shift
    TDLine & lastLine = m_lastBoundaryLines[bdNum];
    int xShift = 0, widthShift = 0, yShift = 0, heightShift = 0;
    estimateShiftByTwoConsecutiveLine(xShift, yShift,
                                      widthShift, heightShift, bdNum, lastLine, updateLine);
    maxBox.x = (maxBox.x + m_lastBox.x + xShift) / 2;
    maxBox.y = (maxBox.y + m_lastBox.y + yShift) / 2;
    maxBox.width = (maxBox.width + m_lastBox.width) / 2; // take it as no width changing
    maxBox.height = (maxBox.height + m_lastBox.height) / 2; // take it as no height changing
    LogD("3333:\n");
    dumpRect(maxBox);
    
    // 4. make the max box at least contain the min box.
    enlargeBoxByMinBox(maxBox, minBox);
    enlargeBoxByMinBox(maxBox, cv::Rect(maxBox.x, maxBox.y, 16, 16));    
    LogD("4444:\n");
    dumpRect(maxBox);
    
    /* 5. Until Now, we get the next box */
    m_curBox = maxBox;
    if (m_largestWidth < m_curBox.width)
        m_largestWidth = m_curBox.width;
    if (m_largestHeight < m_curBox.height)
        m_largestHeight = m_curBox.height;
    
    return 0;
}    

int ContourTrack :: updateCrossOutBox(const int bdNum, TDLine & updateLine,
                                      BgResult & bgResult,
                                      const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    TDLine & lastLine = m_lastBoundaryLines[bdNum];
    int xShift = 0, yShift = 0;
    //getShiftByTwoConsecutiveLine(xShift, yShift, bdNum, lastLine, updateLine);
    // cross_out shift is the opposite of cross in shift
    m_curBox.x = m_lastBox.x - xShift;
    m_curBox.y = m_lastBox.y - yShift;
    m_curBox.width = m_lastBox.width - abs(xShift);
    m_curBox.height = m_lastBox.height - abs(yShift);
    return 0;
}
        
// return > 0: need update this line & mark this line's previousLine
int ContourTrack :: updateUntracedIfNeeded(const int bdNum, TDLine & updateLine)
{
    TDLine boundaryLine = m_lastBoundaryLines[bdNum];
    if (boundaryLine.a.x == -1 && boundaryLine.b.x == -1)
        boundaryLine = rectToBoundaryLine(bdNum, m_lastBox);
    const double consecutivityScore = consecutivityOfTwoLines(updateLine, boundaryLine, 50);
    // TODO: magic number 75.0 here.
    if (consecutivityScore > 75.0)
    {
        updateLine.mayPreviousLineStart = boundaryLine.a;
        updateLine.mayPreviousLineEnd = boundaryLine.b;
        m_lastBoundaryLines[bdNum] = boundaryLine;
        return 1;
    }
    return 0;
}

int ContourTrack :: doEnlargeBoxUsingImage(const cv::Mat & diffOr, cv::Rect & box,
                                           const maxEnlargeDx, const maxEnlargeDy)
{
    const int minX = m_lastBox.x - maxEnlargeDx < 0 ? 0 : m_lastBox.x - maxEnlargeDx;
    const int minY = m_lastBox.y - maxEnlargeDy < 0 ? 0 : lastBox.y - maxEnlargeDy;
    const int maxX = m_lastBox.x + m_lastBox.width + maxEnlargeDx > m_imgWidth ?
                                   m_imgWidth : m_lastBox.x + m_lastBox.width + maxEnlargeDx;
    const int maxY = m_lastBox.y + m_lastBox.height + maxEnlargeDy > m_imgHeight ?
                                   m_imgHeight : m_lastBox.y + m_lastBox.height + maxEnlargeDy;
    if (maxX <= minX || maxY <= minY)
    {
        LogW("Something Wrong: %d-%d-%d-%d, %d-%d-%d-%d. Won't enlarge box.\n",
             minX, minY, maxX, maxY,
             m_lastBox.x, m_lastBox.y, m_lastBox.width, m_lastBox.height);
        return -1;
    }
    
    cv:Rect newBox = boxThisRound;
    int k = 0;
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   // find a 2x2 area with all '255' (foreground).
            if (image.at<uchar>(box.y + k, box.x + j)     &
                image.at<uchar>(box.y + k, box.x + j+1)   &
                image.at<uchar>(box.y + k+1, box.x + j)   &
                image.at<uchar>(box.y + k+1, box.x + j+1) )
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    // do update:
    newBox.y += k;
    newBox.height -= k;
    
    return 0;
}

// box's width & height must be an even number.
int ContourTrack :: doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box)
{   // using a 2x2 window do scaning the image from the border of the box
    // 1. top
    cv::Rect newBox = box;
    int k = 0;
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   // find a 2x2 area with all '255' (foreground).
            if (image.at<uchar>(box.y + k, box.x + j)     &
                image.at<uchar>(box.y + k, box.x + j+1)   &
                image.at<uchar>(box.y + k+1, box.x + j)   &
                image.at<uchar>(box.y + k+1, box.x + j+1) )
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    // do update:
    newBox.y += k;
    newBox.height -= k;
    
    // 2. bottom
    for (k = 0; k < box.height; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.width; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.y + box.height - k, box.x + j)     &
                image.at<uchar>(box.y + box.height - k, box.x + j+1)   &
                image.at<uchar>(box.y + box.height - k+1, box.x + j)   &
                image.at<uchar>(box.y + box.height - k+1, box.x + j+1))
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    // do update: 
    newBox.height -= k;

    // 3. left
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.y + j, box.x + k)     &
                image.at<uchar>(box.y + j, box.x + k+1)   &
                image.at<uchar>(box.y + j+1, box.x + k)   &
                image.at<uchar>(box.y + j+1, box.x + k+1) )
                score++; // find the boundary.
        }
        if (score >= 4 || score * 2.0 / box.height > 0.1)
            break;
    }
    newBox.x += k;
    newBox.width -= k;

    // 4. right
    for (k = 0; k < box.width; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = 0; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(box.y + j, box.x + box.width - k)   &
                image.at<uchar>(box.y + j, box.x + box.width - k-1) &
                image.at<uchar>(box.y + j+1, box.x + box.width -k)  &
                image.at<uchar>(box.y + j+1, box.x + box.width -k-1))
                score++; // find the boundary.
        }
        if (score >= 4 || score * 2.0 / box.height > 0.1)
            break;
    }
    newBox.width -= k;
    box = newBox;
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones
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

void ContourTrack :: enlargeBoxByMinBox(cv::Rect & box, const cv::Rect & minBox)
{
    if (box.x > minBox.x)
        box.x = minBox.x;
    if (box.y < minBox.y)
        box.y = minBox.y;
    if (box.x + box.width < minBox.x + minBox.width)
        box.width = minBox.x + minBox.width - box.x;
    if (box.y + box.height < minBox.y + minBox.height)
        box.height = minBox.y + minBox.height - box.y;    
    return;
}

void ContourTrack :: boundBoxByMaxBox(cv::Rect & box, const cv::Rect & maxBox)
{
    if (box.x < maxBox.x)
        box.x = maxBox.x;
    if (box.y > maxBox.y)
        box.y = maxBox.y;
    if (box.x + box.width > maxBox.x + maxBox.width)
        box.width = maxBox.x + maxBox.width - box.x;
    if (box.y + box.height > maxBox.y + maxBox.height)
        box.height = maxBox.y + maxBox.height - box.y;    
    return;
}

    
// check box close to which boundary (should take skipTB,LR into account)    
vector<MOVING_DIRECTION> ContourTrack :: checkBoxApproachingBoundary(const cv::Rect & rect)
{
    vector<MOVING_DIRECTION> directions;
    static const int APPROCHING_DISTANCE = 8;
    // TODO: magic number 8 should be eliminated
    if (rect.x <= m_skipLR + APPROCHING_DISTANCE)
        directions.push_back(LEFT);
    if (rect.x + rect.width >= m_imgWidth - m_skipLR - APPROCHING_DISTANCE)
        directions.push_back(RIGHT);
    if (rect.y <= m_skipTB + APPROCHING_DISTANCE)
        directions.push_back(TOP);
    if (rect.y + rect.height >= m_imgHeight - m_skipTB - APPROCHING_DISTANCE)
        directions.push_back(RIGHT);

    // normally, the size of directions is 1 or 2, very rare it is 3 or 4.
    return directions;
}

// use two possible consecutive lines to estimate x,y direction moving.
int ContourTrack :: estimateShiftByTwoConsecutiveLine (int & xShift, int & yShift,
                    int & widthShift, int & heightShift, const int bdNum,
                    const TDLine & lastLine, const TDLine & updateLine)
{

    // debugging
    xShift = yShift = widthShift = heightShift = 16;
    return 0;
    /////
    if (lastLine.a.x == -1 && lastLine.b.x == -1)
    {
        LogE("Invalid LastLine, then we cnannot calculate consecutivity of two lines.\n");
        return -1; // should not be the case
    }
    const int startDistance = updateLine.a.x - lastLine.a.x;
    const int endDistance = updateLine.b.x - lastLine.b.x;
    const double averageAngle = (lastLine.movingAngle + updateLine.movingAngle) / 2;
    switch(bdNum)
    {
    case 0:
        xShift = 00; // TODO: 
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
    
} // namespace Seg_Three
