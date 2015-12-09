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
                             const cv::Rect & firstBox, 
                             const int firstAppearFrameCount)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_skipTB(skipTB)
    , m_skipLR(skipLR)      
    , m_inputFrames(0)
    , m_firstAppearFrameCount(firstAppearFrameCount)
    , m_bOutputRegion(false) // may not be used
    , m_lastBox(firstBox)
    , m_curBox(firstBox)
    , m_ctTracker(NULL)
    , m_largestWidth(m_curBox.width) // may not be used
    , m_largestHeight(m_curBox.height) // may not be used
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
    // the possible boundary lines that we may dealing with
    vector<MOVING_DIRECTION> directions = checkBoxApproachingBoundary(m_curBox);
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;    
   
    // 1. consume lines interested by tracker(using curBox/lastBoundaryLine)
    vector<CONSUME_LINE_RESULT> boundaryResults;
    for (int bdNum = 0; bdNum < (int)resultLines.size(); bdNum++)
    {
        auto it = std::find(directions.begin(), directions.end(), bdNum);
        if (it != directions.end())
        {   // we need process boundary lines, after process we marked it as used.
            for (int k = 0; k < (int)resultLines[bdNum].size(); k++)
                boundaryResults.push_back(processOneBoundaryLine(bdNum,
                                            resultLines[bdNum][k],bgResult, diffAnd, diffOr));
        }
    }

    // 2. do status changing update
    doStatusChanging(getConsumeResult(boundaryResults));
    if (m_movingStatus == MOVING_FINISH)
        return 1;
        
    // 3. compressive tracking. some objects may never use this (always cross boundaries)
    if (m_movingStatus == MOVING_INSIDE || m_movingStatus == MOVING_STOP) // STOP needed?
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

    // 4. do post-process of boundary line update (after we get new curBox)
    directions.clear();
    directions = checkBoxApproachingBoundary(m_curBox);
    for (int bdNum = 0; bdNum < (int)resultLines.size(); bdNum++)
    {
        auto it = std::find(directions.begin(), directions.end(), bdNum);
        if (it != directions.end())
        {   // we need process boundary lines, after process we marked it as used.
            for (int k = 0; k < (int)resultLines[bdNum].size(); k++)
            {
                if (resultLines[bdNum][k].bValid == true &&
                    resultLines[bdNum][k].bUsed == false) // For bothin CROSS_IN/OUT
                {
                    TDLine boundaryLine = rectToBoundaryLine(bdNum, m_curBox,
                                                             false, m_skipTB, m_skipLR);
                    const int overlapLen = overlapXLenOfTwolines(boundaryLine,
                                                                 resultLines[bdNum][k]);
                    // TODO: magic number here!!!
                    if (boundaryLine.b.x - boundaryLine.a.x > 0 &&
                        overlapLen * 1.0 / (boundaryLine.b.x - boundaryLine.a.x) > 0.6)
                    {
                        resultLines[bdNum][k].bUsed = true;
                    }
                }
            }
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
// the most important one, all complexities are implemented by this function.
// 1. update curBox using boundary lines
// 2. update moving status using boundary lines
// 3. 
// return values: >= 0, process ok
//                 < 0, process error
CONSUME_LINE_RESULT ContourTrack :: processOneBoundaryLine(const int bdNum,
    TDLine & consumeLine, BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // 0. some pre-check.
    if (consumeLine.bValid == false)
        return CONSUME_NOTHING;
    
    // 1. check its previous line, all valid line(output by BoundaryScan) has previous line
    bool bBoundaryConsume = false;
    TDLine lastLine = m_lastBoundaryLines[bdNum]; 
    if (consumeLine.mayPreviousLineStart.x == lastLine.a.x &&
        consumeLine.mayPreviousLineEnd.x == lastLine.b.x &&
        lastLine.a.x != -1 && lastLine.b.x != -1)
        bBoundaryConsume = true;
    else
    {   // no previous line marked, if have overlap, it is mostly movingOut or movingIn with
        // another boundary.
        // just check its line overlap with curBox
        // NOTE: please make sure curBox never have width/height = 0.
        lastLine = rectToBoundaryLine(bdNum, m_lastBox, false, m_skipTB, m_skipLR);
        const int overlapLen = overlapXLenOfTwolines(lastLine, consumeLine);
        // TODO: magic number here!!!
        if (lastLine.b.x - lastLine.a.x > 0 &&
            overlapLen * 1.0 / (lastLine.b.x - lastLine.a.x) > 0.6)
            bBoundaryConsume = true;
    }
    // whether need boundary update
    if (bBoundaryConsume == false)
        return CONSUME_NOTHING;

    // 2. Ok now, let's update curBox with this boundary line.
    // 1) first we get the minimal kernel, then calculate the enlarge & shrink range.
    static const int maxEnlargeDx = 32, maxEnlargeDy = 32;
    static const int maxShrinkDx = 32, maxShrinkDy = 32;
    cv::Rect box = estimateMinBoxByTwoConsecutiveLine(bdNum, lastLine,
                                                      consumeLine, consumeLine.movingStatus);
    // 2) then do enlarge / shrink / boundBox
    // a). get the possible maxium box using 'diffOr', used for boundbox.
    cv::Rect maxBox = box;
    doEnlargeBoxUsingImage(diffOr, maxBox, maxEnlargeDx * 2, maxEnlargeDy * 2);
    // b). normal enlarge using new bgResult
    doEnlargeBoxUsingImage(bgResult.binaryData, box, maxEnlargeDx, maxEnlargeDy);
    // c). normal shrink  using new bgResult
    doShrinkBoxUsingImage(bgResult.binaryData, box, maxShrinkDx, maxShrinkDy);
    // d). bound box
    boundBoxByMaxBox(box, maxBox);

    // 3) finally, we do some internal update
    m_curBox = box;
    m_lastBoundaryLines[bdNum] = consumeLine;
    if (m_largestWidth < m_curBox.width)
        m_largestWidth = m_curBox.width;
    if (m_largestHeight < m_curBox.height)
        m_largestHeight = m_curBox.height;
    
    return consumeLine.movingStatus == MOVING_CROSS_IN ? CONSUME_IN_LINE : CONSUME_OUT_LINE;
}
    
int ContourTrack :: doEnlargeBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                                           const int maxEnlargeDx, const int maxEnlargeDy)
{
    const int minTopY = box.y - maxEnlargeDy < 0 ? 0 : box.y - maxEnlargeDy;
    const int maxTopY = box.y;
    const int minBottomY = box.y + box.height;
    const int maxBottomY = box.y + box.height + maxEnlargeDy > m_imgHeight ?
                           m_imgHeight : box.y + box.height + maxEnlargeDy;
    const int minLeftX = box.x - maxEnlargeDx < 0 ? 0 : box.x - maxEnlargeDx;
    const int maxLeftX = box.x;
    const int minRightX = box.x + box.width;
    const int maxRightX = box.x + box.width + maxEnlargeDx > m_imgWidth ?
                          m_imgWidth : box.x + box.width + maxEnlargeDx;
    // TODO: magic number here
    const static int enlargeLossThreshold = 3;
    const static double enlargeScorePercentThreshold = 0.1;    
        
    cv::Rect newBox = box;
    int k = 0;
    // 1. enlarge top line
    int loss = 0;
    for (k = maxTopY; k > minTopY; k--) 
    {   
        int j = 0, score = 0;
        for (j = box.x; j < box.x + box.width; j++) 
            if (image.at<uchar>(k, j))
                score++;
        if (score * 1.0 / box.width < enlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= enlargeLossThreshold)
                break;
        }
    }
    newBox.y = k;
    // 2. enlarge bottom line
    loss = 0;
    for (k = minBottomY; k < maxBottomY; k++)
    {   
        int j = 0, score = 0;
        for (j = box.x; j < box.x + box.width; j++)
            if (image.at<uchar>(k, j))
                score++;
        if (score * 1.0 / box.width < enlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= enlargeLossThreshold)
                break;
        }
    }
    newBox.height = k - newBox.y;

    // 3. enlarge left line
    loss = 0;    
    for (k = maxLeftX; k > minLeftX; k--) 
    {
        int j = 0, score = 0;
        // NOTE: here we use new box's y & height
        for (j = newBox.y; j <= newBox.y + newBox.height; j++) // note, j+2 here
            if (image.at<uchar>(j, k))
                score++;
        if (score * 1.0 / newBox.height < enlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= enlargeLossThreshold)
                break;
        }        
    }
    newBox.x = k;
    // 4. enlarge right line
    loss = 0;    
    for (k = minRightX; k < maxRightX; k++) 
    {
        int j = 0, score = 0;        
        for (j = newBox.y; j <= newBox.y + newBox.height; j++)
            if (image.at<uchar>(j, k))
                score++;
        if (score * 1.0 / newBox.height < enlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= enlargeLossThreshold)
                break;
        }        
    }
    newBox.width = k - newBox.x;
    
    LogD("Befor Enlarge: \n");
    dumpRect(box);
    LogD("After Enlarge: \n");
    dumpRect(newBox);
    box = newBox;
    return 0;
}

// the counterpart of the doEnlarge
int ContourTrack :: doShrinkBoxUsingImage(const cv::Mat & image, cv::Rect & box,
                                          const int maxShrinkDx, const int maxShrinkDy)
{   
    const int minTopY = box.y;
    const int maxTopY = maxShrinkDy > box.height/2 ? box.y + box.height/2 : box.y + maxShrinkDy;
    const int maxBottomY = box.y + box.height; 
    const int minBottomY = maxShrinkDy > box.height/2 ?
                                       box.y + box.height/2 : box.y + box.height - maxShrinkDy;
    const int minLeftX = box.x;
    const int maxLeftX = maxShrinkDx > box.width/2 ? box.x + box.width/2 : box.x + maxShrinkDx;
    const int minRightX = maxShrinkDx > box.width/2 ?
                                      box.x + box.width/2 : box.x + box.width - maxShrinkDx;
    const int maxRightX = box.x + box.width;
    
    // using a 2x2 window do scaning the image from the border of the box
    cv::Rect newBox = box;
    int k = 0;
    // 1. top shrink
    for (k = minTopY; k < maxTopY; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = box.x; j < box.x + box.width; j+=2) // note, j+2 here
        {   // find a 2x2 area with all '255' (foreground).
            if (image.at<uchar>(k, j)   & image.at<uchar>(k, j+1) &
                image.at<uchar>(k+1, j) & image.at<uchar>(k+1, j+1))
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    newBox.y = k;
    // 2. bottom shrink
    for (k = maxBottomY; k > minBottomY; k-=2)
    {
        int j = 0;
        int score = 0;
        for (j = box.x; j < box.x + box.width; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(k, j)   & image.at<uchar>(k, j+1) &
                image.at<uchar>(k+1, j) & image.at<uchar>(k+1, j+1))
                score++;
        }
        if (score >= 4 || score * 2.0 / box.width > 0.1)
            break;
    }
    newBox.height = k - newBox.y;
    // 3. left shrink
    for (k = minLeftX; k < maxLeftX; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = box.y; j < box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(j, k)   & image.at<uchar>(j, k+1) &
                image.at<uchar>(j+1, k) & image.at<uchar>(j+1, k+1))
                score++;
        }
        if (score >= 4 || score * 2.0 / box.height > 0.1)
            break;
    }
    newBox.x = k;
    // 4. right
    for (k = maxRightX; k > minRightX; k-=2)
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
    newBox.width = k - newBox.x;
    LogD("Before Shrink: \n");
    dumpRect(box);
    LogD("After Shrink: \n");
    dumpRect(newBox);
    box = newBox;
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones    
// check box close to which boundary (should take skipTB,LR into account)    
vector<MOVING_DIRECTION> ContourTrack :: checkBoxApproachingBoundary(const cv::Rect & rect)
{
    vector<MOVING_DIRECTION> directions;
    static const int APPROCHING_DISTANCE = 4;
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

// Used by: updateCrossInBox, keep in mind there are boundary lines there.
// use two consecutive boundary lines to estimate posssible changing of the box,
// used when cross in. the result will be used in enlarge/shrink operaion as the
// minimal box.
cv::Rect ContourTrack :: estimateMinBoxByTwoConsecutiveLine (const int bdNum,
    const TDLine & lastLine, const TDLine & updateLine, const bool bCrossIn = true)
{
    cv::Rect minBox;
    //const double averageAngle = (lastLine.movingAngle + updateLine.movingAngle) / 2;
    const int dv = bCrossIn ? 4 : -4; // TODO: should using other info to infer this value.
    const int lineLen = std::max(lastLine.b.x - lastLine.a.x, updateLine.b.x - updateLine.a.x); 
    switch(bdNum)
    {
    case 0: // Top
        minBox.x = std::min(lastLine.a.x, updateLine.a.x) + m_skipLR;
        minBox.y = m_skipTB;
        minBox.width = lineLen;
        minBox.height = m_curBox.height + dv;
        break;
    case 1: // Bottom
        minBox.x = std::min(lastLine.a.x, updateLine.a.x) + m_skipLR;
        minBox.y = m_imgHeight - m_curBox.height - m_skipTB - dv;
        minBox.width = lineLen;
        minBox.height = m_curBox.height + dv;
        break;
    case 2: // Left
        minBox.x = m_skipLR;
        minBox.y = std::min(lastLine.a.x, updateLine.a.x) + m_skipTB;
        minBox.width = m_curBox.width + dv;
        minBox.height = lineLen;
        break;
    case 3: // Right
        minBox.x = m_imgWidth - m_curBox.width - m_skipLR - dv;
        minBox.y = std::min(lastLine.a.x, updateLine.a.x) + m_skipTB;
        minBox.width = m_curBox.width + dv;
        minBox.height = lineLen;
        break;
    }
    
    return minBox;
}

int ContourTrack :: getConsumeResult(const vector<CONSUME_LINE_RESULT> & results)
{
    int result = (int)CONSUME_NOTHING;
    for (int k=0; k < (int)results.size(); k++)
        result |= (int)results[k];
    return result;
}

int ContourTrack :: doStatusChanging(const int statusResult)
{
    switch(statusResult)
    {
    case CONSUME_NOTHING:
        if (m_movingStatus == MOVING_CROSS_IN)
            m_allInCount++;
        else if (m_movingStatus == MOVING_CROSS_OUT)
            m_allOutCount++;
        else // MOVING_INSIDE/STOP: this is normal, moving inside should with no boundary lines.
            m_crossOutCount = 0;
        break;
    case CONSUME_IN_LINE:
        m_allInCount = 0;
        break;
    case CONSUME_OUT_LINE:
        m_allOutCount = 0;
        if (m_movingStatus == MOVING_INSIDE)
            m_crossOutCount++;
        break;
    case (CONSUME_IN_LINE | CONSUME_OUT_LINE):
        if (m_movingStatus == MOVING_CROSS_IN)
            LogW("Object cross In & Out simultaneously.\n");
        //cross IN/OUT simultaneously
        break;
    }

    if (m_allInCount >= M_MOVING_STATUS_CHANGING_THRESHOLD &&
        m_movingStatus == MOVING_CROSS_IN)
        m_movingStatus = MOVING_INSIDE;
    else if (m_crossOutCount >= M_MOVING_STATUS_CHANGING_THRESHOLD &&
             m_movingStatus == MOVING_INSIDE)
        m_movingStatus = MOVING_CROSS_OUT;
    else if (m_allOutCount >= M_MOVING_STATUS_CHANGING_THRESHOLD &&
             m_movingStatus == MOVING_CROSS_OUT)
        m_movingStatus = MOVING_FINISH;
    
    return 0;
}


} // namespace Seg_Three

/////////////////////////// End of The File //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////    

/*
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
        // line's movingDirection should be MOVING_CROSS_OUT
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
*/
