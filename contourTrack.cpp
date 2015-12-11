#include <algorithm>
#include <math.h>
#include "contourTrack.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
ContourTrack :: ContourTrack(const int idx, const int width, const int height,
                             const int skipTB, const int skipLR, const int takeFrameInterval,
                             const int directionIn, const TDLine & theLine,
                             const cv::Rect & firstBox, const int firstAppearFrameCount,
                             const cv::Mat & in, BgResult & bgResult)
    : m_idx(idx)
    , m_imgWidth(width)
    , m_imgHeight(height)
    , m_skipTB(skipTB)
    , m_skipLR(skipLR)
    , m_takeFrameInterval(takeFrameInterval)      
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
    
    m_movingInStatusChangingThreshold = 2; //takeFrameInterval > 1 ? 3 : 2;
    m_movingOutStatusChangingThreshold = 2; //takeFrameInterval > 1 ? 1 : 2;    
    m_maxEnlargeDx = 48 * m_takeFrameInterval; 
    m_maxEnlargeDy = 48 * m_takeFrameInterval; 
    m_maxShrinkDx = 48 * m_takeFrameInterval; 
    m_maxShrinkDy = 48 * m_takeFrameInterval; 

    // make a better initial state.
    adjustBoxByBgResult(bgResult, m_curBox);
    // compressive tracker part, create it & re-init when needed(no need delete it).    
    m_ctTracker = new CompressiveTracker();
    assert(m_ctTracker);
    m_ctTracker->init(in, m_curBox);       
    LogD("Create New ContourTrack %d: InDirection: %d, lux:%d, luy:%d, initWidth:%d, "
         "initHeight:%d. BeforeAdjust: %d-%d-%d-%d.\n", m_idx,
         directionIn, m_curBox.x, m_curBox.y, m_curBox.width, m_curBox.height,
         m_lastBox.x, m_lastBox.y, m_lastBox.width, m_lastBox.height);
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
    dumpRect(m_curBox);
    vector<MOVING_DIRECTION> directions = checkBoxApproachingBoundary(m_curBox);
    vector<vector<TDLine> > & resultLines = bgResult.resultLines;    
    // 1. consume lines interested by tracker(using curBox/lastBoundaryLine)
    vector<int> boundaryResults;
    for (int bdNum = 0; bdNum < (int)resultLines.size(); bdNum++)
    {
        auto it = std::find(directions.begin(), directions.end(), bdNum);
        if (it != directions.end())
        {   // we need process boundary lines, after process we marked it as used.
            for (int k = 0; k < (int)resultLines[bdNum].size(); k++)
                boundaryResults.push_back(processOneBoundaryLine(bdNum, resultLines[bdNum][k],
                                                                 bgResult, diffAnd, diffOr));
        }
    }

    // 2. do status changing update
    const int resultStatus = getConsumeResult(boundaryResults);
    doStatusChanging(resultStatus);
    if (m_movingStatus == MOVING_FINISH)
        return 1;
    
    // 3. compressive tracking. some objects may never use this (always cross boundaries)
    //    1) if consume nothing, we use CT
    //    2) update CT's tracking effective area when it is less than 70%.
    if ((m_movingStatus == MOVING_INSIDE || m_movingStatus == MOVING_STOP) ||// STOP needed?
        resultStatus == (int)CONSUME_NOTHING)
    {
        bool bDoTrack = true;
        bool bNeedReset = false;
        // 1) update the tracking effective area.
        const double lastEffectiveness = getEffectivenessOfBox(bgResult.binaryData, m_curBox);
        if (lastEffectiveness < 0.3)
        {   // very bad, we should not do any update(may be covered by other objects)
            // should avoid this to happen.
            bDoTrack = false;
        }
        else if (lastEffectiveness < 0.7)
        {   // we need adjust current area
            cv::Rect box = m_curBox;
            adjustBoxByBgResult(bgResult, box); // adjust little by little
            // not a better result, we use current
            if (getEffectivenessOfBox(bgResult.binaryData, box) > lastEffectiveness)
            {
                m_curBox = box;
                bNeedReset = true;
            }
        } // else: good situation, we use it curBox(won't update) do tracking.

        if (bDoTrack == true)
        {   // across boundary but consume nothing, we need CTTracking (reset first)
            if (m_movingStatus != MOVING_INSIDE && m_movingStatus != MOVING_STOP)
                bNeedReset = true;
            if (bNeedReset == true) // re-init
                m_ctTracker->init(in, m_curBox);
            // because of CTTracker's bug.
            doBoxProtectionCalibrate(m_curBox);
            
            if (m_ctTracker->processFrame(in, m_curBox) < 0)
            {
                LogW("Compressive Tracker do warning a failing track.\n.");
                // TODO: how to do update ? just terminate the tracking right now.
                return 1;
            }
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
                    if (isXContainedBy(resultLines[bdNum][k], boundaryLine))
                        resultLines[bdNum][k].bUsed = true;
                    else
                    {
                        const int overlapLen = overlapXLenOfTwolines(boundaryLine,
                                                                     resultLines[bdNum][k]);
                        // TODO? XT: 0.5 magic number here!
                        // use takeFrameInterval make it less magic.
                        if (boundaryLine.b.x - boundaryLine.a.x > 0 &&
                            m_takeFrameInterval * overlapLen * 1.0 /
                                  (boundaryLine.b.x - boundaryLine.a.x) > 0.5)
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
int ContourTrack :: processOneBoundaryLine(const int bdNum, TDLine & consumeLine, 
    BgResult & bgResult, const cv::Mat & diffAnd, const cv::Mat & diffOr)
{
    // 0. some pre-check.
    if (consumeLine.bValid == false)
        return (int)CONSUME_NOTHING;
    
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
        lastLine = rectToBoundaryLine(bdNum, m_curBox, false, m_skipTB, m_skipLR);
        //LogD("Consume Line: %s %d-%d %s, lastLine, %d-%d.\n",
        //     getMovingDirectionStr((MOVING_DIRECTION)bdNum),
        //     consumeLine.a.x, consumeLine.b.x, 
        //     getMovingStatusStr(consumeLine.movingStatus),
        //     lastLine.a.x, lastLine.b.x);

        if (isXContainedBy(consumeLine, lastLine) == true)
        {
            bBoundaryConsume = true;
            //consumeLine.bUsed = true;
            //return consumeLine.movingStatus == MOVING_CROSS_IN ?
            //    (int)CONSUME_IN_LINE : (int)CONSUME_OUT_LINE;
            LogD("Fully Contained!\n");
        }
        else
        {   
            const int overlapLen = overlapXLenOfTwolines(lastLine, consumeLine);
            // TODO: magic number here!!! use takeFrameInterval make it less magic
            LogD("Overlap Len %d.\n", overlapLen);
            if (lastLine.b.x - lastLine.a.x > 0 &&
                m_takeFrameInterval * overlapLen * 1.0 / (lastLine.b.x - lastLine.a.x) > 0.1)
                bBoundaryConsume = true;
        }
    }
    
    // whether need boundary update
    if (bBoundaryConsume == false)
        return (int)CONSUME_NOTHING;

    // 2. Ok now, let's update curBox with this boundary line.    
    // 1) TODO: magic number, use takeFrameInterval make it less magic.
    //    first we get the minimal kernel, then calculate the enlarge & shrink range.
    cv::Rect box = estimateMinBoxByTwoConsecutiveLine(bdNum, lastLine, consumeLine,
                                          consumeLine.movingStatus == MOVING_CROSS_IN);
    LogD("%d after estimate: \n", m_idx);    
    // 2) then do enlarge / shrink / boundBox    
    // a). get the possible maxium box using 'diffOr', used for boundbox.
    cv::Rect maxBox = box;
    doEnlargeBoxUsingImage(diffOr, maxBox, m_maxEnlargeDx, m_maxEnlargeDy);
    // b). normal enlarge using new bgResult
    dumpRect(box);
    doEnlargeBoxUsingImage(bgResult.binaryData, box, m_maxEnlargeDx, m_maxEnlargeDy);
    LogD("%d after enlarge: \n", m_idx);
    dumpRect(box);
    // c). normal shrink  using new bgResult
    doShrinkBoxUsingImage(bgResult.binaryData, box, m_maxShrinkDx, m_maxShrinkDy);
    LogD("%d after shrink: \n", m_idx);
    dumpRect(box);
    // d). bound box
    boundBoxByMaxBox(box, maxBox);

    // e). add protection of the newly got box.
    doBoxProtectionCalibrate(box);
    
    // 3 finally, we do some internal update
    m_curBox = box;
    m_lastBoundaryLines[bdNum] = consumeLine;
    if (m_largestWidth < m_curBox.width)
        m_largestWidth = m_curBox.width;
    if (m_largestHeight < m_curBox.height)
        m_largestHeight = m_curBox.height;

    consumeLine.bUsed = true;
    LogD("%d Consume One Line: %s %d-%d %s.\n",
         m_idx, getMovingDirectionStr((MOVING_DIRECTION)bdNum),
         consumeLine.a.x, consumeLine.b.x, getMovingStatusStr(consumeLine.movingStatus));
    return consumeLine.movingStatus == MOVING_CROSS_IN ?
        (int)CONSUME_IN_LINE : (int)CONSUME_OUT_LINE;
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
    const static int EnlargeLossThreshold = 3;
    const static double EnlargeScorePercentThreshold = 0.1;    
        
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
        if (score * 1.0 * m_takeFrameInterval / box.width < EnlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= EnlargeLossThreshold)
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
        if (score * 1.0 * m_takeFrameInterval / box.width < EnlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= EnlargeLossThreshold)
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
        if (score * 1.0  * m_takeFrameInterval / newBox.height < EnlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= EnlargeLossThreshold)
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
        if (score * 1.0  * m_takeFrameInterval / newBox.height < EnlargeScorePercentThreshold)
        {
            loss++;
            if (loss >= EnlargeLossThreshold)
                break;
        }        
    }
    newBox.width = k - newBox.x;
    // final update    
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
        if (score * 2.0 * m_takeFrameInterval / box.width > 0.1)
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
        if (score * 2.0 * m_takeFrameInterval / box.width > 0.1)
            break;
    }
    newBox.height = k - newBox.y;
    // 3. left shrink
    for (k = minLeftX; k < maxLeftX; k+=2)
    {
        int j = 0;
        int score = 0;
        for (j = newBox.y; j < newBox.y + box.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(j, k)   & image.at<uchar>(j, k+1) &
                image.at<uchar>(j+1, k) & image.at<uchar>(j+1, k+1))
                score++;
        }
        if (score * 2.0 * m_takeFrameInterval / box.height > 0.1)
            break;
    }
    newBox.x = k;
    // 4. right
    for (k = maxRightX; k > minRightX; k-=2)
    {
        int j = 0;
        int score = 0;
        for (j = newBox.y; j < newBox.y + newBox.height; j+=2) // note, j+2 here
        {   
            if (image.at<uchar>(j, k)   &
                image.at<uchar>(j, k+1) &
                image.at<uchar>(j+1, k)  &
                image.at<uchar>(j+1, k+1))
                score++; // find the boundary.
        }
        if (score * 2.0 * m_takeFrameInterval / box.height > 0.1)
            break;
    }
    newBox.width = k - newBox.x;
    // update
    box = newBox;
    return 0;
}

int ContourTrack :: doShrinkBoxUsingImage2(const cv::Mat & image, cv::Rect & box,
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
    for (k = minTopY; k < maxTopY; k++)
    {
        int j = 0;
        int score = 0;
        for (j = box.x; j < box.x + box.width; j++)
            if (image.at<uchar>(k, j))// & image.at<uchar>(k, j+1))
                score++;
        if (score * 1.0 * m_takeFrameInterval / box.width > 0.1)
            break;
    }
    newBox.y = k;
    // 2. bottom shrink
    for (k = maxBottomY; k > minBottomY; k--)
    {
        int j = 0;
        int score = 0;
        for (j = box.x; j < box.x + box.width; j++)
            if (image.at<uchar>(k, j)) // & image.at<uchar>(k, j+1))
                score++;        
        if (score * 1.0 * m_takeFrameInterval / box.width > 0.1)
            break;
    }
    newBox.height = k - newBox.y;
    // 3. left shrink
    for (k = minLeftX; k < maxLeftX; k++)
    {
        int j = 0;
        int score = 0;
        for (j = box.y; j < newBox.y + newBox.height; j++)
            if (image.at<uchar>(j, k)) //& image.at<uchar>(j+1, k))
                score++;
        if (score * 1.0 * m_takeFrameInterval / newBox.height > 0.1)
            break;
    }
    newBox.x = k;
    
    // 4. right
    for (k = maxRightX; k > minRightX; k--)
    {
        int j = 0;
        int score = 0;
        for (j = newBox.y; j < newBox.y + newBox.height; j++)
        {   
            if (image.at<uchar>(j, k)) // & image.at<uchar>(j+1, k))
                score++; // find the boundary.
        }
        if (score * 1.0 * m_takeFrameInterval / newBox.height > 0.1)
            break;
    }
    newBox.width = k - newBox.x;
    // update
    box = newBox;
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
// trival ones    
// check box close to which boundary (should take skipTB,LR into account)    
vector<MOVING_DIRECTION> ContourTrack :: checkBoxApproachingBoundary(const cv::Rect & rect)
{
    vector<MOVING_DIRECTION> directions;
    // TODO: magic number APPROCHING_DISTANCE should be eliminated    
    static const int APPROCHING_DISTANCE = 4;
    if (rect.x <= m_skipLR + APPROCHING_DISTANCE)
        directions.push_back(LEFT);
    if (rect.x + rect.width >= m_imgWidth - m_skipLR - APPROCHING_DISTANCE)
        directions.push_back(RIGHT);        
    if (rect.y <= m_skipTB + APPROCHING_DISTANCE)
        directions.push_back(TOP);
    if (rect.y + rect.height >= m_imgHeight - m_skipTB - APPROCHING_DISTANCE)
        directions.push_back(BOTTOM);

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
    const int dv = bCrossIn ? 4 : -32; // TODO: should using other info to infer this value.
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
    
int ContourTrack :: getConsumeResult(const vector<int> & results)
{
    int result = (int)CONSUME_NOTHING;
    for (int k=0; k < (int)results.size(); k++)
    {
        result |= results[k];
        //LogD("%d,\n", 0 | 2 | 4);
        //LogD("result in/out: %d, %d.\n", result, CONSUME_IN_LINE | CONSUME_OUT_LINE);
    }
    return result;
}

int ContourTrack :: doStatusChanging(const int statusResult)
{
    LogD("--><--- frame %d statusResult: %d of tracker %d, %s.\n",
         m_inputFrames, statusResult, m_idx, getMovingStatusStr(m_movingStatus));
    dumpRect(m_curBox);
    
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
        if (m_movingStatus == MOVING_CROSS_IN)
            m_allInCount++; // In/Out simultaneously
        else if (m_movingStatus == MOVING_INSIDE)
            m_crossOutCount++;
        break;
    case ((int)CONSUME_IN_LINE | (int)CONSUME_OUT_LINE):
        if (m_movingStatus == MOVING_CROSS_IN)
            LogW("Object cross In & Out simultaneously.\n");
        //cross IN/OUT simultaneously
        break;
    }

    if (m_allInCount >= m_movingInStatusChangingThreshold &&
        m_movingStatus == MOVING_CROSS_IN)
    {
        m_movingStatus = MOVING_INSIDE; // reset lastLines
        for (int k=0; k < (int)BORDER_NUM; k++)
            m_lastBoundaryLines[k] = TDLine();
    }
    else if (m_crossOutCount >= m_movingOutStatusChangingThreshold &&
             m_movingStatus == MOVING_INSIDE)
        m_movingStatus = MOVING_CROSS_OUT;
    else if (m_allOutCount >= m_movingOutStatusChangingThreshold &&
             m_movingStatus == MOVING_CROSS_OUT)
        m_movingStatus = MOVING_FINISH;
    
    return 0;
}

// For using of Updating CTTracker & Retain An Accurate CurBox
double ContourTrack :: getEffectivenessOfBox(const cv::Mat & image, const cv::Rect & box)
{
    int actives = 0;
    for (int k=box.y; k < box.y + box.height; k++)
    {
        for (int j=box.x; j < box.x + box.width; j++)
            if (image.at<uchar>(k, j))
                actives++;
    }
    return actives * 1.0 / (m_curBox.width * m_curBox.height);
}
    
int ContourTrack :: adjustBoxByBgResult(BgResult & bgResult, cv::Rect & baseBox,
                                        const int maxEnlargeDx, const int maxEnlargeDy,
                                        const int maxShrinkDx, const int maxShrinkDy)
{
    doEnlargeBoxUsingImage(bgResult.binaryData, baseBox, maxEnlargeDx, maxEnlargeDy);
    doShrinkBoxUsingImage(bgResult.binaryData, baseBox, maxShrinkDx, maxShrinkDy);
    return 0;
}
    
int ContourTrack :: doBoxProtectionCalibrate(cv::Rect & box)
{
    if (box.width < 32)
        box.width = 32;
    if (box.height < 32)
        box.height = 32;

    // TODO: Seems CTTracker has max tracking height/width, WTF. Fix it later.
    // -16 is because of CTTracker's bug
    boundBoxByMaxBox(box, cv::Rect(0, 0, m_imgWidth - 16, m_imgHeight - 16));    
    // TODO: this is for CTTracker's bug, I would fix it later.
    if (box.width > m_imgWidth - m_imgWidth * 0.2)
        box.width = m_imgWidth - m_imgWidth * 0.2;
    if (box.height > m_imgHeight - m_imgHeight * 0.2)
        box.height = m_imgHeight - m_imgHeight * 0.2;
    
    return 0;
}
    
} // namespace Seg_Three

/////////////////////////// End of The File //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
