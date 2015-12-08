#include "boundaryScan.h"

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// constructor / destructor / init
BoundaryScan :: BoundaryScan()
{
    return;
}

BoundaryScan :: ~BoundaryScan()
{
    return;        
}

int BoundaryScan :: init(const int width, const int height,
                         const int skipTB, const int skipLR,
                         const int scanSizeTB, const int scanSizeLR)
{
    m_imgWidth = width;
    m_imgHeight = height;
    m_inputFrames = 0;
    m_skipTB = skipTB;
    m_skipLR = skipLR;
    m_scanSizeTB = scanSizeTB;
    m_scanSizeLR = scanSizeLR;
    //normaly heightTB=heightLR=2, widthTB=imgWidth, widthLR=imgHeight
    m_bordersMem.init(m_imgWidth - 2 * skipTB, scanSizeTB,
                      m_imgHeight - 2 * skipLR, scanSizeLR);
    // for caching part
    m_curFrontIdx = 0;
    for (int k=0; k < M_BOUNDARY_SCAN_CACHE_LINES; k++)
        m_cacheLines[k].resize(BORDER_NUM);
    return 0;    
}

//////////////////////////////////////////////////////////////////////////////////////////
//// APIs

/***************************************************************************
processFrame:
   do simplified Erode/dilate of two lines: 2x2 window slide the lines; 
   get the lines that are foreground across the boundary.
args:
   in: the binary(0 or 255) of the background / foreground values of the image
   lines: output result, the foreground lines that just in borders
return:   
****************************************************************************/
int BoundaryScan :: processFrame(BgResult & bgResult)
{
    m_inputFrames++;
    // 0. prepare
    assert(bgResult.binaryData.channels() == 1);
    assert((int)bgResult.binaryData.step[0] == m_imgWidth &&
           (int)bgResult.binaryData.step[1] == (int)sizeof(unsigned char));
    assert(m_scanSizeTB == m_bordersMem.heightTB &&
           m_scanSizeLR == m_bordersMem.heightLR );
    // 1. first extract border data from bgResult
    for (int k = 0; k < m_scanSizeTB; k++)
    {   // top & bottom data
        memcpy(m_bordersMem.directions[0] + k*m_bordersMem.widthTB, // top
               bgResult.binaryData.data + m_imgWidth * (k+m_skipTB) + m_skipLR, 
               m_imgWidth - 2*m_skipLR); 
        memcpy(m_bordersMem.directions[1] + k*m_bordersMem.widthTB, // bottom
               bgResult.binaryData.data + m_imgWidth*(m_imgHeight-m_skipTB-k-1) + m_skipLR,
               m_imgWidth - 2*m_skipLR);
    }
    // left & right data
    for (int k = 0; k < m_scanSizeLR; k++)
    {
        for (int j = 0; j < m_bordersMem.widthLR; j++)
        {   
            m_bordersMem.directions[2][k*m_bordersMem.widthLR+j] = // left
                bgResult.binaryData.at<uchar>(j+m_skipTB, k+m_skipLR);
            m_bordersMem.directions[3][k*m_bordersMem.widthLR+j] = // right
                bgResult.binaryData.at<uchar>(j+m_skipTB, m_imgWidth-m_skipLR-k-1);
        }
    }
    // 2. we do open / close: seems for simplified erode/dilate, just open is ok.    
    //for (int k = 0; k < 2; k++)
    {   
        // close: dilate then erode
        doDilate(2);
        doErode(2);
        //// oepn: erode then dilate
        doErode(2);
        doDilate(2);
        //// close again
        doDilate(2);
        doErode(2);
    }
    
    // 3. scan the boundary, get the TDPoint of the lines
    scanBoundaryLines(bgResult);
    // 4. do analyse those lines & do pre-merge (in one frame & one border line level)
    premergeLines(bgResult);

    // 5. cache the first two frames. Won't put Line Result to BgResult
    if (m_inputFrames < M_BOUNDARY_SCAN_CACHE_LINES)
    {
        LogI("Do Caching, FrameNo %d. Won't output LineAnalyse Result to BgResult.\n",
            m_inputFrames);
        m_curFrontIdx++;
        return 0;
    }
    
    // 6. do further merge using cacheLines and mark 'mayPreviousLine' of
    //    consecutive lines (three frames level)
    stableAnalyseAndMarkLineStatus(bgResult);    
    
    // 7. finally, update the curFrontIdx & according
    outputLineAnalyseResultAndUpdate(bgResult);
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Important Internal Helpers
int BoundaryScan :: scanBoundaryLines(const BgResult & bgResult)
{
    // we get borders with erode/dilate, then we get the foreground bgResult.lines.
    LogI(" **** Scan the border %d times, curFrontIdx %d.\n", m_inputFrames, m_curFrontIdx);
    vector<vector<TDLine> > & cacheOneFramelines = m_cacheLines[m_curFrontIdx];
    cacheOneFramelines.clear(); // important reset it here
    cacheOneFramelines.resize(BORDER_NUM);
    
    int width = m_bordersMem.widthTB;
    for (int index = 0; index < BORDER_NUM; index++)
    {
        const vector<double> & xMvs = bgResult.xMvs[index];
        const vector<double> & yMvs = bgResult.yMvs[index];        
        if (index >= 2) // for left/right borders
            width = m_bordersMem.widthLR;

        TDLine line;
        bool bStart = false;
        for (int k = 0; k < width; k++)
        {
            if (bStart == false && m_bordersMem.directions[index][k] == 0xFF)
            {
                bStart = true;
                line.a.x = k;
                line.a.y = 0;
            }
            if (bStart == true && m_bordersMem.directions[index][k] != 0xFF)
            {
                bStart = false;
                line.b.x = k;
                line.b.y = 0;
                line.movingAngle = getLineMoveAngle(line, xMvs, yMvs);
                cacheOneFramelines[index].push_back(line);
                LogD("Get One '%s' Line, %d-%d(%.2f).\n",
                     getMovingDirectionStr((MOVING_DIRECTION)index),
                     line.a.x, line.b.x, line.movingAngle);
                bStart = false;
            }
        }
    }
    return 0;
}

/***************    
 1. merge short-lines that we can be sure they are parts of the same objects.
 2. For lines we process here are lines with no overlap, namely, l1.b.x < l2.a.x.
    So we can make use of this property.
 3. we will look ahead two lines, that some disturbing short lines can be smoothed out
    for instance, line1 (15, 300, -2.5arc), line2 (306-312, 1.2arc), line3 (320,400, -2.93arc)
    line1 & line2's angle is quit different, but line1 & line3 have quit similar degre and
    line2's length is short, so we take line2 as a distrubing line and will merge line1,2,3.
 In sum: pre-merge do three kinds of tesing to merge:
 a. angle is similar  
 b. gap between lines not too large
 c. looking ahead next two lines to merge disturing short-line. 
***************/     
int BoundaryScan :: premergeLines(const BgResult & bgResult)
{
    vector<vector<TDLine> > & cacheFramelines = m_cacheLines[m_curFrontIdx];    
    for (int index = 0; index < BORDER_NUM; index++)
    {
        vector<TDLine> & oneBoundaryLines = cacheFramelines[index];
        if (oneBoundaryLines.size() < 2) // no need merging
            continue;
        const vector<double> & xMvs = bgResult.xMvs[index];
        const vector<double> & yMvs = bgResult.yMvs[index];
        
        for (auto it = oneBoundaryLines.begin(); it != oneBoundaryLines.end(); /*No increment*/)
        {
            auto nextIt = it + 1;
            if (nextIt == oneBoundaryLines.end())
                break;
            else
            {
                auto lookAheadIt = nextIt + 1; // for merge disturbing short line.
                if (lookAheadIt == oneBoundaryLines.end())
                    lookAheadIt = nextIt;            

                const int ret = canLinesBeMerged(*it, *nextIt, *lookAheadIt);
                if (ret == 1)
                {
                    it->b = nextIt->b;
                    it->movingAngle = getLineMoveAngle(*it, xMvs, yMvs);
                    oneBoundaryLines.erase(nextIt);
                    LogD("---- Merged New Line is %d-%d(%.2f).\n",
                         it->a.x, it->b.x, it->movingAngle);
                }
                else if (ret == 2)
                {
                    it->b = lookAheadIt->b;
                    it->movingAngle = getLineMoveAngle(*it, xMvs, yMvs);                    
                    oneBoundaryLines.erase(lookAheadIt);
                    oneBoundaryLines.erase(nextIt);
                    LogD("---- Merged New Line is %d-%d(%.2f).\n",
                         it->a.x , it->b.x, it->movingAngle);
                }
                else
                {
                    LogD("Can Not Merge Line %d-%d(%.2f) AND Line %d-%d(%.2f).\n",
                         it->a.x, it->b.x, it->movingAngle,
                         nextIt->a.x , nextIt->b.x, nextIt->movingAngle);
                    it++;
                }
            }
        }
    }
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
// using mvs & also the line's x direction distance    
// if 1. xDistance is less than 20% of the imgWidth + imgHeight
//    2. xDistance is less than 50% of the 'line1's length + line2's length'
//    3. mvs are quit close (angle less than 20 degree(pi/2 arc))
// then we can merge this two lines.
// return: 0: no merge; 1:merge one line; 2:merge two lines.
// Note: lines' angle would be calculated and assign in this call.    
int BoundaryScan :: canLinesBeMerged(const TDLine & l1, const TDLine & l2, const TDLine & l3)
{
    assert(l1.b.x < l2.a.x); // end's point < start's point.    
    // 1. gaps between two lines should be relatively small
    const int xDistance = l2.a.x - l1.b.x;
    const int line1len = l1.b.x - l1.a.x;
    const int line2len = l2.b.x - l2.a.x;
    // TODO: magic number here
    if (1.0 * xDistance / (m_imgWidth + m_imgHeight) > 0.2)
    {
        LogD("TestingMerge1 Fail for large distance: "
             "xDis:%d, line1:%d, line2:%d imgW:%d, imgHeight:%d.\n",
            xDistance, line1len, line2len, m_imgWidth, m_imgHeight);
        return 0;
    }
    // bigger than 100 pixels is big enough ?
    if (xDistance > 100 && (1.0 * xDistance / (line1len + line2len) > 0.5))
    {
        LogD("TestMerge2 Fail: big gap. "
             "xDis:%d, line1:%d, line2:%d.\n", xDistance, line1len, line2len);
        return 0;
    }
    // 2. check whether the line have the same angle of movement.
    LogD("TestMerge3: angle1 %.2f, angel2: %.2f.\n", l1.movingAngle, l2.movingAngle);
    if (isLineCloseEnough(fabs(l1.movingAngle - l2.movingAngle)) == true)
        return 1;

    // 3. for quit small lines & small gaps, we merge them
    if (line1len <= 16 && xDistance <= 16) // one block
        return 1;
    
    // 4. disturbing short line merging: l3 is only used here
    if (l2.a.x == l3.a.x) // l2, l3 are the same, we hit the end.
        return 0;
    bool bClose = isLineCloseEnough(fabs(l1.movingAngle - l3.movingAngle));
    // TODO: magic number here: not matter, for we will use the merged one as the base line
    //       to do futher merging.
    if (bClose && l2.b.x - l1.a.x <= 32) // 
        return 2; // l2 is a distrubing line
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// ***** Five Star Important    
// 1. using cacheLines to further merge lines
// 2. kick out disturbing lines (merge adjacent lines)
// 3. correct line's MOVING_DIRECTION (using angle and its predecessor & successor)
// 3. do line status marking (final output result of the BoundaryScan)
// 4. Eager Strategy: two out of three are treat as the LINE.
//    Conservative Strategy: all three lines are the "same".
//    We use 'Conservative Strategy'
int BoundaryScan :: stableAnalyseAndMarkLineStatus(BgResult & bgResult)
{   
    // Focus on m_curFrontIdx's caching frame lines. It will be the output result.
    // Take use of all M_BOUNDARY_SCAN_CACHE_LINES=3 frames to do stable analyse.
    const int old = loopIndex(m_curFrontIdx, M_BOUNDARY_SCAN_CACHE_LINES);
    const int middle = loopIndex(old, M_BOUNDARY_SCAN_CACHE_LINES);
    vector<vector<TDLine> > & oldLiness = m_cacheLines[old];
    vector<vector<TDLine> > & middleLiness = m_cacheLines[middle];
    vector<vector<TDLine> > & curLiness = m_cacheLines[m_curFrontIdx];
    assert(curLiness.size() == BORDER_NUM);
    assert(oldLiness.size() == middleLiness.size() && oldLiness.size() == curLiness.size());
    
    for (int bdNum = 0; bdNum < BORDER_NUM; bdNum++)
        if (curLiness[bdNum].size() > 0)
            markPredecessorsRecursively(0, bdNum, bgResult, curLiness[bdNum],
                                        middleLiness[bdNum], oldLiness[bdNum]);
    return 0;
}

int BoundaryScan :: markPredecessorsRecursively(const int curIdx, const int bdNum,
                                                BgResult & bgResult,
                                                vector<TDLine> & curLines,
                                                vector<TDLine> & middleLines,
                                                vector<TDLine> & oldLines)
{
    if (curIdx >= (int)curLines.size())
        return 0; // base: finish the process.    
    goMarking(bdNum, bgResult, curLines[curIdx], middleLines, oldLines);
    mergeOverlapOfOnePositionLines(curLines, curIdx);
    return markPredecessorsRecursively(curIdx+1, bdNum, bgResult,
                                       curLines, middleLines, oldLines);
}
    
int BoundaryScan :: goMarking(const int bdNum, BgResult & bgResult,
                              TDLine & curLine, vector<TDLine> & middleLines,
                              vector<TDLine> & oldLines)
{   // TODO: magic number 60: 70 ?
    // 1. check the middle ones' start point(left point)
    int middleMaxIdx = -1;
    double middleMaxScore = -1.0;
    for (int k = 0; k < (int)middleLines.size(); k++)
    {   // one of the following score is 0.0, the total score will be 100
        double score = leftConsecutivityOfTwoLines(curLine, middleLines[k], 60, true);
        score += rightConsecutivityOfTwoLines(curLine, middleLines[k], 60, true);        
        if (score > middleMaxScore)
        {
            middleMaxScore = score;
            middleMaxIdx = k;
        }
    }

    if (middleMaxScore <= 60.0)
        return -1; // doesn't need to mark any lines

    // 2. check the old ones' start point(left point)
    int oldMaxIdx = -1;
    double oldMaxScore = -1;
    for (int k = 0; k < (int)oldLines.size(); k++)
    {
        double score = leftConsecutivityOfTwoLines(middleLines[middleMaxIdx],
                                                         oldLines[k], 60, true);
        score += rightConsecutivityOfTwoLines(middleLines[middleMaxIdx],
                                                         oldLines[k], 60, true);
        if (score > oldMaxScore)
        {
            oldMaxScore = score;
            oldMaxIdx = k;
        }
    }

    if (oldMaxScore < 60.0)
        return -1; // doesn't need to mark any lines

    // 3. NOW OK: find the start points of all three lines, lets do marking & extending.
    //    For old ones will be replaced by new coming frame soon, so we don't mark the olds, but
    //    olds ones will affect the middle lines' merging.
    if (middleLines[middleMaxIdx].b.x < oldLines[oldMaxIdx].b.x)
        middleLines[middleMaxIdx].b = oldLines[oldMaxIdx].b;

    // calc the new points of curLine
    if (curLine.b.x < middleLines[middleMaxIdx].b.x)
        curLine.b = middleLines[middleMaxIdx].b;
        
    // 4. do other update needed
     // curLines will be merge after this call.
    mergeOverlapOfOnePositionLines(middleLines, middleMaxIdx);
    curLine.mayPreviousLineStart = middleLines[middleMaxIdx].a;
    curLine.mayPreviousLineEnd = middleLines[middleMaxIdx].b;

    const double averageAngle = (curLine.movingAngle +
                                 middleLines[middleMaxIdx].movingAngle +
                                 oldLines[oldMaxIdx].movingAngle) / 3.0;
    
    const vector<double> & xMvs = bgResult.xMvs[bdNum];
    const vector<double> & yMvs = bgResult.yMvs[bdNum];
    const double updateAngle = getLineMoveAngle(curLine, xMvs, yMvs);
    
    curLine.movingAngle =
        middleLines[middleMaxIdx].movingAngle = (averageAngle + updateAngle) / 2;
    // update moving status by angle & bdNum
    calcLineMovingStatus(bdNum, curLine);
    if (middleLines[middleMaxIdx].bValid == true)
        if (middleLines[middleMaxIdx].movingStatus != curLine.movingStatus)
            LogW("Predecessor But not the same moving status!!!. previous %s now %s.\n",
                 getMovingStatusStr(middleLines[middleMaxIdx].movingStatus),
                 getMovingStatusStr(curLine.movingStatus));
    
    // this is a valid line until now.
    curLine.bValid = true;
    
    LogD("Valid Line: cur: %d-%d, previous: %d-%d. "
         "angle: %.2f(old %.2f, average %.2f, update %.2f), status: %s.\n",
         curLine.a.x, curLine.b.x,
         middleLines[middleMaxIdx].a.x, middleLines[middleMaxIdx].b.x, 
         curLine.movingAngle, oldLines[oldMaxIdx].movingAngle, averageAngle, updateAngle,
         getMovingStatusStr(curLine.movingStatus));
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// Trival Inner Helpers    
// return [-pi, pi]
double BoundaryScan :: getLineMoveAngle(const TDLine & l1, const vector<double> & xMvs,
                                        const vector<double> & yMvs)
{
    double xMv = 0.0, yMv = 0.0;
    for (int k = l1.a.x; k <= l1.b.x; k++)
    {
        xMv += xMvs[k];
        yMv += yMvs[k];
    }
    return atan2(yMv, xMv);
}

int BoundaryScan :: outputLineAnalyseResultAndUpdate(BgResult & bgResult)
{   
    // Output curFrontIdx's frame lines as the output result.
    for (int bdNum = 0; bdNum < (int)m_cacheLines[m_curFrontIdx].size(); bdNum++)
        bgResult.resultLines[bdNum] = m_cacheLines[m_curFrontIdx][bdNum];

    m_curFrontIdx = loopIndex(m_curFrontIdx, M_BOUNDARY_SCAN_CACHE_LINES);
    return 0;
}

int BoundaryScan :: calcLineMovingStatus(const int bdNum, TDLine & line)
{      
    const double angle = line.movingAngle;
    line.movingDirection = (MOVING_DIRECTION)bdNum;
    // NOTE: it is not accurate to using Angle to determine the MOVING_STATUS
    switch(bdNum) // tend to moving in
    {
    case 0: // top
        line.movingStatus = angle > 0 ?  MOVING_CROSS_OUT : MOVING_CROSS_IN;
        break;
    case 1: // bottom
        line.movingStatus = angle >= 0 ?  MOVING_CROSS_IN : MOVING_CROSS_OUT;
        break;
    case 2: // left
        line.movingStatus = fabs(angle) <= (M_PI / 2) ? MOVING_CROSS_IN : MOVING_CROSS_OUT;
        break;
    case 3: // right
        line.movingStatus = fabs(angle) < (M_PI / 2) ? MOVING_CROSS_OUT : MOVING_CROSS_IN;
        break;
    default:
        LogE("Impossible Direction n=%d.\n", bdNum);
    }
    return 0;
}

//simplified Erode/dilate
int BoundaryScan :: doErode(const int times)
{
    // NOTE: following code just deal with 2x2 window! Be aware of it.
    for (int t = 0; t < times; t++)
    {
        int width = m_bordersMem.widthTB, height = m_bordersMem.heightTB;
        for (int n = 0; n < 4; n++)
        {   // note k = k + M_ELEMENT_HEIGHT
            if (n >= 2)
            {
                width = m_bordersMem.widthLR;            
                height = m_bordersMem.heightLR;
            }
         
            for (int k = 0; k < height; k+=M_ELEMENT_HEIGHT)
            {
                for (int j = 0; j < width - 1; j++)
                {
                    m_bordersMem.directions[n][k*width + j] =
                      m_bordersMem.directions[n][(k+1)*width + j] =
                        m_bordersMem.directions[n][k*width + j]     &
                        m_bordersMem.directions[n][(k+1)*width + j] &
                        m_bordersMem.directions[n][k*width + j+1]   & 
                        m_bordersMem.directions[n][(k+1)*width + j+1];
                    if (j == width - M_ELEMENT_WIDTH)
                        m_bordersMem.directions[n][k*width + j+1] =
                            m_bordersMem.directions[n][(k+1)*width + j+1] =
                                m_bordersMem.directions[n][k*width + j+1]   &
                                m_bordersMem.directions[n][(k+1)*width + j+1];
                    //if (m_bordersMem.directions[n][k*width + j] == 0xFF)
                    //    LogI("One point 255, x=%d, y=%d\n", j, k);
                }
            }
        }
    }
    return 0;
}

int BoundaryScan :: doDilate(const int times)
{
    for (int t = 0; t < times; t++)
    {
        int width = m_bordersMem.widthTB, height = m_bordersMem.heightTB;        
        for (int n = 0; n < 4; n++)
        {   // note k = k + M_ELEMENT_HEIGHT
            if (n >= 2)
            {
                height = m_bordersMem.heightLR;
                width = m_bordersMem.widthLR;
            }
            for (int k = 0; k < height; k+=M_ELEMENT_HEIGHT)
            {
                for (int j = 0; j < width - 1; j++)
                {
                    m_bordersMem.directions[n][k*width + j] =
                      m_bordersMem.directions[n][(k+1)*width + j] =
                        m_bordersMem.directions[n][k*width + j]     |
                        m_bordersMem.directions[n][(k+1)*width + j] |
                        m_bordersMem.directions[n][k*width + j+1]   | 
                        m_bordersMem.directions[n][(k+1)*width + j+1];
                    if (j == width - M_ELEMENT_WIDTH)
                        m_bordersMem.directions[n][k*width + j+1] =
                            m_bordersMem.directions[n][(k+1)*width + j+1] =
                                m_bordersMem.directions[n][k*width + j+1]   |
                                m_bordersMem.directions[n][(k+1)*width + j+1];
                }
            }
        }
    }
    return 0;
}

// merge overlap points at certain position (extending)
int BoundaryScan :: mergeOverlapOfOnePositionLines(vector<TDLine> & lines, const int curIdx)
{
    for (auto it = lines.begin() + curIdx; it != lines.end(); it++)
    {
        auto nextIt = it + 1;
        while(nextIt != lines.end())
        {
            if (it->b.x >= nextIt->a.x)
            {
                nextIt++;
                continue;
            }
            else
                break;
        }
        auto mergeIt = nextIt - 1;
        it->b = mergeIt->b;
        for (auto deleteIt = it + 1; deleteIt <= mergeIt && deleteIt != lines.end();
                                                                     /*No Increament Here*/)
        {
            deleteIt = lines.erase(deleteIt);
        }
    }
    return 0;
}

} // namespace Seg_Three
