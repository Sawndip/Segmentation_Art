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
    // for (int k = 0; k < 2; k++)
    {   
        // close: dilate then erode
        doDilate(4); // 8 pixels
        doErode(4);
        // oepn: erode then dilate
        doErode();
        doDilate();            
    }
    
    // 3. scan the boundary, get the TDPoint of the lines
    scanBoundaryLines(bgResult);
    // 4. do analyse those lines & do pre-merge
    premergeLines(bgResult);

    // 5. cache the first two frames. Won't put Line Result to BgResult
    if (m_inputFrames < M_BOUNDARY_SCAN_CACHE_LINES - 1)
    {
        LogI("Do Caching, FrameNo %d. Won't output LineAnalyse Result to BgResult.\n",
            m_inputFrames);
        return 0;
    }
    
    // 6. do further merge using cacheLines and mark 'bTraced' & 'previousLine' of
    //    consecutive lines.
    stableAnalyseAndMarkLineStatus();
    
    
    // 9. finally, update the curFrontIdx & according 
    curFrontIdx++;
    if (curFrontIdx >= M_BOUNDARY_SCAN_CACHE_LINES)
        curFrontIdx = 0;
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Important Internal Helpers
int BoundaryScan :: scanBoundaryLines(const BgResult & bgResult)
{
    // we get borders with erode/dilate, then we get the foreground bgResult.lines.
    LogI(" **** Scan the border %d times, curFrontIdx %d.\n", m_inputFrames, m_curFrontIdx);
    vector<vector<TDLine> > & cacheFramelines = m_cacheLines[m_curFrontIdx];
    lines.clear(); // important reset it here
    lines.resize(BORDER_NUM);
    
    int width = m_bordersMem.widthTB;
    for (int index = 0; index < BORDER_NUM; index++)
    {
        vector<double> & xMvs = bgResult.xMvs[index];
        vector<double> & yMvs = bgResult.yMvs[index];        
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
                cacheFramelines[index].push_back(line);
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
 a. angle is similar  b. gap between lines not too large
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
        vector<double> & xMvs = bgResult.xMvs[index];
        vector<double> & yMvs = bgResult.yMvs[index];
        
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
                    LogD("Merge border'%s' two lines: %d-%d(%.2f) AND %d-%d(%.2f).\n",
                         getMovingDirectionStr((MOVING_DIRECTION)index),
                         it->a.x, it->b.x, it->movingAngle,
                         nextIt->a.x, nextIt->b.x,nextIt->movingAngle);                
                    it->b = nextIt->b;
                    it->movingAngle = getLineMoveAngle(*it, xMvs, yMvs);
                    oneBoundaryLines.erase(nextIt);
                    LogD("---- Merged New Line is %d-%d(%.2f).\n",
                         it->a.x , it->b.x, it->movingAngle);
                }
                else if (ret == 2)
                {
                    LogD("Merge border %s three lines: %d-%d(%.2f) AND %d-%d(%.2f), and"
                         " disturbing line: %d-%d(%.2f).\n",
                         getMovingDirectionStr((MOVING_DIRECTION)index),
                         it->a.x, it->b.x, it->movingAngle,
                         lookAheadIt->a.x, lookAheadIt->b.x, lookAheadIt->movingAngle,
                         nextIt->a.x, nextIt->b.x,nextIt->movingAngle);
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
// if 1. xDistance is less than 5% of the imgWidth + imgHeight
//    2. xDistance is less than 20% of the 'line1's length + line2's length'
//    3. mvs are quit close (angle less than 20 degree(pi/180*20 arc)): weight 60;
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
    if (1.0 * xDistance / (m_imgWidth + m_imgHeight) > 0.05)
    {
        LogD("TestingMerge1 Fail for large distance: "
             "xDis:%d, line1:%d, line2:%d imgW:%d, imgHeight:%d.\n",
            xDistance, line1len, line2len, m_imgWidth, m_imgHeight);
        return 0;
    }
    if (1.0 * xDistance / (line1len + line2len) > 0.5)
    {
        LogD("TestMerge2 Fail: big gap. "
             "xDis:%d, line1:%d, line2:%d.\n", xDistance, line1len, line2len);
        return 0;
    }
    // 2. check whether the line have the same angle of movement.
    LogD("TestMerge3: angle1 %.2f, angel2: %.2f.\n", l1.movingAngle, l2.movingAngle);
    if (isLineCloseEnough(fabs(l1.movingAngle - l2.movingAngle)) == true)
        return 1;

    // 3. disturbing short line merging: l3 is only used here
    if (l2.a.x == l3.a.x) // l2, l3 are the same, we hit the end.
        return 0;
    bool bClose = isLineCloseEnough(fabs(l1.movingAngle - l3.movingAngle));
    if (bClose && l2.b.x - l1.a.x <= 32) // 
        return 2; // l2 is a distrubing line
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

int BoundaryScan :: outputLineAnalyseResult(BgResult & bgResult)
{   
    vector<TDLine> & lines = bgResult.lines[index];
    if (lines.size() == 0)
        return 0;

    LogI("=== Boundary Info Of This Frame ====\n");
    for (int k = 0; k < (int)m_cacheLines[m_curFrontIdx].size(); k++)
    {   // in arc: [-pi, pi]
        const double angle = getLineMoveAngle(lines[k], xMvs, yMvs);
        calcLineMovingStatus(angle, index, lines[k]);
        LogD("BorderNum: %d. Start: %d, End: %d, angle:%.2f, direction:%s.\n",
             k, lines[k].a.x, lines[k].b.x, lines[k].movingAngle,
             getMovingDirectionStr(lines[k].movingDirection));
    }
    return 0;
}

int BoundaryScan :: updateLineMovingStatus(const double angle,
                                           const int index, TDLine & line)
{      
    line.movingAngle = angle;
    line.movingDirection = (MOVING_DIRECTION)index;
    switch(index) // tend to moving in
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
        LogE("Impossible Direction n=%d.", index);
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
    
} // namespace Seg_Three
