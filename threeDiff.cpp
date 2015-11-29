#include <algorithm> // std::sort
#include <tuple> 
#include "threeDiff.h"

using namespace cv;

namespace Seg_Three
{
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//// 1. constructor / destructor / init
ThreeDiff :: ThreeDiff()
{
    m_bInit = false;
    return;
}

ThreeDiff :: ~ThreeDiff()
{
    return;        
}

int ThreeDiff :: init(const int width, const int height,
                      const int skipTB, const int skipLR,
                      const int scanSizeTB, const int scanSizeLR)
{
    if (m_bInit == false)
    {
        // general
        m_imgWidth = width;
        m_imgHeight = height;
        m_inputFrames = 0;
        m_skipTB = skipTB;
        m_skipLR = skipLR;
        m_scanSizeTB = scanSizeTB;
        m_scanSizeLR = scanSizeLR;
        
        // cache part
        m_curFrontIdx = 0;
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_bgResults[k].binaryData.create(height, width, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffAndResults[k].create(height, width, CV_8UC1); // gray
        for (int k = 0; k < M_THREE_DIFF_CACHE_FRAMES; k++)
            m_diffOrResults[k].create(height, width, CV_8UC1); // gray
        // crossLines won't be init here.
        // ContourTrack
        m_objIdx = 0;        
        m_bInit = true;
    }
    return 0;    
}

//////////////////////////////////////////////////////////////////////////////////////////
//// 2. APIs
// |><| ************************************************************************
// processFrame:
//     1. Do diff(OR operation) of two frames, store the 'diffResults';
//     2. Do AND operation of 'diffResults' to get the overlap;
//     3. using crossLines(boundary) to emit new contourTrack;
//     4. using existing 'contourTrack' with 'diffResults' to update contourTrack;
// args:
//     outs: output each contourTrack's object rectangle;
//     bgResult: psoBg's result of background/foreground, xMvs, yMvs;
// return:
//     = 0, won't output frames;
//     > 0, output one frame;
//     < 0, process error;
// *****************************************************************************
int ThreeDiff :: processFrame(const cv::Mat & in,
                              BgResult & bgResult,
                              vector<SegResults> & segResults)
{
    m_inputFrames++;
    // 0. do preprocess: cache frames
    if (m_inputFrames <= M_THREE_DIFF_CACHE_FRAMES)
    {
        m_bgResults[m_inputFrames-1] = bgResult;
        if (m_inputFrames > 1)
        {   
            doBgDiff(m_bgResults[m_inputFrames-1].binaryData,
                     m_bgResults[m_inputFrames-2].binaryData);
            m_curFrontIdx++;
            if (m_curFrontIdx % M_THREE_DIFF_CACHE_FRAMES == 0)
                m_curFrontIdx = 0;
        }
        return 0;
    }
    
    // 1. do diff in RGB for Contour's using.
    doBgDiff(bgResult.binaryData, m_bgResults[m_curFrontIdx].binaryData);
    // 2. fill the 'outs' with 'lines', 'diffResult', 'simplified optical flow' 
    doUpdateContourTracking(in, bgResult, segResults);   
    // 3. do boundary check for creating new Contour.
    doCreateNewContourTrack(in, bgResult, segResults);
    // 4. do update internal cache/status
    updateAfterOneFrameProcess(in, bgResult);
    
    // output 1 frame
    return 1;
}

int ThreeDiff :: flushFrame(vector<SegResults> & segResults)
{
    // for the cache frames.
    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////
//// 3. Important Inner helpers

// |><| **********************************************************************************
// doUpdateContourTracking:
//     1. Using m_crossLines to check new coming in objects
//     2. line1's(oldest line) TDPoints can be removed in this call
// args:
//     outs: output new contourTrack's object rectangle;
//     curFourLines: this frames ?? 
// return:
//     >= 0, process ok;
//     < 0, process error;
// ***************************************************************************************
int ThreeDiff :: doUpdateContourTracking(const cv::Mat in, BgResult & bgResult,
                                         vector<SegResults> & segResults)
{
    if (m_trackers.size() == 0)
        return 0;
    // 1. leaving boundary check
    // 2. new curBox amendment.
    // 3. do boundary points kicking
    // TODO: how to merge TRACKERS that track the same object.
    for (auto it = m_trackers.begin(); it != m_trackers.end(); /*No it++, do it inside loop*/)
    {
        SegResults sr;
        // re-calc the curBox, calculate the boundary cross part.
        int ret = (*it)->processFrame(in, bgResult, m_diffAndResults[m_curFrontIdx],
                                      m_diffOrResults[m_curFrontIdx]);
        if (ret < 0)
            LogW("Process failed.\n");
        else if (ret == 1) // all out
        {
            // TODO: other updates??
            delete *it; // delete this ContourTrack
            m_trackers.erase(it); // erase it from the vector.
        }
        else // ok, just do post update
        {
            sr.m_objIdx = (*it)->getIdx();
            sr.m_bOutForRecognize = (*it)->canOutputRegion();
            sr.m_curBox = (*it)->getCurBox();
            // kick the points
            if ((*it)->isAllIn() == false)
                kickOverlapPoints(sr.m_curBox, (*it)->getInDirection());
            it++; // increse here.
        }
        segResults.push_back(sr);
    }
        
    return 0;
}
    
// |><| ************************************************************************
// doCreateNewContourTrack:
//     1. Using m_crossLines to check new coming in objects
//     2. only deal with enter objects. Exit objects handle in 'doUpdateContourTracking'.
// args:
//     outs: output new contourTrack's object rectangle;
// return:
//     >= 0, process ok;
//     < 0, process error;
// *****************************************************************************    
int ThreeDiff :: doCreateNewContourTrack(const cv::Mat & in, BgResult & bgResult,
                                         vector<SegResults> & segResults)
{
    const int oldIdx = (m_curFrontIdx - 1) < 0 ?
                        M_THREE_DIFF_CACHE_FRAMES - 1 : m_curFrontIdx - 1;
    // 1. some inLine already be marked as bTrace, so we should first get those Lines out.
    vector<tuple<int, int> > creates; // the (bdNum, index) tuple
    for (int bdNum=0; bdNum < BORDER_NUM; bdNum++)
        for (int k = 0; k < (int)bgResult.lines[bdNum].size(); k++)
            if (bgResult.lines[bdNum][k].bTraced == false)
                if (markCloseLine(bgResult.lines[bdNum][k],
                                  m_bgResults[oldIdx].lines[bdNum],
                                  m_bgResults[m_curFrontIdx].lines[bdNum]) > 0)
                    creates.push_back(std::make_tuple(bdNum, k));

    // 2. now we get the cross lines stand fro new objects, so we just create them.
    for (int k=0; k < (int)creates.size(); k++)
    {
        const int borderDirection = std::get<0>(creates[k]);
        assert(borderDirection >= 0 && borderDirection < 4);
        const int lineIdx = std::get<1>(creates[k]);
        TDLine & theLine = bgResult.lines[borderDirection][lineIdx];
        
        // 1). we calculate the lux/luy, possible width/height
        int lux = 0, luy = 0, possibleWidth = 0, possibleHeight = 0;
        // TODO: should make 2 & 8 param in future.
        switch(borderDirection)
        {
        case 0: // top: enlarge width 4 pixels, each side with 2.
            lux = theLine.a.x - 2 + m_skipLR < 0 ? 0 : theLine.a.x - 2 + m_skipLR;
            luy = 0; // TODO?? what value should be taken?
            possibleWidth = theLine.b.x + 2 - lux > m_imgWidth ?
                                        m_imgWidth : theLine.b.x + 2 - lux;
            possibleHeight = m_skipTB + 8; // make it 8 pixels for all newly created Rect
            break;                    
        case 1: // bottom
            possibleHeight = m_skipTB + 8;
            lux = theLine.a.x - 2 + m_skipLR < 0 ? 0 : theLine.a.x - 2 + m_skipLR;
            luy = m_imgHeight - possibleHeight;
            possibleWidth = theLine.b.x + 2 - lux > m_imgWidth ?
                                        m_imgWidth : theLine.b.x + 2 - lux;
            break;                    
        case 2: // left
            lux = 0;
            luy = theLine.a.x - 2 + m_skipTB < 0 ? 0 : theLine.a.x - 2 + m_skipTB ;
            possibleWidth = m_skipLR + 8;
            possibleHeight = theLine.b.x + 2 - luy > m_imgHeight ?
                                        m_imgHeight : theLine.b.x + 2 - luy;
            break;                    
        case 3: // right
            possibleWidth = m_skipLR + 8;
            lux = m_imgWidth - possibleWidth;
            luy = theLine.a.x - 2 + m_skipTB < 0 ? 0 : theLine.a.x - 2 + m_skipTB;
            possibleHeight = theLine.b.x + 2 - luy > m_imgHeight ?
                                         m_imgHeight : theLine.b.x + 2 - luy;
            break;
        default:
            LogE("impossible to happen, border direction: %d.\n", borderDirection);
            break;
        }
        // 2). now we create the tracker.
        ContourTrack *pTrack = new ContourTrack(m_objIdx, in,
                                                m_imgWidth, m_imgHeight,
                                                k, lux, luy,
                                                possibleWidth, possibleHeight);
        m_trackers.push_back(pTrack);
        // 3). we ouptput the newly created Segmentation. 
        SegResults sr;
        sr.m_objIdx = m_objIdx;
        sr.m_bOutForRecognize = false;
        sr.m_curBox = cv::Rect(lux, luy, possibleWidth, possibleHeight);
        m_objIdx++;
        segResults.push_back(sr);
    }
    
    return 0;
}

int ThreeDiff :: markCloseLine(TDLine & inLine,
                               vector<TDLine> & cacheLines1,
                               vector<TDLine> & cacheLines2)
{
    // 1) very close of start/end points   ---\ then take as close lines.
    // 2) they are movingAngle is similar  ---/
    TDLine * pClose1 = NULL;
    TDLine * pClose2 = NULL;    
    const double score1 = calcCloseLineScore(inLine, cacheLines1, pClose1);
    const double score2 = calcCloseLineScore(inLine, cacheLines2, pClose2);

    if (pClose1 != NULL && pClose2 != NULL)
    {   // may find new, still need to check their scores.
        if ((score1 + score2) / 2 > 60.0) // || score2 > 85) // score1 is the oldest line
        {
            pClose1->bTraced = true;
            pClose2->bTraced = true;
            inLine.bTraced = true;
            return 1; // NOTE: return here.
        }
    }
    return 0;
}
    
// TODO: PXT: to elaborate the score criterion.    
// using score as the close judge criterion:
// 1. moving angle take 50 points.
// 2. start & end TDPoints take 25 points each.
// 3. distance of 0-100 pixels: 25-0 points, others -10 points
double ThreeDiff :: calcCloseLineScore(TDLine & inLine,
                                       vector<TDLine> & cacheLines, TDLine *pClose)
{
    double maxScore = 0.0;
    for (int k = 0; k < (int)cacheLines.size(); k++)
    {   // we only check untraced cross boundary lines.
        if (cacheLines[k].bTraced == false)
        {   // y = -0.25x + 25
            // start point
            int distance = abs(cacheLines[k].a.x - inLine.a.x);
            double score = distance > 100 ? -10 : ((-0.25) * distance + 25);
            // end point
            distance = abs(cacheLines[k].b.x - inLine.b.x);
            score += distance > 100 ? -10 : ((-0.25) * distance + 25);
            // moving angle: y = (-100/PI)x + 50 & y = (100/PI)x - 150
            const double diffAngle = fabs(cacheLines[k].movingAngle - inLine.movingAngle);
            if (diffAngle <= M_PI)
                score += (-100.0 / M_PI) * diffAngle + 50;
            else
                score += (100.0 / M_PI) * diffAngle - 150;
            if (score > maxScore)
            {
                maxScore = score;
                pClose = &cacheLines[k];
            }
        }
    }

    return maxScore;
}

// if an object is just entering the border, we should kick out boundary scan's points
// that inside object's area.
int ThreeDiff :: kickOverlapPoints(const cv::Rect & box, const MOVING_DIRECTION direction)
{
    TDPoint start;
    TDPoint end;
    switch(direction)
    {
    case TOP:
    case BOTTOM:        
        start.x = box.x;
        start.y = 0;
        end.x = box.x + box.width;
        end.y = 0;        
        break;
    case LEFT:
    case RIGHT:        
        start.x = box.y;
        start.y = 0;
        end.x = box.y + box.height;
        end.y = 0;
        break;
    default:
        LogW("We don't know the direction, cannot kick out points.");        
        return -1;
    }

    return 0;
}
    
//////////////////////////////////////////////////////////////////////////////////////////    
//////////////////////////////////////////////////////////////////////////////////////////
//// 4. trival helpers
int ThreeDiff :: doBgDiff(const cv::Mat & first, const cv::Mat & second)
{
    for (int k = 0; k < m_imgHeight; k++)
    {
        for (int j = 0; j < m_imgWidth; j++)
        {
            m_diffAndResults[m_curFrontIdx].at<uchar>(k, j) =
                first.at<uchar>(k, j) & second.at<uchar>(k, j);
            m_diffOrResults[m_curFrontIdx].at<uchar>(k, j) =
                first.at<uchar>(k, j) | second.at<uchar>(k, j);
        }
    }    
    return 0;
}

int ThreeDiff :: updateAfterOneFrameProcess(const cv::Mat in, const BgResult & bgResult)
{ 
    // diff, in, bgResult, crossLines
    m_curFrontIdx++;
    m_curFrontIdx = m_curFrontIdx % M_THREE_DIFF_CACHE_FRAMES == 0 ? 0 : m_curFrontIdx;
    m_bgResults[m_curFrontIdx] = bgResult;
    return 0;
}    
   
} // namespace Seg_Three    
////////////////////////////// End of File //////////////////////////////////////////
