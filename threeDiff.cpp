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
        m_bgResults[m_curFrontIdx] = bgResult;
        if (m_inputFrames > 1) // M_THREE_DIFF_CACHE_FRAMES may not be 2 in future.
        {   
            doBgDiff(m_bgResults[m_inputFrames-1].binaryData,
                     m_bgResults[m_inputFrames-2].binaryData);
            m_curFrontIdx = loopIndex(m_curFrontIdx, M_THREE_DIFF_CACHE_FRAMES);
        }
        return 0;
    }
    
    // 1. do diff in RGB for Contour's using.
    doBgDiff(bgResult.binaryData, m_bgResults[m_curFrontIdx].binaryData);
    // 2. update the trackers status, also dealing with MOVING_CROSS_OUT part.
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
// return:
//     >= 0, process ok;
//     < 0, process error;
// ***************************************************************************************
int ThreeDiff :: doUpdateContourTracking(const cv::Mat in, BgResult & bgResult,
                                         vector<SegResults> & segResults)
{
    if (m_trackers.size() == 0)
        return 0;

    for (auto it = m_trackers.begin(); it != m_trackers.end(); /*No it++, do it inside loop*/)
    {
        // 1. MOVING_INSIDE trackers just do untraced & MOVING_CROSS_OUT checking
        if ((*it)->getMovingStatus() == MOVING_INSIDE)
        {
            cv::Rect curBox = (*it)->getCurBox();
            // check box's possible moving out direction.
            

        }
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
            //// kick the points
            //if ((*it)->isAllIn() == false)
            //    kickOverlapPoints(sr.m_curBox, (*it)->getInDirection());
            //it++; // increse here.
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
    vector<tuple<int, int> > creates; // the (bdNum, index) tuple
    for (int bdNum=0; bdNum < BORDER_NUM; bdNum++)
    {
        for (int k = 0; k < (int)bgResult.resultLines[bdNum].size(); k++)
        {   // untraced ones & MOVING_CROSS_IN ones will be created.
            if (bgResult.resultLines[bdNum][k].bTraced == false &&
                bgResult.resultLines[bdNum][k].movingStatus == MOVING_CROSS_IN)
            {
                // 2. now we get the cross lines stand fro new objects, so we just create them.
                // 1). we calculate the lux/luy, possible width/height
                int lux = 0, luy = 0, possibleWidth = 0, possibleHeight = 0;
                // TODO: should make 2 & 8 param in future.
                switch(bdNum)
                {
                case 0: // top: enlarge width 4 pixels, each side with 2.
                    lux = theLine.a.x - 2 + m_skipLR < 0 ? 0 : theLine.a.x - 2 + m_skipLR;
                    luy = 0; // TODO?? what value should be taken?
                    possibleWidth = theLine.b.x + 2 - lux > m_imgWidth ?
                        m_imgWidth : theLine.b.x + 2 - lux;
                    // make it 8 pixels for all newly created Rect
                    possibleHeight = m_skipTB + 8; 
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
                                                        m_imgWidth, m_imgHeight, theLine,
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
        }
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
    m_curFrontIdx = loopIndex(m_curFrontIdx, M_THREE_DIFF_CACHE_FRAMES);
    m_bgResults[m_curFrontIdx] = bgResult;
    return 0;
}    
   
} // namespace Seg_Three    
////////////////////////////// End of File //////////////////////////////////////////
